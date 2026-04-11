#!/usr/bin/env python3
"""
Phone2Act Planner (Robot-Agnostic Teleoperation Layer)

This node maps phone (AR pose) motion → robot target pose.

Key Features:
- Fully configurable position + rotation mapping (NO code change needed)
- Supports different robot coordinate frames via parameters
- Safety filtering + workspace clamping
- Real-time safe (precomputed mapping functions)

Author: Om Mandhane
"""

import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32MultiArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Phone2ActPlanner(Node):
    def __init__(self):
        super().__init__('phone2act_planner')

        # =========================
        # 🔧 CONFIGURABLE PARAMETERS
        # =========================

        # Scaling
        self.declare_parameter('scale_pos', 900.0)
        self.declare_parameter('scale_rot', 1.0)

        # Safety
        self.declare_parameter('safe_limit_jump', 75.0)  # mm
        self.declare_parameter('cmd_rate_hz', 45.0)

        # Workspace limits
        self.declare_parameter('limits.x', [-800.0, 0.0])
        self.declare_parameter('limits.y', [-500.0, 500.0])
        self.declare_parameter('limits.z', [13.0, 750.0])

        # Position mapping (axis + sign)
        self.declare_parameter('mapping.position.x', "-z")
        self.declare_parameter('mapping.position.y', "-x")
        self.declare_parameter('mapping.position.z', "+y")

        # Rotation mapping
        self.declare_parameter('mapping.rotation.rx', "+pitch")
        self.declare_parameter('mapping.rotation.ry', "+yaw")
        self.declare_parameter('mapping.rotation.rz', "+roll")

        # =========================
        # ⚙️ INTERNAL STATE
        # =========================

        self.robot_pose = None
        self.initial_robot_pose = None
        self.initial_phone_pose = None
        self.initial_phone_euler = None

        self.clutch_engaged = True
        self.initialized = False
        self.last_target = None

        # Rate limiting
        self.last_cmd_time = 0.0
        self.cmd_interval = 1.0 / self.get_parameter('cmd_rate_hz').value

        # Load workspace limits
        self.LIMITS = {
            'x': self.get_parameter('limits.x').value,
            'y': self.get_parameter('limits.y').value,
            'z': self.get_parameter('limits.z').value,
        }

        # =========================
        # 🔥 BUILD MAPPERS (ONCE)
        # =========================

        def build_mapper(expr):
            """
            Parses a YAML mapping string (e.g., '-z', '+pitch') and returns a 
            lambda function that applies the correct sign and axis translation.
            """
            sign = -1 if expr[0] == '-' else 1
            key = expr[1:]
            return lambda d: sign * d[key]

        # Position mappers
        self.map_pos = {
            'x': build_mapper(self.get_parameter('mapping.position.x').value),
            'y': build_mapper(self.get_parameter('mapping.position.y').value),
            'z': build_mapper(self.get_parameter('mapping.position.z').value),
        }

        # Rotation mappers
        self.map_rot = {
            'rx': build_mapper(self.get_parameter('mapping.rotation.rx').value),
            'ry': build_mapper(self.get_parameter('mapping.rotation.ry').value),
            'rz': build_mapper(self.get_parameter('mapping.rotation.rz').value),
        }

        # =========================
        # 📡 ROS INTERFACES
        # =========================

        qos_realtime = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(PoseStamped, 'phone2act/robot_feedback',
                                 self.cb_robot_feedback, qos_profile=qos_realtime)

        self.create_subscription(PoseStamped, 'bros2/ar_pose',
                                 self.cb_ar_pose, 10)

        self.create_subscription(Int32MultiArray, 'bros2/volume',
                                 self.cb_clutch, 10)

        self.pub_target = self.create_publisher(PoseStamped,
                                                'phone2act/target_pose',
                                                qos_profile=qos_realtime)

        self.pub_gripper = self.create_publisher(Float32,
                                                 'phone2act/gripper_cmd', 10)

        self.get_logger().info("✅ Phone2Act Planner (Generic) Started")

    # =========================
    # 📥 CALLBACKS
    # =========================

    def cb_robot_feedback(self, msg):
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]

        (rx, ry, rz) = euler_from_quaternion(q)

        self.robot_pose = {
            'x': msg.pose.position.x * 1000.0,
            'y': msg.pose.position.y * 1000.0,
            'z': msg.pose.position.z * 1000.0,
            'rx': rx, 'ry': ry, 'rz': rz
        }

    def cb_clutch(self, msg):
        data = list(msg.data)
        if len(data) < 2:
            return

        # Gripper
        grip_cmd = Float32()
        grip_cmd.data = 1.0 if data[0] == 0 else 0.0
        self.pub_gripper.publish(grip_cmd)

        # Clutch logic
        if data[1] == 1:
            self.clutch_engaged = True
        else:
            if self.clutch_engaged:
                self.initialized = False
            self.clutch_engaged = False

    def cb_ar_pose(self, msg):
        """
        Main teleoperation loop. 
        Receives the raw 6-DoF pose from the phone, calculates the spatial delta 
        from the initial engagement point, applies axis mapping/scaling, and 
        publishes the final target pose to the robot bridge.
        """
        current_time = time.time()

        # Rate limiting
        if (current_time - self.last_cmd_time) < self.cmd_interval:
            return

        if self.clutch_engaged or self.robot_pose is None:
            return

        # Initialization
        if not self.initialized:
            self.initial_robot_pose = self.robot_pose.copy()
            self.initial_phone_pose = msg.pose

            q = [msg.pose.orientation.x, msg.pose.orientation.y,
                 msg.pose.orientation.z, msg.pose.orientation.w]

            self.initial_phone_euler = euler_from_quaternion(q)
            self.initialized = True
            return

        # =========================
        # 📐 DELTA COMPUTATION
        # =========================

        scale_p = self.get_parameter('scale_pos').value
        scale_r = self.get_parameter('scale_rot').value

        p_curr = msg.pose
        p_init = self.initial_phone_pose

        dx = (p_curr.position.x - p_init.position.x) * scale_p
        dy = (p_curr.position.y - p_init.position.y) * scale_p
        dz = (p_curr.position.z - p_init.position.z) * scale_p

        delta_pos = {'x': dx, 'y': dy, 'z': dz}

        # =========================
        # 🎯 POSITION MAPPING
        # =========================

        target_x = self.initial_robot_pose['x'] + self.map_pos['x'](delta_pos)
        target_y = self.initial_robot_pose['y'] + self.map_pos['y'](delta_pos)
        target_z = self.initial_robot_pose['z'] + self.map_pos['z'](delta_pos)

        # Clamp workspace
        target_x = max(self.LIMITS['x'][0], min(target_x, self.LIMITS['x'][1]))
        target_y = max(self.LIMITS['y'][0], min(target_y, self.LIMITS['y'][1]))
        target_z = max(self.LIMITS['z'][0], min(target_z, self.LIMITS['z'][1]))

        # =========================
        # 🔄 ROTATION MAPPING
        # =========================

        q_curr = [p_curr.orientation.x, p_curr.orientation.y,
                  p_curr.orientation.z, p_curr.orientation.w]

        e_curr = euler_from_quaternion(q_curr)

        delta_rot = {
            'roll':  e_curr[0] - self.initial_phone_euler[0],
            'pitch': e_curr[1] - self.initial_phone_euler[1],
            'yaw':   e_curr[2] - self.initial_phone_euler[2],
        }

        rx = self.initial_robot_pose['rx'] + self.map_rot['rx'](delta_rot) * scale_r
        ry = self.initial_robot_pose['ry'] + self.map_rot['ry'](delta_rot) * scale_r
        rz = self.initial_robot_pose['rz'] + self.map_rot['rz'](delta_rot) * scale_r

        # =========================
        # 🛡️ SAFETY FILTER
        # =========================

        if self.last_target:
            dist = math.sqrt(
                (target_x - self.last_target[0])**2 +
                (target_y - self.last_target[1])**2 +
                (target_z - self.last_target[2])**2
            )
            if dist > self.get_parameter('safe_limit_jump').value:
                return

        # =========================
        # 📤 PUBLISH
        # =========================

        out_msg = PoseStamped()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = "robot_base"

        out_msg.pose.position.x = target_x / 1000.0
        out_msg.pose.position.y = target_y / 1000.0
        out_msg.pose.position.z = target_z / 1000.0

        q_final = quaternion_from_euler(rx, ry, rz)

        out_msg.pose.orientation.x = q_final[0]
        out_msg.pose.orientation.y = q_final[1]
        out_msg.pose.orientation.z = q_final[2]
        out_msg.pose.orientation.w = q_final[3]

        self.pub_target.publish(out_msg)

        self.last_target = (target_x, target_y, target_z)
        self.last_cmd_time = current_time


def main():
    rclpy.init()
    node = Phone2ActPlanner()
    rclpy.spin(node)
    rclpy.shutdown()