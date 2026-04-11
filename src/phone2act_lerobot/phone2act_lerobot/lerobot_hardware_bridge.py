#!/usr/bin/env python3
import os
import threading
from pathlib import Path

import rclpy
import draccus
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from tf_transformations import quaternion_matrix, quaternion_from_matrix

import ikpy.chain
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

# ==============================================================================
# USER CONFIGURATION
# ==============================================================================
ROBOT_NAME   = "so100_follower"
CONTROL_FREQ = 45.0

# Dynamically resolve paths so this works on any machine that clones the repo
PKG_SHARE_DIR    = get_package_share_directory('phone2act_lerobot')
URDF_PATH        = os.path.join(PKG_SHARE_DIR, 'urdf', 'so101_new_calib.urdf')
CALIBRATION_PATH = Path(os.path.join(PKG_SHARE_DIR, 'config', 'my_awesome_follower_arm.json'))
GRIPPER_OPEN  =  80.0
GRIPPER_CLOSE = -80.0

MOTOR_SETUP = {
    "shoulder_pan":  Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex":    Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex":    Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll":    Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper":       Motor(6, "sts3215", MotorNormMode.RANGE_M100_100),
}

JOINT_LIMITS_RAD = {
    "shoulder_pan":  (-1.91986,  1.91986),
    "shoulder_lift": (-1.74533,  1.74533),
    "elbow_flex":    (-1.69000,  1.69000),
    "wrist_flex":    (-1.65806,  1.65806),
    "wrist_roll":    (-2.74385,  2.84121),
}

JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

def motor_to_rad(motor_val: float, joint_name: str) -> float:
    lo, hi = JOINT_LIMITS_RAD[joint_name]
    t = (motor_val + 100.0) / 200.0
    return lo + t * (hi - lo)

def rad_to_motor(rad_val: float, joint_name: str) -> float:
    lo, hi = JOINT_LIMITS_RAD[joint_name]
    t = (rad_val - lo) / (hi - lo)
    return max(-100.0, min(100.0, t * 200.0 - 100.0))

class LeRobotHardwareBridge(Node):
    def __init__(self):
        super().__init__(f'{ROBOT_NAME}_bridge')

        self.declare_parameter('usb_port', '/dev/ttyACM0')
        self.usb_port = self.get_parameter('usb_port').value
        self.get_logger().info(f"Using USB port: {self.usb_port}")

        self.qos_realtime = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.hw_group  = MutuallyExclusiveCallbackGroup()
        self.ik_group  = MutuallyExclusiveCallbackGroup()
        self.cmd_group = ReentrantCallbackGroup()

        try:
            self.chain = ikpy.chain.Chain.from_urdf_file(URDF_PATH)
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF: {e}")
            raise

        self.create_subscription(
            PoseStamped, 'phone2act/target_pose',
            self.cb_target, qos_profile=self.qos_realtime, callback_group=self.ik_group
        )
        self.create_subscription(
            Float32, 'phone2act/gripper_cmd',
            self.cb_gripper, 10, callback_group=self.cmd_group
        )
        
        # ==============================================================================
        # PHONE2ACT ARCHITECTURE NOTE:
        # This publisher broadcasts the actual, real-time TCP pose of the robot.
        # It is critical for the modularity described in the Phone2Act paper:
        # 1. The Phone2Act Universal Recorder listens to this topic to log synchronized
        #    robot state data for GR00T policy training.
        # 2. The Phone2Act Teleop node listens to this to maintain the correct
        #    spatial offsets and rotation matching relative to the phone's initial pose.
        # ==============================================================================
        self.pub_feedback = self.create_publisher(
            PoseStamped, 'phone2act/robot_feedback', qos_profile=self.qos_realtime
        )
        
        self.create_timer(1.0 / CONTROL_FREQ, self.loop_feedback, callback_group=self.hw_group)

        self._lock = threading.Lock()
        self.bus = None
        self.current_joints_rad = [0.0] * len(self.chain.links)
        self.target_joints = {k: 0.0 for k in MOTOR_SETUP.keys()}
        self.gripper_target = 0.0
        self.torque_enabled = False
        self._hw_initialized = False

        self.connect_hardware()

    def connect_hardware(self):
        try:
            with open(CALIBRATION_PATH) as f, draccus.config_type("json"):
                calibration = draccus.load(dict[str, MotorCalibration], f)

            self.bus = FeetechMotorsBus(port=self.usb_port, motors=MOTOR_SETUP, calibration=calibration)
            self.bus.connect()

            for name in MOTOR_SETUP:
                try: self.bus.write("Lock", name, 0)
                except Exception: pass
                self.bus.write("Torque_Enable", name, 1)

            self.torque_enabled = True
            self.get_logger().info("LeRobot SO-100 connected. Torque ON.")

            for name in MOTOR_SETUP:
                self.target_joints[name] = float(self.bus.read("Present_Position", name))
            self.gripper_target = self.target_joints["gripper"]

            raw_seed = [0.0] * len(self.chain.links)
            for chain_idx, joint_name in enumerate(JOINT_ORDER, start=1):
                raw_seed[chain_idx] = motor_to_rad(self.target_joints[joint_name], joint_name)

            with self._lock:
                self.current_joints_rad = self._clamp_seed_to_bounds(raw_seed)
                self._hw_initialized = True

        except Exception as e:
            self.get_logger().error(f"Hardware connection failed: {e}")

    def cb_target(self, msg):
        with self._lock:
            if self.bus is None or not self._hw_initialized: return
            seed = self._clamp_seed_to_bounds(self.current_joints_rad)

        target_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        target_rot_3x3 = quaternion_matrix(q)[:3, :3]

        try:
            ik_joints_rad = self.chain.inverse_kinematics(
                target_position=target_pos, target_orientation=target_rot_3x3,
                orientation_mode="all", initial_position=seed,
            )
        except Exception:
            return

        new_targets = {
            joint_name: rad_to_motor(ik_joints_rad[chain_idx], joint_name)
            for chain_idx, joint_name in enumerate(JOINT_ORDER, start=1)
        }

        with self._lock:
            new_targets["gripper"] = self.gripper_target
            self.target_joints.update(new_targets)

    def cb_gripper(self, msg):
        target = GRIPPER_CLOSE if msg.data == 1.0 else GRIPPER_OPEN
        with self._lock:
            self.gripper_target = target
            self.target_joints["gripper"] = target

    def loop_feedback(self):
        """
        Reads actual motor positions, computes Forward Kinematics (FK), 
        and publishes the real TCP pose to /phone2act/robot_feedback.
        """
        with self._lock:
            if self.bus is None or not self.torque_enabled: return
            targets = self.target_joints.copy()

        try:
            for name, val in targets.items():
                self.bus.write("Goal_Position", name, val)
            current = {name: float(self.bus.read("Present_Position", name)) for name in MOTOR_SETUP}
        except Exception:
            return

        joints_rad = [0.0] * len(self.chain.links)
        for chain_idx, joint_name in enumerate(JOINT_ORDER, start=1):
            joints_rad[chain_idx] = motor_to_rad(current[joint_name], joint_name)

        with self._lock:
            self.current_joints_rad = joints_rad

        # Compute Forward Kinematics for actual pose feedback
        fk_matrix = self.chain.forward_kinematics(joints_rad)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "robot_base"
        out.pose.position.x = fk_matrix[0, 3]
        out.pose.position.y = fk_matrix[1, 3]
        out.pose.position.z = fk_matrix[2, 3]

        q = quaternion_from_matrix(fk_matrix)
        out.pose.orientation.x = q[0]
        out.pose.orientation.y = q[1]
        out.pose.orientation.z = q[2]
        out.pose.orientation.w = q[3]

        self.pub_feedback.publish(out)

    def _clamp_seed_to_bounds(self, joints_rad):
        seed = list(joints_rad)
        for i, link in enumerate(self.chain.links):
            if link.bounds is not None:
                lo, hi = link.bounds
                if lo is not None and hi is not None:
                    seed[i] = max(float(lo), min(float(hi), seed[i]))
        return seed

def main():
    rclpy.init()
    node = LeRobotHardwareBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.bus is not None:
            for name in MOTOR_SETUP:
                try: node.bus.write("Torque_Enable", name, 0)
                except Exception: pass
            node.bus.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()