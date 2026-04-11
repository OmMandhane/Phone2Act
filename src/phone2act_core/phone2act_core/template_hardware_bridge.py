#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

# ==============================================================================
# 🛠️ USER CONFIGURATION SECTION
# ==============================================================================
ROBOT_NAME = "my_custom_robot"  # e.g., "ur5", "franka", "so_100"
CONTROL_FREQ = 50.0             # Hz (Should be >= Planner Frequency)
# ==============================================================================

class GenericRobotBridge(Node):
    def __init__(self):
        super().__init__(f'{ROBOT_NAME}_bridge')

        # --- 1. CRITICAL: LOW LATENCY QoS SETUP ---
        # This configuration guarantees the robot never processes old data.
        # It forces a "Mailbox Size" of 1. Old commands are deleted instantly.
        self.qos_realtime = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1  # <--- The Secret Sauce: Drop stale packets immediately
        )

        # --- 2. ROS INTERFACE (DO NOT CHANGE) ---
        # Subscribe to the standard Phone2Act Command Topic
        self.create_subscription(
            PoseStamped, 
            '/phone2act/target_pose', 
            self.cb_target, 
            qos_profile=self.qos_realtime
        )
        
        # Publish Feedback to the Planner
        self.pub_feedback = self.create_publisher(
            PoseStamped, 
            '/phone2act/robot_feedback', 
            qos_profile=self.qos_realtime
        )

        # Feedback Loop Timer
        self.create_timer(1.0 / CONTROL_FREQ, self.loop_feedback)

        # --- 3. HARDWARE CONNECTION ---
        self.get_logger().info(f"🔌 Connecting to {ROBOT_NAME}...")
        self.user_connect_hardware()
        self.get_logger().info("✅ Bridge Ready.")

    # ==========================================================================
    # 🏗️ SECTION A: CONNECT TO YOUR ROBOT
    # ==========================================================================
    def user_connect_hardware(self):
        """
        TODO: Put your robot's connection code here.
        Example: serial.Serial('/dev/ttyUSB0') or socket.connect('192.168.1.5')
        """
        # self.robot = MyRobotDriver()
        # self.robot.connect()
        pass

    # ==========================================================================
    # 🚀 SECTION B: TRANSLATE COMMANDS (ROS -> ROBOT)
    # ==========================================================================
    def cb_target(self, msg):
        """
        Callback for new commands.
        Input: msg (PoseStamped) in METERS and QUATERNIONS.
        """
        # 1. Extract Position (Standard Units: Meters)
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # 2. Extract Rotation (Standard Units: Radians)
        q = [msg.pose.orientation.x, msg.pose.orientation.y, 
             msg.pose.orientation.z, msg.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)

        # 3. TODO: SEND TO YOUR ROBOT
        # Convert units if needed (e.g., meters -> mm, radians -> degrees)
        # Example: 
        #   cmd = f"MoveTo({x*1000}, {y*1000}, {z*1000})"
        #   self.robot.send(cmd)
        pass

    # ==========================================================================
    # 📡 SECTION C: TRANSLATE FEEDBACK (ROBOT -> ROS)
    # ==========================================================================
    def loop_feedback(self):
        """
        Reads robot state and publishes it for the Planner.
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "robot_base"

        # 1. TODO: GET STATE FROM ROBOT
        # Example: 
        #   pos = self.robot.get_current_pose()
        #   current_x, current_y, current_z = pos.x, pos.y, pos.z
        
        # 2. FILL MSG (Convert back to Meters/Radians if needed)
        # msg.pose.position.x = current_x
        # msg.pose.position.y = current_y
        # msg.pose.position.z = current_z
        
        # msg.pose.orientation.w = 1.0 # (Or fill actual quaternion)

        self.pub_feedback.publish(msg)

def main():
    rclpy.init()
    node = GenericRobotBridge()
    rclpy.spin(node)
    rclpy.shutdown()