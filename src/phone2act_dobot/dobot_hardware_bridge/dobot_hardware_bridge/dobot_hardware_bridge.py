#!/usr/bin/env python3
import math
import socket
import threading
import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from dobot_msgs_v4.msg import ToolVectorActual
from dobot_msgs_v4.srv import EnableRobot, ClearError

class DobotHardwareBridge(Node):
    """
    Phone2Act Hardware Bridge for Dobot CR5.
    Translates hardware-agnostic target poses from the core teleop node into 
    robot-specific TCP/HTTP commands, and publishes standardized robot feedback.
    """
    def __init__(self):
        super().__init__('dobot_hardware_bridge')

        # ==============================================================================
        # CONFIGURATION
        # ==============================================================================
        self.ROBOT_IP = "192.168.1.6"
        self.CMD_PORT = 30003  # Dobot Real-Time Control Port
        self.sock = None

        # QoS PROFILE: "Freshness Only"
        # Forces queue_size=1 behavior. If the robot/network is busy, it drops 
        # old messages and only evaluates the most recent pose, preventing latency buildup.
        qos_realtime = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ==============================================================================
        # ROS 2 INTERFACES
        # ==============================================================================
        
        # 1. Listeners (From Phone2Act Core)
        self.create_subscription(
            PoseStamped, '/phone2act/target_pose', 
            self.cb_target, qos_profile=qos_realtime 
        )
        self.create_subscription(
            Float32, '/phone2act/gripper_cmd', 
            self.cb_gripper, 10
        )

        # 2. Hardware Feedback Listener (From Dobot Driver)
        self.create_subscription(
            ToolVectorActual, '/dobot_msgs_v4/msg/ToolVectorActual', 
            self.cb_dobot_feedback, 10
        )

        # 3. Standardized Publisher (To Phone2Act Core / Universal Recorder)
        self.pub_feedback = self.create_publisher(
            PoseStamped, '/phone2act/robot_feedback', 
            qos_profile=qos_realtime
        )

        # 4. Service Clients (For Initialization)
        self.clear_error_client = self.create_client(ClearError, '/dobot_bringup_ros2/srv/ClearError')
        self.enable_robot_client = self.create_client(EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot')

        # ==============================================================================
        # INITIALIZATION
        # ==============================================================================
        self.get_logger().info(f"Connecting to Dobot at {self.ROBOT_IP}:{self.CMD_PORT}...")
        self.connect_socket()
        self.wait_for_services()
        self.async_clear_error()

    # ==========================================================================
    # LOW-LEVEL NETWORKING
    # ==========================================================================
    def connect_socket(self):
        """Establishes a low-latency TCP connection to the Dobot controller."""
        if self.sock:
            try: self.sock.close()
            except: pass
            
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # TCP_NODELAY disables Nagle's algorithm, crucial for <350ms teleop latency
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1) 
            self.sock.settimeout(0.01) 
            self.sock.connect((self.ROBOT_IP, self.CMD_PORT))
            self.get_logger().info("Connected to Dobot TCP Port.")
        except Exception as e:
            self.get_logger().error(f"TCP Connection Failed: {e}")
            self.sock = None

    def send_tcp_command(self, cmd_str):
        """Sends a formatted command string to the Dobot over TCP."""
        if not self.sock:
            self.connect_socket()
            if not self.sock: return
            
        try:
            # Flush the read buffer to prevent stale data buildup
            try: self.sock.recv(1024) 
            except: pass 
            
            full_cmd = cmd_str + "\r\n"
            self.sock.sendall(full_cmd.encode('utf-8'))
        except BrokenPipeError:
            self.get_logger().warn("TCP Broken Pipe. Attempting reconnect...")
            self.sock = None
        except Exception:
            pass

    # ==========================================================================
    # ROBOT INITIALIZATION SERVICES
    # ==========================================================================
    def wait_for_services(self):
        while not self.clear_error_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ClearError service...')
        while not self.enable_robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for EnableRobot service...')

    def async_clear_error(self):
        self.clear_error_client.call_async(ClearError.Request()).add_done_callback(self.clear_cb)
        
    def clear_cb(self, f):
        if f.result().res == 0: 
            self.async_enable_robot()
        else: 
            self.create_timer(1.0, self.async_clear_error)

    def async_enable_robot(self):
        self.enable_robot_client.call_async(EnableRobot.Request()).add_done_callback(self.enable_cb)
        
    def enable_cb(self, f):
        if f.result().res == 0: 
            self.get_logger().info("Robot ENABLED and Ready for Teleop.")
        else: 
            self.create_timer(1.0, self.async_enable_robot)

    # ==========================================================================
    # CALLBACKS
    # ==========================================================================
    def cb_dobot_feedback(self, msg):
        """
        Converts hardware-specific Dobot feedback (mm, degrees) into the 
        standard Phone2Act ROS pose (meters, quaternions).
        """
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "robot_base"
        
        # 1. Position: mm -> meters (ROS Standard)
        out.pose.position.x = msg.x / 1000.0
        out.pose.position.y = msg.y / 1000.0
        out.pose.position.z = msg.z / 1000.0
        
        # 2. Orientation: Degrees -> Radians -> Quaternion
        rx_rad = math.radians(msg.rx)
        ry_rad = math.radians(msg.ry)
        rz_rad = math.radians(msg.rz)

        q = quaternion_from_euler(rx_rad, ry_rad, rz_rad)
        out.pose.orientation.x = q[0]
        out.pose.orientation.y = q[1]
        out.pose.orientation.z = q[2]
        out.pose.orientation.w = q[3]
        
        self.pub_feedback.publish(out)

    def cb_target(self, msg):
        """
        Converts the standard Phone2Act target pose (meters, quaternions) 
        into the hardware-specific Dobot ServoP command (mm, degrees).
        """
        # 1. Position: meters -> mm
        x = msg.pose.position.x * 1000.0
        y = msg.pose.position.y * 1000.0
        z = msg.pose.position.z * 1000.0
        
        # 2. Orientation: Quaternion -> Euler Radians
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (rx_rad, ry_rad, rz_rad) = euler_from_quaternion(q)
        
        # 3. Orientation: Radians -> Degrees (Required by ServoP)
        rx = math.degrees(rx_rad)
        ry = math.degrees(ry_rad)
        rz = math.degrees(rz_rad)
        
        # 4. Format and transmit TCP command
        cmd_str = f"ServoP({x:.3f},{y:.3f},{z:.3f},{rx:.3f},{ry:.3f},{rz:.3f})"
        self.send_tcp_command(cmd_str)

    def cb_gripper(self, msg):
        """
        Handles gripper commands in a separate daemon thread to ensure HTTP 
        requests do not block the high-frequency spatial tracking thread.
        """
        action = "close" if msg.data > 0.5 else "open"
        threading.Thread(target=self._send_gripper_http, args=(action,), daemon=True).start()

    def _send_gripper_http(self, action):
        """Sends the specific MagicBox HTTP payloads for the Dobot gripper."""
        try:
            url = f"http://{self.ROBOT_IP}:22000/interface/toolDataExchange"
            headers = {"Content-Type": "application/json"}
            
            if action == "close": 
                payload = {"value": [1, 6, 1, 3, 0, 0, 120, 54]}
            else: 
                payload = {"value": [1, 6, 1, 3, 3, 232, 120, 136]}
                
            requests.post(url, json=payload, headers=headers, timeout=2.0)
        except Exception: 
            pass

def main():
    rclpy.init()
    node = DobotHardwareBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()