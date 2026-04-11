from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='phone2act_core',
            executable='phone2act_teleop_planner',
            name='phone2act_planner',
            parameters=['config/dobot.yaml']
        )
    ])