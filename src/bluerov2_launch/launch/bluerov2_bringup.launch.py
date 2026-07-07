#VERY MUCH ON HOLD - DO NOT, DO NOT TRY TO RUN! Package names are outdated, nodes outdated as well.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='bluerov2_arm_disarm', executable='arm_disarm'),
        Node(package='bluerov2_behavior', executable='lights'),
        Node(package='bluerov2_behavior', executable='movement'),
        Node(package='bluerov2_behavior', executable='manual_next'),
        Node(package='bluerov2_controllers', executable='pid_depth'),
        Node(package='bluerov2_controllers', executable='pid_heading'),
        Node(package='bluerov2_controllers', executable='pid_controller'),
        Node(package='bluerov2_controllers', executable='depth_publisher'),
        Node(package='bluerov2_detection', executable='april_tag'),
        Node(package='bluerov2_detection', executable='depth'),
        Node(package='bluerov2_path_planning', executable='path_planning'),
    ])
