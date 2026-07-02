from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='path_planning_pkg', executable='path_planning_node'),
        Node(package='flash_lights_pkg', executable='flash_lights_node'),
        Node(package='rov_move_pkg', executable='rov_move_node'),
        Node(package='manual_next_pkg', executable='manual_next_node'),
        Node(package='depth_pid_pkg', executable='depth_pid_node'),
        Node(package='heading_pid_pkg', executable='heading_pid_node'),
        Node(package='arm_disarm_pkg', executable='arm_disarm_node'),
    ])
