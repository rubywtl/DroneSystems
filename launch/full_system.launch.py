from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='camera_node', executable='camera_node', output='screen'),
        Node(package='object_detection_node', executable='object_detection_node', output='screen'),
        Node(package='control_node', executable='control_node', output='screen'),
    ])
