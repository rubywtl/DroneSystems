from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    mode = 'deployment'  # Change to 'deployment' if needed

    return LaunchDescription([
        Node(
            package='camera_node',
            executable='camera_node',
            output='screen',
            parameters=[{'mode': mode}]
        ),
        Node(
            package='object_detection_node',
            executable='object_detection_node',
            output='screen',
            parameters=[{'mode': mode}]
        ),
        Node(
            package='grabber_node',
            executable='grabber_node',
            output='screen',
            parameters=[{'mode': mode}]
        ),
    ])
