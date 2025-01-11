import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the camera_node
        Node(
            package='camera_node',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        # Launch the object_detection_node
        Node(
            package='object_detection_node',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            remappings=[('/camera_topic', '/camera_node/output_topic')]  # Remap topics if needed
        )
    ])
