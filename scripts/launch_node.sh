#!/bin/bash
# Launch ROS 2 nodes using a launch file

echo "Launching nodes..."
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch launch/launch.py
