#!/bin/bash
# Build the ROS 2 workspace

echo "Building ROS 2 workspace..."
rm -rf src/build src/install
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
echo "Build complete."
