# Drone Processing & Control

This ROS package provides image processing and motor control functionality for a drone. The package is designed to process camera feed data and control the drone's movement based on the processed information.

## Features

- **Image Processing**: Captures camera feed and performs image processing tasks.
- **Motor Control**: Controls the drone's motor based on input data or image processing results.
- **ROS Integration**: Uses ROS nodes for communication between different modules.
- **Visualization**: Displays the camera feed in real-time using `rqt_image_view`.

## Requirements

- ROS (Robot Operating System)
- Catkin workspace setup
- OpenCV (for image processing)
- Sensor drivers for the camera

## Setup

### 1. Create the Workspace

First, create a ROS workspace if you don't have one already:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash

```
## Launch the Nodes

### 1. Launch Image Processing Node
Run the image processing node which processes the camera feed:

```bash
roslaunch drone_processing imageprocessing.launch
```

### 2. Launch Motor Control Node
Run the motor control node to control the drone's motors:

```bash
roslaunch drone_processing control.launch
```

## Visualize the Camera Feed
You can visualize the camera feed from /camera/image_raw using rqt_image_view:

```bash
rosrun rqt_image_view rqt_image_view
```

This will open a GUI window displaying the live camera feed from the drone.

### Troubleshooting
- Camera Feed Not Displaying: Ensure that the camera is properly connected and streaming. You can check if the /camera/image_raw topic is being published by running:

```bash
rostopic list
```

- Missing Dependencies: If you encounter issues with missing dependencies, run:

```bash
rosdep install --from-paths src --ignore-src -r -y
```