cmake_minimum_required(VERSION 3.0.2)
project(drone_processing)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  image_transport
)

# Find OpenCV
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs cv_bridge image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# For building C++ node (camera driver)
add_executable(cameradriver src/imageprocessing/cameradriver.cpp)
target_link_libraries(cameradriver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# For building C++ node (motor control)
add_executable(control src/machinecontrol/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})
