docker run -it --rm --net=host --privileged \
    -v $(pwd)/src:/ros_ws/src \
    ros2_dev_image
