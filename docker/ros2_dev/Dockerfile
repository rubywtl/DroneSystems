# Use the base ROS 2 image
FROM osrf/ros:humble-desktop

# Set the working directory to /workspace
WORKDIR /workspace

# Set timezone to Los Angeles
RUN ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata

# Install necessary system dependencies for building ROS 2 packages and Python ML libraries
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    curl \
    libatlas-base-dev libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install pigpio dependencies and build it from source
RUN apt-get update && apt-get install -y \
    git \
    make \
    gcc \
    g++ \
    && git clone https://github.com/joan2937/pigpio.git /tmp/pigpio \
    && cd /tmp/pigpio \
    && make \
    && sudo make install \
    && rm -rf /tmp/pigpio

# Install a specific version of setuptools and other Python dependencies
RUN pip3 install setuptools==58.2.0 pip && \
    pip3 install --ignore-installed sympy opencv-python-headless tensorflow torch && \
    pip3 install "numpy<2"

# Install ROS 2 dependencies (specific to your package)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update

# Copy the local ROS workspace into the container
COPY . /workspace

# Set up the ROS 2 workspace and build it
RUN colcon build --symlink-install

# Set up environment variables for ROS 2 workspace
ENV AMENT_PREFIX_PATH=/workspace/install/control_node:/workspace/install/camera_node:/opt/ros/humble
ENV CMAKE_PREFIX_PATH=/workspace/install/control_node:/opt/ros/humble

# Source the ROS 2 setup script to set up the ROS environment
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]
RUN source ~/.bashrc

# Expose ROS 2 communication ports (for testing with external tools like RViz, if needed)
EXPOSE 11311

# Set the default command to run when the container starts
CMD ["bash", "-c", "source /workspace/install/setup.bash && ros2 launch launch/full_system.launch.py"]
