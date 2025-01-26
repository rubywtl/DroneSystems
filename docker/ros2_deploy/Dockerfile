# Use the official ROS 2 Humble ROS Core image
FROM arm64v8/ros:humble-ros-core

# Set the working directory to /workspace
WORKDIR /workspace

# Install system dependencies (common tools for building ROS 2 packages)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    curl \ 
    liblgpio-dev \
    ros-humble-cv-bridge \
    libgl1-mesa-glx
    
RUN rm -rf /var/lib/apt/lists/* 

# Install Python dependencies (for ML tasks)
RUN pip3 install --upgrade pip && \
    pip3 install opencv-python tensorflow torch pigpio

# Install ROS 2 dependencies (specific to your package)
RUN rosdep init && \
    rosdep update

# Copy the local ROS workspace into the container
COPY . /workspace

# Set up the ROS 2 workspace and build it
RUN colcon build --symlink-install

# Source the ROS 2 setup script to set up the ROS environment
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]
RUN source ~/.bashrc

# Expose ROS 2 communication ports (for testing with external tools if necessary)
EXPOSE 11311

# Run the ROS 2 launch to start all nodes
# CMD ["ros2", "launch", "launch/full_system.launch.py"]
