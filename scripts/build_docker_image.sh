#!/bin/bash
# Build the ROS 2 Docker image

IMAGE_NAME=$1
DOCKERFILE_PATH=$2

echo "Building Docker image $IMAGE_NAME..."
docker build -t $IMAGE_NAME -f $DOCKERFILE_PATH .
echo "Docker image built successfully."
