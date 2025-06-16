#!/bin/bash

# Check if an argument is provided
if [ -z "$1" ]; then
  echo "Usage: $0 [pc|rasp]"
  exit 1
fi

# Set Dockerfile based on argument
if [ "$1" == "pc" ]; then
  DOCKERFILE="mavros_ros2_pc.dockerfile"
elif [ "$1" == "rasp" ]; then
  DOCKERFILE="mavros_ros2_rasp.dockerfile"
else
  echo "Invalid argument: $1. Use 'pc' or 'rasp'."
  exit 1
fi

# Build the Docker image
docker build -f "$DOCKERFILE" -t mavros_ros2 .
