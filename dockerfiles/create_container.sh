#!/bin/bash

# Set variables for paths and options
WORKSPACE_DIR="/home/user/ws"
CONTAINER_NAME="mavros_ros2"
IMAGE_NAME="mavros_ros2"

# Detect display protocol (X11 or Wayland)
if [ -n "$WAYLAND_DISPLAY" ]; then
  echo "Wayland detected."
  DISPLAY_ENV="-e XDG_RUNTIME_DIR=/tmp -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY"
  VOLUME_MAPPING="-v $XDG_RUNTIME_DIR/$WAYLAND_DISPLAY:/tmp/$WAYLAND_DISPLAY"
else
  echo "X11 detected."
  DISPLAY_ENV="-e DISPLAY=$DISPLAY"
  VOLUME_MAPPING="-v /tmp/.X11-unix:/tmp/.X11-unix"
fi

# Run Docker container with appropriate settings
docker run --gpus all \
  --network host \
  --privileged \
  --runtime=nvidia \
  -v "$WORKSPACE_DIR:/home/ws" \
  $DISPLAY_ENV \
  $VOLUME_MAPPING \
  --entrypoint /bin/bash \
  --name "$CONTAINER_NAME" \
  -it "$IMAGE_NAME"
