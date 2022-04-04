#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")"; cd ..; pwd)"

sudo docker run -it \
  --name ros-docker-simple \
  --volume ${PROJECT_ROOT}/catkin_ws/src:/HIL \
  # -w \
  # --rm \
  # ros-docker-simple "$@"

# Share a device with the Docker container by adding:
#   --device=/dev/ttyUSB0 \
# below the line containing "--name"