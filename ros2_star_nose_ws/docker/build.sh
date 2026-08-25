#!/usr/bin/env bash
# Build the Humble parity image, then colcon-build the workspace inside it.
set -euo pipefail
cd "$(dirname "$0")/.."

docker build -t star_nose_ros2:humble -f docker/Dockerfile .

docker run --rm \
    -v "$(pwd):/ws" \
    -w /ws \
    star_nose_ros2:humble \
    bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
