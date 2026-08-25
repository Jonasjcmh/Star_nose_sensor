#!/usr/bin/env bash
# Interactive Humble dev shell with the workspace mounted (install/build
# from an earlier `build.sh` run persist across containers).
set -euo pipefail
cd "$(dirname "$0")/.."

docker run --rm -it \
    -v "$(pwd):/ws" \
    -w /ws \
    star_nose_ros2:humble \
    bash -c "source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; exec bash"
