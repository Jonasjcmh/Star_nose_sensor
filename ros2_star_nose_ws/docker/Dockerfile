# Humble parity image for building/testing star_nose_ros2 before it goes
# anywhere near the UR5's own Humble machine.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Python deps the nodes import: PyYAML (points.yaml), pyserial (MUCA),
# ur_rtde (rtde_control/rtde_receive -- the UR5 driver library).
RUN pip3 install --no-cache-dir pyyaml pyserial ur_rtde

WORKDIR /ws
