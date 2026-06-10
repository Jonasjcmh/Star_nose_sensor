import numpy as np
import time
from ur_rtde import rtde_control

robot = rtde_control.RTDEControlInterface("177.22.22.2")

# --- Trajectory parameters ---
amplitude = 0.10       # 10 cm
wavelength = 0.10      # 10 cm per cycle
num_cycles = 3
samples_per_cycle = 200
speed = 0.05           # m/s feed speed (adjust as needed)

# --- Generate trajectory ---
total_points = samples_per_cycle * num_cycles
x = np.linspace(0, wavelength * num_cycles, total_points)
z = amplitude * np.sin(2 * np.pi * x / wavelength)

# Get current TCP pose [x, y, z, rx, ry, rz]
start_pose = np.array(robot.getActualTCPPose())
poses = []

for i in range(total_points):
    pose = start_pose.copy()
    pose[0] = start_pose[0] + x[i]   # move along X
    pose[2] = start_pose[2] + z[i]   # sinusoidal in Z
    poses.append(pose)

# --- Execute trajectory ---
print("Starting sinusoidal motion...")

for pose in poses:
    robot.servoL(pose, 1.0, 0.5, dt=0.008)  # servo control for smooth motion
    time.sleep(0.008)

# Stop motion and disconnect
robot.servoStop()
robot.disconnect()

print("Trajectory completed.")
