import rtde_control, rtde_receive
import time

ROBOT_IP = "177.22.22.2"     # <-- change to your robot IP
PRINT_INTERVAL = 0.5         # seconds

# -------------------------
# Connect to the robot
# -------------------------
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

# -------------------------
# Get starting pose
# -------------------------
start_pose = rtde_r.getActualTCPPose()   # [x, y, z, rx, ry, rz]
force_sensor=rtde_r.getActualTCPForce()

# Set a target pose 10 cm forward in X
target_pose = start_pose.copy()
target_pose[2] += 0.01   # move 10 cm in X direction

print("Starting linear movement...")

# -------------------------
# Move in a straight line
# -------------------------
speed = 0.1      # m/s
accel = 0.2      # m/s^2
rtde_c.moveL(target_pose, speed, accel)

# -------------------------
# Print position every 0.5 s
# -------------------------
print("Printing TCP pose every 0.5s... (Ctrl+C to stop)")
try:
    while True:
        pose = rtde_r.getActualTCPPose()
        force=rtde_r.getActualTCPForce()
        print(f"TCP Pose: {force}")
        time.sleep(PRINT_INTERVAL)
except KeyboardInterrupt:
    print("Stopping...")

# -------------------------
# Disconnect
# -------------------------
rtde_c.stopScript()
print("Done.")
