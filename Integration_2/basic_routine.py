import rtde_control, rtde_receive
import time

ROBOT_IP = "177.22.22.2"  # <-- change to your robot IP

# -------------------------
# Parameters
# -------------------------
WAIT_TIME = 5.0  # seconds to wait at each position
MOVE_DISTANCE = 0.012  # 1.5 cm down
SPEED = 0.05  # m/s
ACCEL = 0.1  # m/s^2
CYCLES = 5  # number of repetitions
PRINT_INTERVAL = 0.5

# -------------------------
# Connect to robot
# -------------------------
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

# -------------------------
# Get initial pose
# -------------------------
home_pose = rtde_r.getActualTCPPose()  # [x, y, z, rx, ry, rz]

# -------------------------
# Prepare top and bottom poses
# -------------------------
top_pose = home_pose.copy()
bottom_pose = home_pose.copy()
bottom_pose[2] -= MOVE_DISTANCE  # move down

print("Starting cyclic Z movement...")

try:
    for cycle in range(CYCLES):
        print(f"\n--- Cycle {cycle + 1} ---")

        # Wait at top
        print("Waiting at top position...")
        t_start = time.time()
        while time.time() - t_start < WAIT_TIME:
            pose = rtde_r.getActualTCPPose()
            force = rtde_r.getActualTCPForce()
            print(f"TCP Pose: {pose}, TCP Force: {force}")
            time.sleep(PRINT_INTERVAL)

        # Move down
        print("Moving down...")
        rtde_c.moveL(bottom_pose, SPEED, ACCEL, True)
        # Wait until reached
        time.sleep(MOVE_DISTANCE / SPEED)

        # Wait at bottom
        print("Waiting at bottom position...")
        t_start = time.time()
        while time.time() - t_start < WAIT_TIME:
            pose = rtde_r.getActualTCPPose()
            force = rtde_r.getActualTCPForce()
            print(f"TCP Pose: {pose}, TCP Force: {force}")
            time.sleep(PRINT_INTERVAL)

        # Move back up
        print("Returning to top...")
        rtde_c.moveL(top_pose, SPEED, ACCEL, True)
        time.sleep(MOVE_DISTANCE / SPEED)

except KeyboardInterrupt:
    print("Stopping movement...")
    rtde_c.stopL()

print("Cyclic routine completed.")
