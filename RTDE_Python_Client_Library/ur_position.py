import rtde_receive

rtde_r = rtde_receive.RTDEReceiveInterface("177.22.22.2")
pose = rtde_r.getActualTCPPose()

print("Current TCP pose:")
print(f"  X  = {pose[0]:.5f} m")
print(f"  Y  = {pose[1]:.5f} m")
print(f"  Z  = {pose[2]:.5f} m")
print(f"  Rx = {pose[3]:.5f} rad")
print(f"  Ry = {pose[4]:.5f} rad")
print(f"  Rz = {pose[5]:.5f} rad")

rtde_r.disconnect()
