# Gripper (Robotiq 2F-85 on UR5e)

Control of the Robotiq 2F-85 through the Robotiq URCap socket (port 63352 on the robot IP).

| File | Purpose |
|---|---|
| `robotiq_gripper.py` | Driver (`RobotiqGripper`) + `SimulatedGripper` for testing without hardware |
| `gripper_gui.py` | Tkinter GUI: position slider, speed/force, live position, object-gripped indicator |

## Robot-side requirements
- Gripper connected to the Tool I/O, Robotiq URCap installed and activated (PolyScope 5.12)
- Robot on and reachable from the PC
- No PolyScope program driving the gripper at the same time

## Usage
```bash
python gripper_gui.py --sim                # test the GUI without hardware
python gripper_gui.py --ip 192.168.1.10    # real robot
```
No extra packages needed (Tkinter is part of standard Python; on some Linux
systems install `python3-tk`).

In the GUI: Connect -> Activate (gripper opens/closes fully) -> move the slider
(command is sent when you release it) or use Open / Close.
The status box shows **OBJECT GRIPPED** when the gripper stopped on contact.

Use from your own scripts:
```python
from robotiq_gripper import RobotiqGripper
g = RobotiqGripper("192.168.1.10"); g.connect(); g.activate()
st = g.move_and_wait(255, speed=100, force=80)
print("object gripped:", st["object_detected"])
```

## Notes
- Position 0 = fully open, 255 = fully closed. The mm value shown is a linear
  approximation of the 85 mm stroke.
- Object detection uses the gripper's OBJ register (1/2 = stopped by contact,
  3 = reached position without object).
- Not yet tested on the real gripper (connector pending). Start with low speed and force.
