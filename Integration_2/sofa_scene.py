"""
sofa_scene.py
SOFA visualization — 19 sensor points as colored spheres.
Reads from sensor.py shared memory — never opens serial port.
Run via: runSofa sofa_scene.py
"""
import sys
sys.path = [p for p in sys.path if 'python3.13' not in p and 'sofa-venv' not in p]
sys.path.insert(0, '/usr/local/lib/python3.10/dist-packages')
sys.path.insert(0, '/home/cao/PycharmProjects/PythonProject/Integration')

import Sofa
import Sofa.Core
import numpy as np
import sensor
import ur5_control

SCALE    = 0.012
BASE_Y   = 0.0
MAX_LIFT = 0.05
SPHERE_R = 0.007

POINTS_MM = [
    (-8,  +14), ( 0, +14), (+8, +14),
    (-12,  +7), (-4,  +7), (+4,  +7), (+12, +7),
    (-16,   0), (-8,   0), ( 0,   0), (+8,   0), (+16, 0),
    (-12,  -7), (-4,  -7), (+4,  -7), (+12, -7),
    (-8,  -14), ( 0, -14), (+8, -14),
]
RAW_CELLS = [2,15,28,1,14,27,40,0,13,26,39,52,12,25,38,51,24,37,50]
N = 19

RAMP = np.array([
    [0.165, 0.710, 0.627],  # teal   (0.0 rest)
    [0.200, 0.900, 0.400],  # green  (0.25)
    [1.000, 0.900, 0.100],  # yellow (0.5)
    [1.000, 0.450, 0.000],  # orange (0.75)
    [0.850, 0.000, 0.000],  # red    (1.0)
], dtype=float)

def get_color(v):
    v   = float(np.clip(v, 0.0, 1.0))
    idx = v * (len(RAMP) - 1)
    lo  = int(idx); hi = min(lo+1, len(RAMP)-1); t = idx - lo
    c   = RAMP[lo] + t * (RAMP[hi] - RAMP[lo])
    return [float(c[0]), float(c[1]), float(c[2]), 1.0]

CAMERA_VIEWS = {
    '1': dict(position=[0.0,  0.30, 0.001],
              lookAt=[0,0,0], distance=0.35, name='Top'),
    '2': dict(position=[0.15, 0.20, 0.28],
              lookAt=[0,0,0], distance=0.38, name='Isometric'),
    '3': dict(position=[0.0,  0.05, 0.42],
              lookAt=[0,0,0], distance=0.38, name='Side'),
}

class Controller(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.point_mos  = kwargs['point_mos']
        self.point_cols = kwargs['point_cols']
        self.camera     = kwargs['camera']
        self.base_pos   = [
            [x*SCALE, BASE_Y, y*SCALE]
            for (x, y) in POINTS_MM
        ]
        self.frame = 0

    def onAnimateBeginEvent(self, event):
        if not sensor.is_ready():
            return

        values    = np.array(sensor.get_values(), dtype=float)
        ur5_state = ur5_control.get_state()

        for i in range(N):
            v = float(values[i])

            # Move sphere up proportional to pressure
            pos    = list(self.base_pos[i])
            pos[1] = BASE_Y + v * MAX_LIFT
            self.point_mos[i].position.value = [pos]

            # Grow sphere with pressure
            r = SPHERE_R + v * SPHERE_R * 1.5
            self.point_cols[i].radius.value = r

        self.frame += 1
        if self.frame % 150 == 0:
            active = int(np.sum(values > 0.05))
            pt     = ur5_state.get('point', '?')
            press  = '▼' if ur5_state.get('pressing') else ' '
            print(f"[sofa] f={self.frame} | "
                  f"active={active}/19 | "
                  f"max={values.max():.3f} | "
                  f"UR5=P{pt}{press}")

    def onKeyPressedEvent(self, event):
        key = event.get('key', '')
        if key in CAMERA_VIEWS:
            v = CAMERA_VIEWS[key]
            self.camera.position.value = v['position']
            self.camera.lookAt.value   = v['lookAt']
            self.camera.distance.value = v['distance']
            print(f"[camera] → {v['name']}")

def createScene(rootNode):
    rootNode.gravity = [0, 0, 0]
    rootNode.dt      = 0.02

    rootNode.addObject('RequiredPlugin', pluginName=[
        'Sofa.Component.StateContainer',
        'Sofa.Component.AnimationLoop',
        'Sofa.Component.Visual',
        'Sofa.Component.Collision.Geometry',
        'Sofa.Component.Setting',
        'Sofa.GL.Component.Rendering3D',
    ])

    rootNode.addObject('DefaultAnimationLoop')
    rootNode.addObject('VisualStyle',
                       displayFlags='showCollisionModels')
    rootNode.addObject('BackgroundSetting',
                       color=[0.10, 0.10, 0.13, 1.0])

    camera = rootNode.addObject('InteractiveCamera',
                                name='cam',
                                position=[0.0, 0.30, 0.001],
                                lookAt=[0.0, 0.0, 0.0],
                                distance=0.35,
                                fieldOfView=45)

    point_mos  = []
    point_cols = []

    for i, (xmm, ymm) in enumerate(POINTS_MM):
        x = xmm * SCALE
        z = ymm * SCALE

        node = rootNode.addChild(f'P{i+1}_S{RAW_CELLS[i]}')
        mo   = node.addObject('MechanicalObject',
                              name='mo',
                              template='Vec3d',
                              position=[[x, BASE_Y, z]])
        col  = node.addObject('SphereCollisionModel',
                              radius=SPHERE_R,
                              color=get_color(0.0))
        point_mos.append(mo)
        point_cols.append(col)

    rootNode.addObject(Controller(
        name='Ctrl',
        point_mos=point_mos,
        point_cols=point_cols,
        camera=camera,
    ))

    # Never start sensor here — main.py owns the serial port
    if sensor.is_ready():
        print("[scene] Sensor already live!")
    else:
        print("[scene] Waiting for sensor data from main.py...")

    print(f"[scene] {N} points ready")
    print("[keys]  1=Top  2=Iso  3=Side  Space=Play")
    return rootNode