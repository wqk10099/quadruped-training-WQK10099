import threading
import time
from pathlib import Path

import mujoco.viewer
import numpy as np
from .simulator import MuJoCoSimulator
from .controller import RobotController
from .controller import RobotMode
PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = PROJECT_ROOT / "scenes" / "flat_scene.xml"
VIEWER_DT = 0.02

Q_LIE = np.array([0,  1.55, -2.45,
                      0, -1.55,  2.45,
                      0, -1.55,  2.45,
                      0,  1.55, -2.45])

Q_STAND = np.array([0,  0.6, -1.0,
                        0, -0.6,  1.0,
                        0, -0.6,  1.0,
                        0,  0.6, -1.0])
sim = MuJoCoSimulator(SCENE_PATH)
controller = RobotController(Q_STAND,Q_LIE)
lock = threading.Lock()
pending={"mode":None}   #跨线程传一个请求
KEY_MAP = {
    ord("S"): RobotMode.STAND,
    ord("L"): RobotMode.LIE,
    ord("D"): RobotMode.DAMPING,
}


def physics_loop():
    while viewer.is_running():
        step_start = time.time()

        with lock:
            if pending["mode"] is not None:
                controller.set_mode(pending["mode"])
                pending["mode"]=None
            controller.update(sim)
            sim.step()

        time_left = sim.timestep - (time.time() - step_start)
        if time_left > 0:
            time.sleep(time_left)
def viewer_loop():
    while viewer.is_running():
        with lock:
            viewer.sync()

        time.sleep(VIEWER_DT)
def on_key(key):
        mode=KEY_MAP.get(key)
        if mode is not None:
            pending["mode"]=mode

if __name__=="__main__":
    
    with mujoco.viewer.launch_passive(sim.model, sim.data,key_callback=on_key) as viewer:
        physics_thread = threading.Thread(target=physics_loop)
        viewer_thread = threading.Thread(target=viewer_loop)

        physics_thread.start()
        viewer_thread.start()

        physics_thread.join()
        viewer_thread.join()