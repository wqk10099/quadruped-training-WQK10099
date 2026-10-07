import threading
import time
from pathlib import Path

import mujoco.viewer
import numpy as np
from .simulator import MuJoCoSimulator
from .controller import Damping_StandController

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
controller = Damping_StandController(Q_STAND,Q_LIE)
lock = threading.Lock()
pending={"stand":None}   #跨线程传一个请求



def physics_loop():
    while viewer.is_running():
        step_start = time.time()

        with lock:
            if pending["stand"] is not None:
                if pending["stand"]==True:
                    controller.set_stand()
                else:
                    controller.set_damping()
                pending["stand"]=None
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
        if key == ord("S"): #按S键站立
            pending["stand"] = True
        elif key == ord("D"): #按D键阻尼状态
            pending["stand"] = False

if __name__=="__main__":
    
    with mujoco.viewer.launch_passive(sim.model, sim.data,key_callback=on_key) as viewer:
        physics_thread = threading.Thread(target=physics_loop)
        viewer_thread = threading.Thread(target=viewer_loop)

        physics_thread.start()
        viewer_thread.start()

        physics_thread.join()
        viewer_thread.join()