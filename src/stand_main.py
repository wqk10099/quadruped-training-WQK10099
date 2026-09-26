import threading
import time
from pathlib import Path

import mujoco.viewer
import numpy as np
from .simulator import MuJoCoSimulator
from .stand_controller import PDController

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = PROJECT_ROOT / "scenes" / "flat_scene.xml"
VIEWER_DT = 0.02
sim = MuJoCoSimulator(SCENE_PATH)
q_des = np.array([
    0.0,  0.6, -1.0,
    0.0, -0.6,  1.0,
    0.0, -0.6,  1.0,
    0.0,  0.6, -1.0,
])
controller = PDController(
    kp=40.0,
    kd=2.0,
    q_des=q_des,
)
lock = threading.Lock()
sim.reset_to_keyframe("stand")

def physics_loop():
    while viewer.is_running():
        step_start = time.time()

        with lock:
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

if __name__=="__main__":
    
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        physics_thread = threading.Thread(target=physics_loop)
        viewer_thread = threading.Thread(target=viewer_loop)

        physics_thread.start()
        viewer_thread.start()

        physics_thread.join()
        viewer_thread.join()
