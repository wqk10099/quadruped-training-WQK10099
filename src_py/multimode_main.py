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
# 12 个关节目标角，顺序 FL, FR, RR, RL（每条腿 hip, thigh, calf）
Q_LIE = np.array([0,  1.55, -2.45,
                      0, -1.55,  2.45,
                      0, -1.55,  2.45,
                      0,  1.55, -2.45])
# 一条腿分为    hip（髋）：负责让整条腿左右内收或外摆
#             thingh(大腿)：负责让腿前后摆动，是前进的主要动力
#             calf(小腿)：负责膝盖的弯曲和伸展，抬腿依靠它
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
    ord("W"):RobotMode.WALK,
    ord("M"):RobotMode.MARCH
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