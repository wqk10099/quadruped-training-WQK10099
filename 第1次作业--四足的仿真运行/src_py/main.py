from pathlib import Path
import time
import mujoco.viewer

from .simulator import MuJoCoSimulator
from .controller import ZeroTorqueController


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCENE_PATH = PROJECT_ROOT / "scenes" / "flat_scene.xml"


def main():
    sim = MuJoCoSimulator(SCENE_PATH)
    controller = ZeroTorqueController()

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            controller.update(sim)
            sim.step()
            viewer.sync()

            time_left = sim.timestep - (time.time() - step_start)
            if time_left > 0:
                time.sleep(time_left)


if __name__ == "__main__":
    main()