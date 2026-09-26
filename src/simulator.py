from pathlib import Path
import mujoco


class MuJoCoSimulator:
    def __init__(self, scene_path:Path):
        self.model=mujoco.MjModel.from_xml_path(str(scene_path))
        self.data=mujoco.MjData(self.model)
    @property
    def timestep(self):
        return self.model.opt.timestep
    @property
    def nq(self):
        return self.model.nq

    @property
    def nv(self):
        return self.model.nv

    @property
    def nu(self):
        return self.model.nu

    def reset(self):
        mujoco.mj_resetData(self.model,self.data)

    def step(self):
        mujoco.mj_step(self.model,self.data)