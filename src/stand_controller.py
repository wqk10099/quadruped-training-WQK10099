import numpy as np

class PDController:
    def __init__(self,kp,kd,q_des):
        self.kp=kp
        self.kd=kd
        self.q_des=q_des
    def update(self,simulator):
        q = simulator.data.qpos[7:]
        dq = simulator.data.qvel[6:]
        tau = self.kp * (self.q_des - q) - self.kd * dq
        simulator.data.ctrl[:] = np.clip(tau, -20.0, 20.0)
