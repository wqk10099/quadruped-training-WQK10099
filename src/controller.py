import numpy as np
class ZeroTorqueController:
    def update(self, simulator):
        simulator.data.ctrl[:]=0.0
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
class DampingController:
    def __init__(self, kd=2.0, clip=30.0):#参数有阻尼系数kd与上下限clip
        self.kd = kd
        self.clip = clip

    def update(self, simulator):
        dq = simulator.data.qvel[6:] # 12 个关节速度
        tau = -self.kd * dq
        simulator.data.ctrl[:] = np.clip(tau, -self.clip, self.clip)
class Damping_StandController:
    def __init__(self,q_stand_des,q_lie_des,kp=60.0,kd=3.0,clip=30.0,joint_speed=2.0):
        self.kp=kp
        self.kd=kd
        self.clip = clip
        self.joint_speed=joint_speed #规定旋转速度上限
        self.standing = False  #False=阻尼  True=站立
        self.q_stand = np.array(q_stand_des, dtype=float)
        self.q_lie   = np.array(q_lie_des,   dtype=float)
        self.q_cmd = self.q_stand.copy()        # 会移动的"指令角"，必须是副本
        self.waypoints = []       # 起立时经过的中间姿态
    def set_damping(self):
        self.standing = False
        self.waypoints = []

    def set_stand(self):
        self.standing = True
        self.waypoints = [self.q_lie.copy(), self.q_stand.copy()]   # 两段式起立

    def update(self, sim):
        d = sim.data
        # === 阻尼模式 ===
        if not self.standing:                       
            d.ctrl[:] = np.clip(-self.kd * d.qvel[6:], -self.clip, self.clip)
            self.q_cmd = d.qpos[7:].copy()          # ★关键：记住腿现在在哪
            return

        # === 起立模式 ===先回趴卧位，再起立
        target = self.waypoints[0] if self.waypoints else self.q_stand #if  else语句是兜底，如果所有路都走完了，就用q_stand,保持站立姿态

        # 指令角平滑逼近当前路点，而不是一步跳过去
        step = self.joint_speed * sim.timestep  #规定最大步长
        self.q_cmd += np.clip(target - self.q_cmd, -step, step)

        # 到了就换下一个路点
        if self.waypoints and np.max(np.abs(self.q_cmd - self.waypoints[0])) < 1e-2:
            self.waypoints.pop(0)

        q = d.qpos[7:]
        dq = d.qvel[6:]
        d.ctrl[:] = np.clip(self.kp * (self.q_cmd - q) - self.kd * dq,-self.clip, self.clip)
class RobotMode:
    DAMPING = "damping"
    STAND   = "stand"
    LIE     = "lie" #零力矩趴卧


class RobotController:
    # 12 个关节目标角，顺序 FL, FR, RR, RL（每条腿 hip, thigh, calf）
    Q_LIE = np.array([0,  1.55, -2.45,
                      0, -1.55,  2.45,
                      0, -1.55,  2.45,
                      0,  1.55, -2.45])

    Q_STAND = np.array([0,  0.6, -1.0,
                        0, -0.6,  1.0,
                        0, -0.6,  1.0,
                        0,  0.6, -1.0])

    def __init__(self, q_stand_des,q_lie_des, kp=60.0, kd=3.0, clip=30.0, joint_speed=2.0):
        self.kp = kp
        self.kd = kd
        self.clip = clip
        self.joint_speed = joint_speed

        self.mode = RobotMode.DAMPING       # 开机的默认模式
        self.q_stand = np.array(q_stand_des, dtype=float)
        self.q_lie   = np.array(q_lie_des,   dtype=float)
        self.q_cmd = self.q_stand.copy() 
        self.waypoints = []                 # 待走的路点清单

    # ---------------- 对外接口：按键调用 ----------------
    def set_mode(self, mode):
        if mode == self.mode:               # 已经在目标模式，什么都不做
            return
        self.mode = mode
        self.waypoints = []                 # 换模式时清空旧清单

        if mode == RobotMode.STAND:
            # 任意姿态 → 先收回趴卧位（归一化），再撑起站立
            self.waypoints = [self.q_lie.copy(), self.q_stand.copy()]
        elif mode == RobotMode.LIE:
            # 先收到趴卧位，到位后会松开力矩
            self.waypoints = [self.q_lie.copy()]

    # ---------------- 每个物理步调用一次 ----------------
    def update(self, simulator):
        d = simulator.data

        # ---- 情况 1：阻尼模式：只有速度阻尼，不跟踪角度 ----
        if self.mode == RobotMode.DAMPING:
            d.ctrl[:] = np.clip(-self.kd * d.qvel[6:], -self.clip, self.clip)
            self.q_cmd = d.qpos[7:].copy()      # 记住腿现在在哪
            return

        # ---- 情况 2：趴下已到位：松开力矩，自然塌成趴卧 ----
        if self.mode == RobotMode.LIE and not self.waypoints:
            d.ctrl[:] = 0.0
            self.q_cmd = d.qpos[7:].copy()
            return

        # ---- 情况 3：站立 / 趴下的移动过程：平滑跟踪路点 ----
        target = self.waypoints[0] if self.waypoints else self.q_stand

        step = self.joint_speed * simulator.timestep
        self.q_cmd += np.clip(target - self.q_cmd, -step, step)

        if self.waypoints and np.max(np.abs(self.q_cmd - self.waypoints[0])) < 1e-2:
            self.waypoints.pop(0)

        d.ctrl[:] = np.clip(self.kp * (self.q_cmd - d.qpos[7:]) - self.kd * d.qvel[6:],
                            -self.clip, self.clip)
