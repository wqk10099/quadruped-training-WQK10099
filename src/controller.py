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
    MARCH   = "march" #原地踏步
    WALK    = "walk"  #行走


class RobotController:
        # 每条腿的“几何系数”（由正运动学测得）：关节转 1 弧度，脚会移动多少米"的比例系数
    #   dx = KX * d(thigh)   foot x 位移 / 大腿关节
    #   dz = KZ * d(calf)    foot z 位移 / 小腿关节
    #   dx = KC * d(calf)    calf 对 foot x 的耦合
    # 顺序同 FL, FR, RR, RL
    KX = np.array([-0.420,  0.427,  0.427, -0.420])  #大腿转 1 rad  →  脚在前后方向移动 KX 米
    KZ = np.array([-0.085,  0.129,  0.129, -0.085])  #小腿转 1 rad  →  脚在上下方向移动 KZ 米
    KC = np.array([-0.229,  0.2075, 0.2075, -0.229]) #小腿转 1 rad  →  脚在前后方向移动 KC 米 
    # 步态顺序 FL, FR, RL, RR（一次抬一条腿，始终三条腿着地）
    GAIT_ORDER = (0, 1, 3, 2)

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
        self.phase = 0.0                   #phase代表步幅状态的一个周期数
        self.freq = 1.2                    # 步态频率 Hz
        self.lift = 0.06                   # 抬脚高度 m
        self.stride = 0.0                  # 步幅 m（踏步=0，行走>0）

    # ---------------- 对外接口：按键调用 ----------------
    def set_mode(self, mode):
        if mode == self.mode:               # 已经在目标模式，什么都不做
            return
        self.mode = mode
        self.phase=0.0
        self.waypoints = []                 # 换模式时清空旧清单

        if mode == RobotMode.STAND:
            # 任意姿态 → 先收回趴卧位（归一化），再撑起站立
            self.waypoints = [self.q_lie.copy(), self.q_stand.copy()]
        elif mode == RobotMode.LIE:
            # 先收到趴卧位（先到Q_LIE姿态），到位后会松开力矩
            self.waypoints = [self.q_lie.copy()]
        elif mode ==RobotMode.MARCH or mode==RobotMode.WALK:
            self.stride=0.0 if mode==RobotMode.MARCH else 0.20
            self.waypoints=[self.q_stand.copy()]#先站好再迈步

    # ---------------- 每个物理步调用一次 ----------------
    def update(self, simulator):
        d = simulator.data

        # ---- 情况 1：阻尼模式：只有速度阻尼，不跟踪角度 ----
        if self.mode == RobotMode.DAMPING:
            d.ctrl[:] = np.clip(-self.kd * d.qvel[6:], -self.clip, self.clip) #应输入力矩tau为-self.kd * d.qvel[6:]，经过上下限裁切过后得到新的输入力矩
            self.q_cmd = d.qpos[7:].copy()      # 记住腿现在在哪
            return

        # ---- 情况 2：趴下已到位：松开力矩，自然塌成趴卧 ----
        if self.mode == RobotMode.LIE and not self.waypoints: #mode=LIE的路点清单执行完后执行此步骤
            d.ctrl[:] = 0.0
            self.q_cmd = d.qpos[7:].copy()
            return

        # ---- 情况 3：站立 / 趴下的移动过程：平滑跟踪路点 ----(该阶段主要用于路点清单的执行)
        target = self.find_target(simulator)
        if (self.mode==RobotMode.MARCH or self.mode==RobotMode.WALK) and not self.waypoints:
            self.q_cmd=target.copy()  #此时 狗进入步态状态，跳过限速，直接跟随
        else:
            step = self.joint_speed * simulator.timestep
            self.q_cmd += np.clip(target - self.q_cmd, -step, step) #让q_cmd向着target位置一点一点平缓地发生变化
        #如果路点清单非空且q_cmd位置达到目标位置则将路点清单的第一项姿态清除，自动执行 下一个姿态
        if self.waypoints and np.max(np.abs(self.q_cmd - self.waypoints[0])) < 1e-2:  
            self.waypoints.pop(0)

        d.ctrl[:] = np.clip(self.kp * (self.q_cmd - d.qpos[7:]) - self.kd * d.qvel[6:],
                            -self.clip, self.clip)

        
    def find_target(self,simulator):
        if self.mode==RobotMode.STAND:
            return self.waypoints[0] if self.waypoints else self.q_stand #r如果waypoints归零则保持站立姿态
        if self.mode==RobotMode.LIE:
            return self.waypoints[0]
        if self.mode==RobotMode.MARCH or self.mode==RobotMode.WALK:
            if self.waypoints:
                return self.waypoints[0]
            self.phase=self.phase+self.freq*simulator.timestep  
            return self.q_stand.copy()+self._gait_offset(self.phase)  #步态 = 站立姿态 + 一个周期性偏移,此时步态可以理解为在站立姿态的基础上作周期摆动
        return self.q_stand.copy()

    def _gait_offset(self,phase):
        off=np.zeros(12)  #步态偏移量矩阵初始化
        sw=0.25          #摆动相占比时间25%，支撑相占比时间75%，保证四条腿各有四分之一时间在抬腿四分之三时间在支撑
        for k,leg in enumerate(self.GAIT_ORDER):
            u=(phase-k/4.0)%1.0
            if u<sw:
               s=u/sw  #归一化
               xoff=-self.stride/2+self.stride*s
               lift=self.lift*np.sin(np.pi*s)   #正弦式抬脚，在中间的时候抬脚幅度最大，也就是一只腿抬起来有放下去
            else:
                s=(u-sw)/(1-sw)
                xoff=self.stride/2-self.stride*s
                lift=0.0
            dc=lift/self.KZ[leg]
            off[3*leg+2]=off[3*leg+2]+dc
            off[3*leg+1]=off[3*leg+1]+xoff/self.KX[leg]-self.KC[leg]*dc/self.KX[leg]
        return off
    #phase 区间        谁在抬脚
    #[0.00, 0.25)      FL
    #[0.25, 0.50)      FR
    #[0.50, 0.75)      RL
    #[0.75, 1.00)      RR