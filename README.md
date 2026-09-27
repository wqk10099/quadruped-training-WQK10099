# MuJoCo 四足机器人基础仿真

## 项目目标

使用 Python 将四足机器人的 URDF 模型转换并整理为 MuJoCo 的 MJCF 模型，在 MuJoCo 中加载平坦地形并运行仿真。

项目目前分为两个层次。

**基础部分（已完成）**：建立完整的仿真流程、模型理解、代码结构和线程设计意识。

- 从 GitHub 获取四足机器人 URDF 与网格文件；
- 学习 URDF 与 MJCF 的结构差异；
- 将 URDF 转换为 MJCF，并手动检查、整理转换结果；
- 在 MJCF 中为机器人基座添加自由基座；
- 将 12 个关节配置为 motor 力矩执行器；
- 在平坦地面场景中加载机器狗；
- 保持 data.ctrl[:] = 0.0，验证机器人能够稳定趴卧；
- 阅读 unitree_mujoco，分析模型加载、状态读取、data.ctrl 写入、mj_step() 和线程设计；
- 将仿真代码拆分为模型、控制器和主程序；
- 对比单线程与双线程仿真结构；
- 选做：使用 C++ 重新完成核心仿真流程。

**控制扩展部分（进行中）**：在零力矩趴卧的基础上，逐步给 12 个关节加上主动力矩控制。

1. **第 1 步：阻尼模式** —— ctrl = -kd · dq，让腿像泡在油里一样被"拖住"；
2. **第 2 步：按键起立** —— 狗默认处于阻尼趴卧状态，按一个键让它站起来；
3. **第 3 步：多模式切换** —— 无论当前处于什么模式，按 S 站起 / 按 L 趴下 / 按 D 阻尼；
4. 第 4 步：原地踏步（待实现）；
5. 第 5 步：平地行走几步（待实现）。

## 项目状态

基础部分：

- [x] 获取机器人 URDF 和全部 STL 网格文件
- [x] 完成 URDF 到 MJCF 的初步转换
- [x] 整理转换后的 MJCF 模型和网格路径
- [x] 为 trunk 基座添加 freejoint
- [x] 添加 12 个 motor 力矩执行器
- [x] 编写平坦地面场景
- [x] 在零力矩输入下观察机器狗稳定趴卧
- [x] 将仿真代码拆分为 MuJoCoSimulator、控制器和主程序
- [x] 对比单线程与双线程仿真结构
- [x] 阅读 unitree_mujoco 的 README、Python 主程序和桥接层
- [x] 选做：使用 C++ 重写核心仿真程序（无界面版本）
- [ ] 可选增强：为 C++ 版本接入 Viewer

控制扩展部分：

- [x] 实现 PD 定点站立
- [x] 第 1 步：阻尼模式
- [x] 第 2 步：按键从阻尼趴卧起立
- [x] 第 3 步：多模式切换（站立 / 趴下 / 阻尼）
- [ ] 第 4 步：原地踏步
- [ ] 第 5 步：平地行走几步
- [ ] 可选：加入机身姿态反馈，提高站立的抗扰能力

## 环境与依赖

建议使用 Ubuntu 或兼容的 Linux 环境。

- Python 3
- MuJoCo Python 包
- NumPy
- Git，用于版本管理

安装 MuJoCo：

~~~bash
python3 -m pip install mujoco
~~~

验证安装：

~~~bash
python3 -c "import mujoco; print(mujoco.__version__)"
~~~

本项目在 MuJoCo 3.14.0、Python 3.10 上验证通过。

## 目录结构

~~~text
project/
├── models/
│   └── black/
│       ├── black_description.urdf
│       ├── black_description_mjcf_raw.xml
│       ├── black_description_mjcf.xml
│       └── meshes/                  STL 网格文件
├── scenes/
│   └── flat_scene.xml               平坦地形 + 机器狗 + stand keyframe
├── src/
│   ├── __init__.py
│   ├── simulator.py                 仿真器封装
│   ├── controller.py                所有控制器（见下表）
│   ├── main.py                      零力矩趴卧 · 单线程
│   ├── main_threaded.py             零力矩趴卧 · 双线程
│   ├── stand_main.py                PD 定点站立
│   ├── damping_main.py              第 1 步：阻尼模式
│   ├── damping_stand_main.py        第 2 步：阻尼 → 按键起立
│   └── multimode_main.py            第 3 步：多模式切换
├── scripts/
│   └── simulate_flat.py             最初的单文件版本，保留作参考
├── cpp/
│   ├── simulate_prone.cpp
│   └── README.md
├── .gitignore
└── README.md
~~~

文件职责：

- models/black/black_description.urdf：原始 URDF 模型。
- models/black/black_description_mjcf_raw.xml：URDF 转换工具生成的原始 MJCF，保留作参考。
- models/black/black_description_mjcf.xml：整理后实际使用的 MJCF 模型。
- scenes/flat_scene.xml：平坦地面、灯光、机器人模型以及 stand keyframe。
- src/simulator.py：封装 MjModel、MjData、重置和 mj_step()。
- src/controller.py：所有控制策略集中在这里，见下表。
- scripts/simulate_flat.py：最初的单文件版本，保留作学习参考。

src/controller.py 中的控制类：

| 类 | 控制律 | 使用它的主程序 |
|---|---|---|
| ZeroTorqueController | ctrl = 0 | main.py / main_threaded.py |
| PDController | ctrl = kp(q_des - q) - kd·dq | stand_main.py |
| DampingController | ctrl = -kd·dq | damping_main.py |
| Damping_StandController | 阻尼 ⇄ 按键起立（两步版） | damping_stand_main.py |
| RobotController（配 RobotMode） | 阻尼 / 站立 / 趴下 三模式 | multimode_main.py |

## URDF 来源与转换过程

原始模型来自 GitHub 用户 N-W-wolf 的 Training_Materials 仓库。实际使用的 URDF 位于：

~~~text
第二次培训/black/black_description.urdf
~~~

网格文件位于：

~~~text
第二次培训/black/meshes/
~~~

转换和整理过程如下：

1. 下载 URDF 文件及 meshes 目录；
2. 使用 URDF 预览与转换工具查看模型；
3. 保存未经修改的转换结果 black_description_mjcf_raw.xml；
4. 检查 compiler、asset、mesh 路径是否正确；
5. 核对每条腿的 hip、thigh、calf 关节名称、轴、范围和父子关系；
6. 在 trunk 基座 body 中增加 freejoint；
7. 在 actuator 中为 12 个关节分别添加 motor；
8. 保存整理后的模型为 black_description_mjcf.xml。

### mesh 路径修正

转换后的 MJCF 曾使用：

~~~xml
<compiler meshdir="meshes/" ... />
~~~

当该文件被 flat_scene.xml 通过 include 引入时，MuJoCo 对相对 mesh 路径的解析会产生歧义。为解决这个问题，最终采用：

~~~xml
<compiler angle="radian" inertiafromgeom="auto" inertiagrouprange="3 3" />
~~~

并让每个 mesh 的 file 属性直接包含 meshes 前缀，例如：

~~~xml
<mesh name="trunk" file="meshes/trunk.STL" />
<mesh name="d435" file="meshes/d435.stl" />
~~~

这样无论从项目根目录、scenes 目录还是 scripts 目录加载场景，相对路径都能正确工作。

## 模型结构

| 项目 | 数量 | 说明 |
|---|---:|---|
| 自由基座 | 1 | 位于 trunk |
| hinge 关节 | 12 | 每条腿 3 个关节 |
| motor 执行器 | 12 | 与 12 个关节一一对应 |
| 网格资源 | 14 | STL 模型文件 |

模型编译后得到：

~~~text
nq = 19   # 自由基座 7 + 12 个关节角
nv = 18   # 自由基座 6 + 12 个关节速度
nu = 12   # 12 个 motor 执行器
~~~

12 个力矩执行器采用直接驱动形式：

~~~xml
<motor name="FL_hip_motor" joint="FL_hip_joint" gear="1" />
~~~

12 个关节在 data.ctrl、data.qpos[7:]、data.qvel[6:] 中的顺序一致，都是：

~~~text
FL_hip, FL_thigh, FL_calf,   FR_hip, FR_thigh, FR_calf,
RR_hip, RR_thigh, RR_calf,   RL_hip, RL_thigh, RL_calf
~~~

注意前后腿和左右腿的安装朝向是镜像的，所以目标角里左右腿的正负号相反，例如站立姿态中 FL_thigh 为 +0.6、FR_thigh 为 -0.6。

## 场景与控制方式

flat_scene.xml 通过 include 引入整理后的机器人模型，并添加平面地板和灯光：

~~~xml
<mujoco model="flat_scene">
    <keyframe>
        <key name="stand" qpos="0 0 0.5102516 1 0 0 0
              0 0.6 -1.0
              0 -0.6 1.0
              0 -0.6 1.0
              0 0.6 -1.0"/>
    </keyframe>
    <include file="../models/black/black_description_mjcf.xml"/>
    <option timestep="0.002" gravity="0 0 -9.81"/>
    <worldbody>
        <light pos="0 0 3"/>
        <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
    </worldbody>
</mujoco>
~~~

机器人初始高度暂时设置为：

~~~xml
<body name="trunk" pos="0 0 0.65">
~~~

因此仿真开始后，机器狗会先从空中下落，四个脚先接触地面。此时如果 ctrl 全为零，它会在重力作用下形成稳定趴卧姿态。

## 运行方式

所有主程序都必须在**项目根目录**下用 -m 模块方式运行（因为使用了相对导入）：

~~~bash
cd ~/mujoco_training/03_robot_dog/project
~~~

| 命令 | 内容 | 按键 |
|---|---|---|
| python3 -m src.main | 零力矩趴卧（单线程） | 无 |
| python3 -m src.main_threaded | 零力矩趴卧（双线程） | 无 |
| python3 -m src.stand_main | PD 定点站立 | 无 |
| python3 -m src.damping_main | 第 1 步：阻尼模式 | 无 |
| python3 -m src.damping_stand_main | 第 2 步：阻尼 → 按键起立 | S 起立 / D 阻尼 |
| python3 -m src.multimode_main | 第 3 步：三模式切换 | S 站立 / L 趴下 / D 阻尼 |

如果直接写成 python3 src/xxx_main.py，会报 attempted relative import with no known parent package，这是启动方式的问题，不是代码本身的问题。

## 力矩控制三步（重点）

基础部分里 ctrl 一直是 0，机器人完全靠重力自然趴下。这三步开始给 12 个关节写主动力矩。

### 0. 先理解三个量

所有控制律都基于这三行：

~~~python
q  = data.qpos[7:]      # 12 个关节的实际角度
dq = data.qvel[6:]      # 12 个关节的实际角速度
data.ctrl[:] = ...      # 写给 12 个电机的力矩，shape = (12,)
~~~

为什么是 7 和 6：qpos 的前 7 个数是基座的位置和姿态四元数，qvel 的前 6 个数是基座的线速度和角速度，之后才是 12 个关节。

两个关键姿态（关节顺序见上文「模型结构」一节）：

~~~text
Q_STAND = [0, 0.6, -1.0,   0, -0.6, 1.0,   0, -0.6, 1.0,   0, 0.6, -1.0]
Q_LIE   = [0, 1.55, -2.45, 0, -1.55, 2.45, 0, -1.55, 2.45, 0, 1.55, -2.45]
~~~

### 1. 第 1 步：阻尼模式

**控制律**

~~~text
ctrl = -kd · dq
~~~

不跟踪任何目标角度，只输出一个与关节速度相反、大小成正比的力矩。

**效果**：腿像泡在油里。被外力推动后缓慢移动、很快停住，不会来回晃。

**文件**

- 控制器：src/controller.py 中的 DampingController
- 主程序：src/damping_main.py
- 运行：python3 -m src.damping_main

**实测（从站立姿态放开）**

| kd | 行为 |
|---|---|
| 0.5 | 几乎没阻力，约 0.5 s 就砸到地面 |
| 2.0 | 约 2.5 s 缓慢塌到趴卧，有"黏住"的手感（推荐） |
| 5.0 | 更慢，约 3 s 时仍在下降 |

**阻尼模式 vs 零力矩（容易混淆，务必分清）**

| | 零力矩 | 阻尼 |
|---|---|---|
| 控制律 | ctrl = 0.0 | ctrl = -kd·dq |
| 静止时输出 | 精确为 0（根本不计算） | 约等于 0（算了，结果很小） |
| 被推一下 | 完全不抵抗 | 立刻产生反向力矩 |
| 最终姿态 | 完全塌平，z 约 0.145，关节顶到限位 | 腿撑住的拱形，z 约 0.19 ~ 0.22 |

静止时两者的力矩都接近 0，但含义完全不同。判断方法：阻尼模式下把狗推一下，它会立刻反推回来。

### 2. 第 2 步：按键起立

**要求**：狗默认处于阻尼趴卧状态，按一个键让它站起来。

**为什么不能直接把 PD 目标设成站立角**

~~~python
d.ctrl[:] = kp * (Q_STAND - q)    # 危险写法
~~~

按下的瞬间腿还在趴卧角度，误差可能高达 2 ~ 3 弧度，乘以 kp = 60 后力矩会顶到限制值，电机拼命把腿"啪"地掰过去，狗会弹起来翻掉。

**核心技巧：引入会"慢慢走"的指令角 q_cmd**

~~~python
step = joint_speed * dt
q_cmd += np.clip(target - q_cmd, -step, step)      # q_cmd 缓慢逼近目标
ctrl = kp * (q_cmd - q) - kd * dq                  # PD 跟踪 q_cmd，而不是跟踪 target
~~~

- q_cmd 是"我们希望腿当前应该在哪"，每秒最多移动 joint_speed 弧度；
- 腿被 PD 拉着跟 q_cmd 走，所以也是慢慢移动；
- 这样腿就"一段一段"地动，狗能靠腿撑地把自己推起来。

**两段式起立**

阻尼模式下狗停住时，往往卡在一个"腿撑开的拱形姿态"，直接撑起会歪掉。实测：

~~~text
从阻尼姿态直接跳到站立目标  -> z = 0.452，机身倾斜约 42 度   （歪着站）
先回趴卧位、再撑起（两段）  -> z = 0.496，姿态几乎水平        （正确）
~~~

所以起立分两站走，用路点清单实现（见下一节）：

~~~python
self.waypoints = [self.q_lie.copy(), self.q_stand.copy()]
~~~

**按键接入**

MuJoCo 3.14 的 passive viewer 支持键盘回调，不需要额外安装库：

~~~python
def on_key(key):                      # key 是 GLFW 键码
    mode = KEY_MAP.get(key)           # 字母键码 = 对应大写字母的 ASCII 值
    if mode is not None:
        pending["mode"] = mode

with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=on_key) as viewer:
    ...
~~~

注意：on_key 运行在 Viewer 自己的线程里，所以它只往 pending 里写一个"请求"，由物理线程读取并执行，避免两个线程同时修改控制器状态。

**文件**

- 控制器：src/controller.py 中的 Damping_StandController
- 主程序：src/damping_stand_main.py
- 按键：S 起立 / D 回到阻尼

**实测**

~~~text
启动（阻尼）    z = 0.225
按 S 起立       z = 0.498   姿态四元数接近 [1,0,0,0]
按 D 回阻尼     z = 0.146
再按 S 起立     z = 0.498   可以反复来回切换
~~~

### 3. 第 3 步：多模式切换

**要求**：无论狗当前处于什么模式或姿态，按 S 站起来、按 L 趴下去。

**为什么要把布尔量升级成"模式"**

第 2 步只有两个状态，用 self.standing = True/False 就够了。第 3 步有三个状态，布尔量表达不了，所以换成一个存"当前模式名"的变量：

~~~python
class RobotMode:
    DAMPING = "damping"     # 阻尼
    STAND   = "stand"       # 站立
    LIE     = "lie"         # 趴下

self.mode = RobotMode.DAMPING      # 开机默认
~~~

以后再加踏步、行走，只要往 RobotMode 里加一行，主程序一个字都不用改。

**三种模式分别输出什么**

| 模式 | 干什么 | 控制律 |
|---|---|---|
| DAMPING | 卸力，只按速度阻尼 | ctrl = -kd·dq |
| STAND | 先收回趴卧位，再撑起 | 跟踪路点 [q_lie, q_stand] |
| LIE | 收到趴卧位，然后松开力矩 | 跟踪路点 [q_lie]，到位后 ctrl = 0 |

两个要点：

1. 只有 DAMPING 不需要 q_cmd，因为它不跟踪角度，算完直接 return；
2. LIE 到位后要"松开力矩"，不能一直用 PD 顶在 q_lie 上。PD 顶住时狗会停在"腿微微撑起"的姿态（z 约 0.21），松开力矩后它自然塌成真正的趴卧姿态（z 约 0.145），和开机时一致。这样"站起来"的起点每次都一样。

**"从任意姿态站起来"靠两件事**

1. 阻尼分支里执行 self.q_cmd = data.qpos[7:].copy()，保证按 S 的瞬间，q_cmd 就是腿此刻的真实角度。所以无论狗现在什么样，都是从当前位置出发，不会有突变；
2. 起立时先走一站 q_lie 把腿收回标准趴卧姿势（归一化），再从那里撑起。

**按键映射表**

~~~python
KEY_MAP = {
    ord("S"): RobotMode.STAND,
    ord("L"): RobotMode.LIE,
    ord("D"): RobotMode.DAMPING,
}
~~~

想加踏步，只要在这里加 ord("T"): RobotMode.TROT，别处不用动。

**文件**

- 控制器：src/controller.py 中的 RobotController（配 RobotMode）
- 主程序：src/multimode_main.py
- 按键：S 站立 / L 趴下 / D 阻尼

**实测**

~~~text
启动（阻尼）       z = 0.225   w = 0.999
按 S 站起          z = 0.498   w = 1.000     站直
按 L 趴下          z = 0.145   w = 1.000     塌平成趴卧
再按 S 站起        z = 0.498   w = 1.000
按 D 阻尼          z = 0.145   w = 1.000
从阻尼按 S 站起    z = 0.498   w = 1.000
~~~

（w 是姿态四元数的第一个分量，1.000 表示机身完全水平。）

### 4. 路点（waypoints）机制

waypoints 是一张"待办清单"：起立要依次经过哪些姿态。

~~~python
target = self.waypoints[0] if self.waypoints else self.q_stand   # 当前要去哪
step = self.joint_speed * dt
self.q_cmd += np.clip(target - self.q_cmd, -step, step)          # 朝它走

if self.waypoints and np.max(np.abs(self.q_cmd - self.waypoints[0])) < 1e-2:
    self.waypoints.pop(0)                                        # 到了就划掉，换下一站
~~~

以 FL_thigh 这个关节为例（q_lie 里是 1.55，q_stand 里是 0.6），实测时间线：

~~~text
  t(s) | 清单剩 | 当前路点 | q_cmd  | 实际关节角 | 机身 z
  -----+--------+----------+--------+------------+-------
   0.0 |   2    |  1.550   | -0.922 |   -0.922   | 0.227   刚按 S
   0.4 |   2    |  1.550   | -0.122 |   -0.206   | 0.319
   0.8 |   2    |  1.550   |  0.678 |    0.724   | 0.278
   1.2 |   2    |  1.550   |  1.478 |    1.358   | 0.211   快到了
   1.4 |   1    |  0.600   |  1.334 |    1.492   | 0.231   到达 -> 划掉，换第 2 站
   1.8 |   1    |  0.600   |  0.600 |    0.634   | 0.404
   2.2 |   0    |   （空）  |  0.600 |    0.599   | 0.487   到达 -> 清单空
   3.0 |   0    |   （空）  |  0.600 |    0.706   | 0.498   兜底保持站立
~~~

能看出的三件事：

1. q_cmd 以固定速度（joint_speed = 2 弧度/秒）移动，从 -0.922 爬到 1.55 走了 2.47 弧度，正好约 1.24 s；
2. t = 1.4 s 时清单从 2 项变成 1 项，当前路点从 1.55 跳成 0.600，这就是 pop(0) 的瞬间；
3. 清单空了以后 target 回落到 self.q_stand（兜底），保持站立。注意实际关节角总滞后于 q_cmd，这是 PD"追着走"的正常表现。

**两个关键细节**

- 路点必须存副本。self.q_cmd += ... 是 numpy 的原地修改，如果把 self.q_cmd 本身放进清单，清单第一项会和"当前位置"一起变，两者永远相等，狗就停在原地不动了。所以必须写成 self.q_lie.copy() / self.q_stand.copy()；
- 到达判定用所有关节里误差最大的那个（np.max），只要有一个关节没到位就不切换下一站。

### 5. 调试记录：踩过的坑

调试过程中遇到的典型问题，记录下来避免重犯。

| 现象 | 原因 | 修法 |
|---|---|---|
| 按 S 立刻报 AttributeError | __init__ 里写 self.qcmd2，set_stand 里用 self.q_cmd2，名字不一致 | 统一命名，并且改成 np.array(..., dtype=float) 存副本 |
| 按 S 后狗只趴平、站不起来 | waypoints 里直接存了 self.q_cmd 本身，而它此刻是"当前姿态"不是"站立姿态"；且 += 是原地修改，目标跟着当前角一起跑 | 路点用 self.q_lie.copy() / self.q_stand.copy()，并且把站立目标单独存成 self.q_stand |
| 窗口能打开但狗完全不动，控制台报 KeyError | 主程序里 pending 字典的键是 "mode"，物理循环却读 pending["stand"] | 键名统一，或干脆定义成常量 KEY_PENDING = "mode" |
| 从阻尼姿态单段起立会歪 | 阻尼停住的拱形姿态直接撑起会失衡 | 改成两段式：先回 q_lie 再撑起 |

经验：这类"两个名字对不上"的错误，Python 不会提前报错，只有真正执行到那一行才暴露。定义变量或字典时，顺手检查所有用到它的地方是否同名；跨线程传请求时，把键名抽成常量最稳妥。

### 6. 能力边界（如实说明）

**能做到**

- 从程序自己的任何模式（阻尼 / 站立 / 趴下）按 S 都能站起来；
- 从正常的趴卧、拱起姿态按 S 都能站起来；
- 按 L 能从任意模式趴下，且最终姿态和开机时一致。

**做不到**

- 狗完全翻过去（仰面、或侧躺超过约 60 ~ 75 度）时站不起来。

原因很朴素：控制输出的只有 12 个关节力矩，控制器没有机身姿态反馈，狗没法主动"翻身"。真机上这叫"摔倒恢复"，需要额外的姿态估计与策略，属于后续内容。

**实用小技巧**：如果按 S 后狗站歪了，按一下 L 让它重新趴平，再按 S，就又能站直了。

## 当前实现结果

基础部分：

- MJCF 模型可以成功加载，nq、nv、nu 分别为 19、18、12；
- 12 个关节均配置为力矩 motor；
- ctrl 全为零时，机器狗由重力驱动下落，四个脚先接触地面；
- 最终腹部朝下、四条腿展开，稳定趴卧在平地上，机身高度约 0.145 m；
- 姿态四元数接近 [1, 0, 0, 0]，最终 qvel 接近 0，未观察到持续抖动。

控制扩展部分：

- PD 定点站立：机身高度约 0.489 m，四足接触，姿态接近单位四元数；
- 阻尼模式：从站立约 2.5 s 缓慢塌到趴卧，腿有"黏住"的手感；
- 按键起立：从阻尼趴卧按 S 起立到 z 约 0.498，可反复切换；
- 多模式切换：S / L / D 三键，站立、趴下、阻尼之间任意切换，站起后姿态四元数约 [1,0,0,0]。

辅助检查用的代码：

~~~python
print("nq =", model.nq)
print("nv =", model.nv)
print("nu =", model.nu)
print("time =", data.time)
print("qpos =", data.qpos)
print("qvel =", data.qvel)
print("ctrl =", data.ctrl)
~~~

## 单线程与双线程设计

### 单线程版本

把物理推进和 Viewer 刷新放在同一个循环里：

~~~text
controller.update()
mj_step()
viewer.sync()
sleep()
~~~

优点：代码简单、没有 mj_data 并发访问、调试直观。
代价：每个物理步都要 sync 一次 Viewer，渲染慢时物理循环也被拖慢，实际观察中下落速度明显慢于真实时间。

### 双线程版本

把物理和 Viewer 分开：

~~~text
物理线程：controller.update + sim.step（按 timestep 等待）
Viewer 线程：viewer.sync（按 VIEWER_DT 等待）
共享保护：threading.Lock
~~~

优点：物理循环不再被渲染拖慢，仿真速度更接近真实时间，物理频率和显示频率可以分别设置。
代价：必须用锁保护共享 mj_data；线程退出、异常处理和调试更复杂。

第 3 步的 multimode_main.py 用的就是这套双线程结构。按键回调运行在 Viewer 线程，它只写一个 pending 请求，真正的模式切换在物理线程里执行。

### 与 unitree_mujoco 的对比

unitree_mujoco 的线程结构更复杂：物理仿真线程、Viewer 或渲染线程、DDS 通信线程、LowCmd 回调、LowState 与 SportModeState 等状态发布线程。

桥接层中的核心控制公式为：

~~~text
ctrl = tau + kp * (q_desired - q_current) + kd * (dq_desired - dq_current)
~~~

状态回传路径为：

~~~text
mj_data.sensordata -> LowState -> DDS -> 控制程序
~~~

阅读代码时发现一个值得注意的问题：unitree_mujoco 的通信桥接线程会直接修改 mj_data.ctrl，而物理线程会调用 mj_step()，主锁与 LowCmd 消息锁保护的对象并不完全相同。对于当前项目，双线程实验的作用是理解物理与显示解耦；如果以后接入高频率控制器或真实机器人接口，应该考虑使用控制快照、线程安全队列或明确的数据所有权，而不是让多个线程无保护地直接修改 mj_data。

## 已知问题

- main_threaded.py 和 multimode_main.py 都把 Viewer 放在独立线程中，属于实验实现；
- 站立为固定姿态 PD，不含机身位置和姿态反馈，站立的抗扰能力有限，且对 kp、kd、初始姿态敏感；
- 完全翻倒时无法自行站起（见"能力边界"）；
- 踏步与行走尚未实现，目前没有任何周期性的步态生成；
- URDF 转 MJCF 后仍需继续核对碰撞体、质量、惯量和关节阻尼；
- 稳定趴卧姿态依赖初始高度和模型碰撞参数，后续可以固化为 keyframe。

## 后续计划

1. 将稳定趴卧的 qpos 保存为 MuJoCo keyframe，作为统一起点；
2. 设计只读状态接口和线程安全的控制命令接口；
3. 在站立控制基础上实现第 4 步：原地踏步（一次抬一条腿的爬行式步态，始终三条腿着地，静稳定）；
4. 在第 4 步基础上实现第 5 步：平地行走几步（支撑腿向后蹬地产生前进位移）；
5. 加入机身姿态反馈（用 qpos[3:7] 的四元数计算倾斜，修正髋/大腿目标），提高站立的抗扰能力；
6. 可选：为 C++ 版本接入 Viewer。
