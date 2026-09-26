# MuJoCo 四足机器人基础仿真

## 项目目标

使用 Python 将四足机器人的 URDF 模型转换并整理为 MuJoCo 的 MJCF 模型，在 MuJoCo 中加载平坦地形并运行仿真，使机器狗在关节输出力矩为零的条件下稳定趴卧。

本阶段重点不是实现步态控制，而是建立完整的仿真流程、模型理解、代码结构和线程设计意识：

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

## 项目状态

- [x] 获取机器人 URDF 和全部 STL 网格文件
- [x] 完成 URDF 到 MJCF 的初步转换
- [x] 整理转换后的 MJCF 模型和网格路径
- [x] 为 trunk 基座添加 freejoint
- [x] 添加 12 个 motor 力矩执行器
- [x] 编写平坦地面场景
- [x] 在零力矩输入下观察机器狗稳定趴卧
- [x] 实现 PD 控制站立（扩展实验）
- [ ] 实现原地踏步（扩展实验）
- [x] 将仿真代码拆分为 MuJoCoSimulator、ZeroTorqueController 和 main
- [x] 阅读 unitree_mujoco 的 README、Python 主程序和桥接层
- [x] 对比单线程与双线程仿真结构
- [x] 选做：使用 C++ 重写核心仿真程序（无界面版本）
- [ ] 可选增强：为 C++ 版本接入 Viewer

## 环境与依赖

建议使用 Ubuntu 或兼容的 Linux 环境。

- Python 3
- MuJoCo Python 包
- NumPy
- Git，用于版本管理

安装 MuJoCo：

```bash
python3 -m pip install mujoco
```

验证安装：

```bash
python3 -c "import mujoco; print(mujoco.__version__)"
```

## 目录结构

```text
project/
├── models/
│   └── black/
│       ├── black_description.urdf
│       ├── black_description_mjcf_raw.xml
│       ├── black_description_mjcf.xml
│       └── meshes/
│           ├── trunk.STL
│           ├── FL_hip.STL
│           ├── ...
│           └── RR_calf.STL
├── scenes/
│   └── flat_scene.xml
├── src/
│   ├── __init__.py
│   ├── simulator.py
│   ├── controller.py
│   ├── stand_controller.py
│   ├── main.py
│   ├── main_threaded.py
│   └── stand_main.py
├── scripts/
│   └── simulate_flat.py
├── cpp/
│   ├── simulate_prone.cpp
│   └── README.md
├── .gitignore
└── README.md
```

文件职责：

- black_description.urdf：原始 URDF 模型。
- black_description_mjcf_raw.xml：URDF 转换工具生成的原始 MJCF，保留作参考。
- black_description_mjcf.xml：整理后实际使用的 MJCF 模型。
- meshes/：URDF 和 MJCF 引用的 STL 网格。
- flat_scene.xml：平坦地面、灯光以及机器人模型的组合场景。
- src/simulator.py：封装 MjModel、MjData、重置和 mj_step()。
- src/controller.py：封装控制策略，当前实现为零力矩控制器。
- src/main.py：单线程主程序，组合仿真器、控制器和 Viewer。
- src/main_threaded.py：双线程实验，将物理循环和 Viewer 循环分开。
- scripts/simulate_flat.py：最初的单文件版本，保留作学习参考。

## URDF 来源与转换过程

原始模型来自 GitHub 用户 N-W-wolf 的 Training_Materials 仓库。实际使用的 URDF 位于：

```text
第二次培训/black/black_description.urdf
```

网格文件位于：

```text
第二次培训/black/meshes/
```

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

```xml
<compiler meshdir="meshes/" ... />
```

当该文件被 flat_scene.xml 通过 include 引入时，MuJoCo 对相对 mesh 路径的解析会产生歧义。为解决这个问题，最终采用：

```xml
<compiler angle="radian" inertiafromgeom="auto" inertiagrouprange="3 3" />
```

并让每个 mesh 的 file 属性直接包含 meshes 前缀，例如：

```xml
<mesh name="trunk" file="meshes/trunk.STL" />
<mesh name="d435" file="meshes/d435.stl" />
```

这样无论从项目根目录、scenes 目录还是 scripts 目录加载场景，相对路径都能正确工作。

## 模型结构

当前模型的预期结构为：

| 项目 | 数量 | 说明 |
|---|---:|---|
| 自由基座 | 1 | 位于 trunk |
| hinge 关节 | 12 | 每条腿 3 个关节 |
| motor 执行器 | 12 | 与 12 个关节一一对应 |
| 网格资源 | 14 | STL 模型文件 |

模型编译后得到：

```text
nq = 19   # 自由基座 7 + 12 个关节角
nv = 18   # 自由基座 6 + 12 个关节速度
nu = 12   # 12 个 motor 执行器
```

12 个力矩执行器采用直接驱动形式：

```xml
<motor name="FL_hip_motor" joint="FL_hip_joint" gear="1" />
```

运行时保持：

```python
data.ctrl[:] = 0.0
```

因此在任务第一阶段中，机器人不受主动控制力矩作用，只受到重力、接触力、关节约束和模型惯性的影响。

## 场景与控制方式

flat_scene.xml 通过 include 引入整理后的机器人模型，并添加平面地板和灯光：

```xml
<mujoco model="flat_scene">
    <include file="../models/black/black_description_mjcf.xml"/>
    <option timestep="0.002" gravity="0 0 -9.81"/>
    <worldbody>
        <light pos="0 0 3"/>
        <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
    </worldbody>
</mujoco>
```

机器人初始高度暂时设置为：

```xml
<body name="trunk" pos="0 0 0.65">
```

因此仿真开始后，机器狗会先从空中下落，四个脚先接触地面，随后在零力矩条件下形成稳定趴卧姿态。

## 运行方式

### 单线程版本

从项目根目录运行：

```bash
cd ~/mujoco_training/03_robot_dog/project
python3 -m src.main
```

主要流程：

```text
controller.update(sim)
sim.step()
viewer.sync()
sleep()
```

特点：

- 结构简单，容易理解；
- 所有操作在同一线程内完成；
- 不存在 mj_data 的多线程并发访问问题；
- viewer.sync() 会限制物理循环的有效速度。

### 双线程实验版本

```bash
python3 -m src.main_threaded
```

主要流程：

```text
物理线程：
    controller.update(sim)
    sim.step()
    按 timestep 等待

Viewer 线程：
    viewer.sync()
    按 VIEWER_DT 等待
```

特点：

- 物理仿真和 Viewer 刷新频率解耦；
- 仿真速度更接近真实时间；
- 使用 threading.Lock 保护共享 mj_data；
- GUI 操作位于独立线程，仍属于实验性设计。

### 最初的单文件版本

```bash
cd ~/mujoco_training/03_robot_dog/project/scripts
python3 simulate_flat.py
```

该版本保留作学习参考，主要入口已经迁移到 src/ 目录。

## 站立控制（扩展实验）

在完成零力矩稳定趴卧后，项目增加了基于 PD 控制的站立实验。站立不是原任务的验收条件，主要用于理解 motor 力矩控制、关节目标角跟踪和 mj_data 中的状态反馈。

### 站立目标姿态

站立姿态使用略屈膝的四点支撑姿势。12 个执行器对应的目标角度为：

```text
[FL_hip, FL_thigh, FL_calf,
 FR_hip, FR_thigh, FR_calf,
 RR_hip, RR_thigh, RR_calf,
 RL_hip, RL_thigh, RL_calf]

[0.0,  0.6, -1.0,
 0.0, -0.6,  1.0,
 0.0, -0.6,  1.0,
 0.0,  0.6, -1.0]
```

前置和后置腿的符号不同，是为了适配模型左右腿镜像的关节轴方向。

### stand keyframe

flat_scene.xml 中增加了 stand keyframe：

```xml
<keyframe>
    <key name="stand"
        qpos="0 0 0.5102516 1 0 0 0
              0 0.6 -1.0
              0 -0.6 1.0
              0 -0.6 1.0
              0 0.6 -1.0"/>
</keyframe>
```

其中 qpos 的顺序为：

```text
基座位置 3 个数 + 基座姿态四元数 4 个数
+ 12 个关节目标角
```

MuJoCoSimulator.reset_to_keyframe() 通过 mj_resetDataKeyframe() 恢复到该姿态。

### PD 力矩控制

motor 执行器写入的是力矩。站立控制器使用目标关节角和当前状态计算：

```text
tau = kp * (q_des - q) - kd * dq
```

其中：

```text
q  = data.qpos[7:]
dq = data.qvel[6:]
```

当前站立实验使用：

```text
kp = 40.0
kd = 2.0
tau 限制 = [-20, 20] N·m
```

相关文件：

- src/stand_controller.py：PDController。
- src/stand_main.py：站立控制主程序。
- scenes/flat_scene.xml：stand keyframe。

### 运行站立实验

```bash
cd ~/mujoco_training/03_robot_dog/project
python3 -m src.stand_main
```

预期状态：

- 四个脚接触地面；
- 机身高度 qpos[2] 约为 0.489 m；
- 姿态四元数接近 [1, 0, 0, 0]；
- qvel 在稳定后接近零；
- ctrl 由 PD 控制器计算，不再全部为零；
- 机器人能够保持直立姿态。

### 当前站立控制的限制

- 当前控制目标是固定站立姿态，不包含机身位置和姿态反馈。
- 还没有步态生成、足端轨迹规划和身体平衡控制器。
- 站立姿态对 kp、kd 和初始姿态敏感，参数变化后需要重新验证。
- 下一步应在站立稳定的基础上实现原地踏步，再加入简单的周期关节目标。

## 当前实现结果

- MJCF 模型可以成功加载；
- nq、nv、nu 分别为 19、18、12；
- 12 个关节均配置为力矩 motor；
- data.ctrl 在整个运行过程中保持为零；
- 机器狗由重力驱动下落，四个脚先接触地面；
- 最终机身腹部朝下，四条腿展开，稳定趴卧在平地上；
- 四元数接近 [1, 0, 0, 0]，机身姿态稳定；
- 最终 qvel 接近 0，未观察到持续抖动或机械振荡。

PD 站立扩展实验中，机器人能够在 PD 力矩控制下保持四点接触和近似直立姿态，机身高度约为 0.489 m，姿态四元数接近单位四元数。站立控制使用 src/stand_controller.py 中的 PDController 和 src/stand_main.py 主程序。

可用于辅助检查的代码：

```python
print("nq =", model.nq)
print("nv =", model.nv)
print("nu =", model.nu)

print("time =", data.time)
print("qpos =", data.qpos)
print("qvel =", data.qvel)
print("ctrl =", data.ctrl)
```

## 单线程与双线程设计

### 单线程版本

单线程版本把物理推进和 Viewer 刷新放在同一个循环中：

```text
controller.update()
mj_step()
viewer.sync()
sleep()
```

优点：

- 代码简单；
- 没有 mj_data 并发访问；
- 调试和排查问题更直接。

代价：

- 每个物理步都调用一次 viewer.sync()；
- 如果 Viewer 刷新较慢，整个物理循环也会变慢；
- 实际观察中，机器狗下落速度明显慢于真实时间。

原因可以理解为：

```text
每个循环只让仿真时间前进 0.002 s
但循环还可能花费数毫秒用于渲染
因此 1 s 真实时间对应的仿真时间可能小于 1 s
```

### 双线程版本

双线程版本将物理和 Viewer 分开：

```text
物理线程：controller.update + sim.step
Viewer 线程：viewer.sync
共享保护：threading.Lock
```

优点：

- 物理循环不再被每个 Viewer 刷新拖慢；
- 仿真速度更接近真实时间；
- 物理频率和显示频率可以分别设置。

代价：

- 必须使用锁保护共享 mj_data；
- 线程退出、异常处理和调试更复杂；
- 在独立线程中操作 Viewer 需要额外谨慎。

### 与 unitree_mujoco 的对比

unitree_mujoco 的线程结构比本项目复杂，主要包括：

- 物理仿真线程；
- Viewer 或渲染线程；
- DDS 通信线程；
- LowCmd 回调；
- LowState、SportModeState 等状态发布线程。

桥接层中的核心控制公式为：

```text
ctrl =
    tau
    + kp * (q_desired - q_current)
    + kd * (dq_desired - dq_current)
```

状态回传路径为：

```text
mj_data.sensordata
    -> LowState
    -> DDS
    -> 控制程序
```

阅读代码时发现一个重要问题：unitree_mujoco 的通信桥接线程会直接修改 mj_data.ctrl，而物理线程会调用 mj_step()。主锁与 LowCmd 消息锁保护的对象并不完全相同，因此这类设计需要认真考虑线程安全和数据一致性。

对于当前零力矩任务，不需要引入 DDS 和多层通信线程。双线程实验的作用是理解物理仿真和显示频率解耦；如果后续接入高频率控制器或真实机器人接口，应该考虑使用控制快照、线程安全队列或明确的数据所有权，而不是让多个线程无保护地直接修改 mj_data。

## 已知问题

- main_threaded.py 目前把 Viewer 放在独立线程中，属于实验实现；
- 当前程序尚未接入 DDS 或真实机器人通信；
- 还没有实现站立、行走、PD 控制或步态控制；
- URDF 转 MJCF 后仍需继续核对碰撞体、质量、惯量和关节阻尼；
- 当前稳定趴卧姿态依赖初始高度和模型碰撞参数，后续可以固化为 keyframe。

## 后续计划

1. 将稳定趴卧的 qpos 保存为 MuJoCo keyframe；
2. 设计只读状态接口和线程安全的控制命令接口；
3. 进一步比较单线程、物理线程加主线程 Viewer 等结构；
4. 在 PD 站立控制基础上实现原地踏步；
5. 调整踏步振幅、频率和 PD 参数，保持机身稳定；
6. 在稳定踏步基础上尝试简单前进步态；
7. 可选：为 C++ 版本接入 Viewer。
