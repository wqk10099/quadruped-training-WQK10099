# 第3次作业 —— ROS2 系统下的仿真改良

## 一、作业目标与总体思路

第 1 次作业的程序把**仿真步进、控制计算、键盘输入、渲染**全部揉在一个 `while` 循环里。
控制器要读取机器人状态，只能直接读写仿真器的 `mjData`（共享内存）——控制器和仿真耦合死了，
想换成实机就无从下手。

本次作业按 ROS2 的思路把它拆成**能独立运行的节点**，节点之间用**话题**和**服务**交换信息：

| 层次 | 第 1 次作业 | 本次作业 |
| --- | --- | --- |
| 控制器层 | `RobotController::update()` 直接写 `simulator.data()->ctrl[]` | **controller_node**（C++）只把 `{kp,kd,q,w,tau}` 发到话题上 |
| 仿真运行层 | 主循环里 `mj_step` + 渲染 | **sim_node**（C++）订阅命令、按 MIT 公式算力矩、`mj_step`、发布关节状态与 IMU |
| 人工输入 | SDL 键盘事件（S/W/D/L/M） | **teleop_node**（Python）读 Xbox 协议手柄，用**服务**切换模式 |
| 启动方式 | 手动运行可执行文件 | `ros2 launch dog_cpp bringup.launch.py` 一键启动三个节点 |

解耦带来的直接好处：**控制器节点完全不认识 MuJoCo**（在 `controller.cpp` 里搜不到
`simulator` / `mjData` / `qpos`），它只认一件事——"收到状态、发出 MIT 参数"。
以后要上实机，只需把仿真节点换成实机节点，控制器一行都不用改。

## 二、目录结构

```
第3次作业--ros2系统下的仿真改良/
└── ros2_ws/                      # ROS2 工作空间
    └── src/
        ├── my_interfaces/        # 接口包（自定义消息与服务）
        │   ├── msg/JointCommand.msg
        │   ├── msg/JointState.msg
        │   └── srv/SetMode.srv
        ├── dog_cpp/              # C++ 节点包（仿真 + 控制器）
        │   ├── src/
        │   │   ├── simulator.hpp / .cpp        # 复用第1次作业，封装 MuJoCo
        │   │   ├── controller.hpp / .cpp       # 由第1次作业改造：不再依赖 MuJoCo
        │   │   ├── sim_node.cpp                # 仿真运行节点（含 SDL/OpenGL 渲染）
        │   │   └── controller_node.cpp         # 控制器节点（含模式切换服务）
        │   ├── scenes/, models/  # 场景与网格，随包安装到 share/dog_cpp
        │   └── launch/bringup.launch.py
        └── dog_teleop/           # Python 节点包（手柄遥控）
            └── dog_teleop/teleop_node.py
```

## 三、节点与通信接口

### 3.1 节点

| 节点 | 语言 | 职责 |
| --- | --- | --- |
| `sim_node` | C++ | 订阅 `/joint_command`，计算力矩并 `mj_step`，发布 `/joint_states` 与 `/imu`；同时弹出 SDL/OpenGL 渲染窗口 |
| `controller_node` | C++ | 订阅 `/joint_states`，运行 `RobotController` 计算目标轨迹与增益，发布 `/joint_command`；提供 `/set_mode` 服务 |
| `teleop_node` | Python | 用 pygame 读取 Xbox 协议手柄按键，通过 `/set_mode` 服务切换机器人模式 |

### 3.2 话题与服务

| 名称 | 类型 | 方向 | 内容 |
| --- | --- | --- | --- |
| `/joint_command` | `my_interfaces/msg/JointCommand` | 控制器 → 仿真 | 12 个电机的 5 个 MIT 参数 `{kp,kd,q,w,tau}` |
| `/joint_states` | `my_interfaces/msg/JointState` | 仿真 → 控制器 | 12 个电机的 `{q,dq,ddq,tau,cur}` |
| `/imu` | `sensor_msgs/msg/Imu` | 仿真 → （任意订阅者） | 基座姿态（四元数）、角速度、线加速度 |
| `/set_mode` | `my_interfaces/srv/SetMode` | 手柄 → 控制器 | 请求：目标模式；响应：是否成功 + 说明文字 |

**为什么模式切换用服务而不是话题？** 因为它是**一次性的请求**，而且**需要确认对方收到了**
（服务有 response，话题没有）。用话题的话，丢一帧就"这一下白按了"，而且发的人无从知晓。
这与课堂上讲的"话题像广播、服务像打电话"一致。

## 四、自定义消息与服务

`my_interfaces/msg/JointCommand.msg`
```
std_msgs/Header header
float64[12] q      # 期望关节位置
float64[12] w      # 期望关节速度（前馈）
float64[12] tau    # 前馈力矩
float64[12] kp     # 位置增益
float64[12] kd     # 阻尼增益
```

`my_interfaces/msg/JointState.msg`
```
std_msgs/Header header
float64[12] q      # 关节位置
float64[12] dq     # 关节速度
float64[12] ddq    # 关节加速度
float64[12] cur    # 电流（由力矩等效估算）
float64[12] tau    # 实际施加的力矩
```

`my_interfaces/srv/SetMode.srv`
```
int32 mode      # 0=DAMPING 1=STAND 2=LIE 3=MARCH 4=WALK
---
bool success
string message
```

## 五、如何运行

### 5.1 依赖

- ROS2 Humble
- MuJoCo 的 Python 包（`pip install mujoco`）：C++ 端链接的就是它自带的 `libmujoco.so`
- `libsdl2-dev`：渲染窗口
- `pygame`：手柄节点（`pip install pygame`）

### 5.2 编译

把本目录的 `ros2_ws` 作为工作空间（或把 `ros2_ws/src` 下的三个包拷进你自己的 `src`）：

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 5.3 一键启动

```bash
ros2 launch dog_cpp bringup.launch.py
```

会一并启动 `sim_node`、`controller_node`、`teleop_node` 三个节点，并弹出 MuJoCo 渲染窗口。
按 `Ctrl+C` 三个节点会一起退出。

**场景路径无需手动修改**：`scenes/` 与 `models/` 会随 `dog_cpp` 一起安装到
`install/dog_cpp/share/dog_cpp/`，launch 文件用 `get_package_share_directory()` 定位，
仓库 clone 到任何路径都能直接跑。

### 5.4 操作性验证

```bash
# 另开一个终端（先 source）
ros2 node list          # 应看到 sim_node / controller_node / teleop_node
ros2 topic list -t      # 应看到 /joint_command /joint_states /imu 及类型
ros2 topic hz /joint_states     # 约 500 Hz（= 1 / 模型 timestep 0.002s）

# 没有手柄时，可以直接用服务命令代替手柄
ros2 service call /set_mode my_interfaces/srv/SetMode "{mode: 1}"   # 站立
ros2 service call /set_mode my_interfaces/srv/SetMode "{mode: 2}"   # 趴下
ros2 service call /set_mode my_interfaces/srv/SetMode "{mode: 3}"   # 原地踏步
ros2 service call /set_mode my_interfaces/srv/SetMode "{mode: 4}"   # 行走
ros2 service call /set_mode my_interfaces/srv/SetMode "{mode: 99}"  # 非法值 → success: false
```

### 5.5 手柄按键

手柄节点使用 pygame 读取设备，按键编号在 `teleop_node.py` 的 `BUTTON_MAP` 中配置
（不同手柄驱动编号不同，可用探针脚本实测后修改）：

| 手柄按键 | 模式 |
| --- | --- |
| X | DAMPING（阻尼） |
| A | STAND（站立） |
| B | LIE（趴下） |
| Y | MARCH（原地踏步） |
| LB | WALK（行走） |

## 六、几个关键设计说明

1. **PD 力矩在仿真侧计算。** 控制器只下发 MIT 参数 `{kp,kd,q,w,tau}`，由仿真节点按
   `tau = kp*(q_des - q) + kd*(w_des - dq) + tau_ff` 计算力矩。原来写在控制器里的那段
   力矩计算被搬到了仿真侧——这正是"MIT 接口"的形态，也是控制器得以脱离 MuJoCo 的原因。
   （阻尼模式等价于 `kp=0, kd=kd_, w=0, tau=0`。）

2. **控制器不再依赖 MuJoCo。** 改造后 `RobotController::update()` 的签名是
   `update(const std::array<double,12>& q_cur, double dt)`，只接收"当前关节位置"和"时间步"，
   返回 MIT 参数结构体。所以 `controller_node` 编译时**不需要链接 MuJoCo**。

3. **渲染与物理分线程。** 物理以 500 Hz 在 ROS 执行器线程上跑，渲染在主线程以约 60 Hz 跑。
   两者通过互斥锁保护 `mjData`：`mjv_updateScene`（读 `mjData`）在锁内，
   `mjr_render`（只读已生成的 scene）在锁外。这样渲染再慢也不会拖慢物理——
   实测 `/joint_states` 仍稳定在约 500 Hz。
   （OpenGL 上下文有线程亲和性，因此"创建窗口"和"渲染"必须在同一个线程。）

4. **IMU 数据的来源与注意点。** 模型里没有定义 MuJoCo 传感器，IMU 由基座自由关节推出：
   姿态取 `qpos[3..6]`（**注意 MuJoCo 的四元数顺序是 `w,x,y,z`，而 ROS `Imu.orientation` 是
   `x,y,z,w`，必须重排**）、角速度取 `qvel[3..5]`、线加速度取 `qacc[0..2]`。
   其中 `linear_acceleration` 填的是 MuJoCo 的净加速度，机器人静止站立时接近 0；
   真实加速度计此时应读到约 +9.81 m/s²（重力反力），这里做了工程简化。

5. **服务端做了输入校验。** `SetMode` 服务对 `mode` 做越界检查，非法值返回
   `success=false` 与说明文字，而不是让数组越界导致节点崩溃。

## 七、过程中用到的 ROS2 命令

```bash
ros2 pkg create ...              # 建包
ros2 interface show <类型>        # 查看消息/服务字段（写自定义消息时反复用）
ros2 topic list -t               # 列出话题与消息类型
ros2 topic echo /joint_states    # 观察话题内容
ros2 topic hz /joint_states      # 观察发布频率（验证实时性）
ros2 topic pub --once ...        # 手动发一条消息（没有控制器节点时先测仿真节点）
ros2 service list -t             # 列出服务与类型
ros2 service call <服务> <类型> '{...}'   # 手动调用服务（没有手柄时先测服务端）
ros2 node list / ros2 node info  # 确认节点是否被发现、接口是否正确
ros2 launch <包> <launch文件>      # 一键启动
```

**排查经验**：`echo` 收不到数据时，先 `ros2 node list` 确认节点在跑、再 `ros2 topic list -t`
确认话题名和类型，而不是先怀疑代码。
