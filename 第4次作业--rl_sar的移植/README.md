# 第 4 次作业：rl_sar 的移植

本作业将开源强化学习部署框架 **rl_sar** 移植到本人此前编写的 ROS 2 + MuJoCo 仿真节点，使训练好的 `best.pt` 策略能够根据仿真中的关节状态、IMU 和速度指令输出 12 个关节目标，让 Black 四足机器狗完成站立、稳定行走和导航控制。

## 1. 作业目标与基本原理

强化学习策略可以看成一个函数：

```text
a_t = f(o_t)
```

- `o_t`：当前时刻的观测量，由 3 维速度命令、3 维机体角速度、3 维重力方向、12 维关节位置、12 维关节速度和 12 维上一帧动作组成，单帧共 45 维。
- `f`：训练好的神经网络，本作业中对应 `policy/black/legged_gym/best.pt`。
- `a_t`：12 个关节的增量动作。动作先缩放，再叠加默认关节位置，最后转换为关节目标位置和 PD 控制量。
- 模型实际输入为连续 6 帧观测，因此网络输入维度为 `6 × 45 = 270`。

完整闭环为：

```text
MuJoCo 仿真
   │
   ├── /joint_states  (my_interfaces/msg/JointState)
   └── /imu          (sensor_msgs/msg/Imu)
             │
             ▼
      rl_sim / RobotControl
             │
             ▼
        状态映射 GetState
             │
             ▼
     RunModel（50 Hz） → Forward()
             │
             ├── ComputeObservation：缩放、拼接当前 45 维观测
             ├── ObservationBuffer：缓存并展开 6 帧历史
             └── best.pt：神经网络前向推理
             │
             ▼
      ComputeOutput：动作缩放、默认姿态、PD、力矩限幅
             │
             ▼
       FSM / RL_Locomotion
             │
             ▼
      /joint_command
             │
             ▼
       MuJoCo 仿真执行
```

## 2. 目录结构

```text
第4次作业--rl_sar的移植/
├── README.md
├── .gitignore
├── 大作业要求.md
├── ros2_ws/
│   └── src/
│       ├── dog_cpp/             # 自写 MuJoCo 仿真、场景、模型和旧控制器
│       ├── my_interfaces/       # JointState、JointCommand 等自定义消息
│       └── dog_teleop/          # 键盘速度控制节点
└── rl_sar_ws/
    ├── build.sh                 # rl_sar 原构建脚本
    └── src/
        ├── rl_sar/              # 已移植的 RL 控制框架
        ├── robot_msgs/          # rl_sar 原有消息依赖
        └── robot_joint_controller/  # rl_sar 原有控制器依赖
```

运行所需的源码、配置、模型和第三方依赖均已放入对应工作空间。`build/`、`install/`、`log/`、`.git/` 和 Python 缓存不提交，避免提交编译产物。

## 3. rl_sar 的四层结构

| 层次 | 核心文件 | 作用 |
| --- | --- | --- |
| ROS 接口层 | `rl_sar/src/rl_sim.cpp`、`rl_sar/include/rl_sim.hpp` | 创建 ROS 2 节点，订阅 `/joint_states`、`/imu`、`/cmd_vel`，发布 `/joint_command`，并启动控制循环与推理循环。 |
| RL 核心层 | `library/core/rl_sdk/rl_sdk.cpp`、`rl_sdk.hpp` | 读取 YAML，初始化模型，构造观测，执行前向推理，将动作转换为关节位置、速度和力矩，并处理键盘输入。 |
| 观测缓存层 | `library/core/observation_buffer/observation_buffer.cpp`、`.hpp` | 保存连续 6 帧单帧观测，并按时间顺序展开成 270 维模型输入。 |
| FSM 状态层 | `library/core/fsm/fsm_core.hpp`、`policy/black/fsm.hpp` | 管理 Passive、GetUp、RL_Locomotion、GetDown 状态及状态切换。 |
| 配置与模型层 | `policy/black/base.yaml`、`policy/black/legged_gym/config.yaml`、`best.pt` | 配置关节映射、默认姿态、控制周期、观测缩放、动作缩放、KP/KD、力矩限制和模型文件名。 |

真正承担“控制器节点”职责的核心是：

1. `rl_sim.cpp`：ROS 接口和主循环。
2. `rl_sdk.cpp`：策略推理与控制量生成。
3. `policy/black/fsm.hpp`：运行状态与安全切换。
4. `policy/black/` 下的 YAML 和 `best.pt`：机器人专属参数和模型。

## 4. 一次控制周期的数据流

### 4.1 仿真状态进入控制器

`RobotStateCallback()` 接收 `my_interfaces::msg::JointState` 和 `sensor_msgs::msg::Imu`。`GetState()` 按 `joint_mapping` 将仿真关节顺序转换为策略训练时的顺序：

```text
[0, 1, 2, 3, 4, 5, 9, 10, 11, 6, 7, 8]
```

映射后的数据写入 `robot_state`，供控制循环读取。IMU 提供角速度和四元数，其中四元数用于计算机体重力方向。

### 4.2 控制策略循环

`RobotControl()` 以 `base.yaml` 中的 `dt = 0.005 s` 运行，约为 200 Hz：

1. 读取最新 `robot_state`。
2. 调用当前 FSM 状态的 `Run()`。
3. 如果处于 RL_Locomotion，则从输出队列取出最近一次模型结果。
4. 将关节目标写入 `robot_command`。
5. 调用 `SetCommand()`，按 `joint_mapping` 还原为仿真关节顺序。
6. 发布 `/joint_command`。

### 4.3 策略推理循环

`RunModel()` 以 `dt × decimation = 0.005 × 4 = 0.02 s` 运行，约为 50 Hz：

1. 从最新 `robot_state` 读取角速度、四元数、关节位置和关节速度。
2. 根据导航模式选择 `/cmd_vel` 或键盘/手柄速度指令。
3. 调用 `Forward()`。
4. `ComputeObservation()` 生成并缩放当前 45 维观测。
5. `ObservationBuffer` 插入当前帧并取出 6 帧历史。
6. `model.forward()` 调用 `best.pt`。
7. 对动作进行上下限裁剪。
8. `ComputeOutput()` 计算关节目标位置和力矩。
9. 将结果压入输出队列，由 200 Hz 控制循环消费。

因此，`RunModel()` 是外层调用者，`Forward()` 是模型推理函数；`Forward()` 内部最终执行 `model.forward()`。两者不是并列关系。

### 4.4 为什么使用 200 Hz 和 50 Hz 双频

控制循环需要快速发送关节指令并保持 FSM 响应，因此运行在 200 Hz；神经网络推理开销较大，策略训练时通常采用 50 Hz，因此每 4 个控制周期更新一次动作。

两个循环通过线程安全队列解耦：

- 200 Hz 循环持续消费最新动作并发布控制量。
- 50 Hz 循环持续生成新动作。
- 模型较慢时不会阻塞底层关节控制和状态响应。

如果把控制循环直接降到 50 Hz，FSM 响应、关节命令更新和底层 PD 支撑都会变慢，可能出现姿态下陷、动作抖动或稳定性下降；但策略推理本身仍可保持 50 Hz。

## 5. 状态机

`policy/black/fsm.hpp` 实现了以下状态：

| 状态 | 作用 | 是否使用 best.pt |
| --- | --- | --- |
| `RLFSMStatePassive` | 阻尼被动状态，KP=0，KD=8，清除力矩。 | 否 |
| `RLFSMStateGetUp` | 先插值到趴下准备姿态，再插值到默认站立姿态。 | 否 |
| `RLFSMStateRL_Locomotion` | 进入时加载模型，执行强化学习行走和导航。 | 是 |
| `RLFSMStateGetDown` | 从当前姿态插值回趴下准备姿态，再回到默认姿态。 | 否 |

状态切换：

```text
Passive --按 0--> GetUp --起身完成且按 1--> RL_Locomotion
   ▲                                              │
   │                                              ├--按 9--> GetDown --> Passive
   └──────────────────按 P────────────────────────┘
```

只有 `RL_Locomotion` 调用 `InitRL()` 和模型推理。Passive、GetUp、GetDown 使用固定 KP/KD 和姿态插值，不运行 `best.pt`。

## 6. 相对原 rl_sar 的移植修改

### 6.1 ROS 2 消息与话题适配

原 rl_sar 仿真接口使用 `robot_msgs::msg::RobotState` 和 `robot_msgs::msg::RobotCommand`。本作业修改为：

| 方向 | 话题 | 类型 |
| --- | --- | --- |
| 订阅 | `/joint_states` | `my_interfaces/msg/JointState` |
| 订阅 | `/imu` | `sensor_msgs/msg/Imu` |
| 订阅 | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| 发布 | `/joint_command` | `my_interfaces/msg/JointCommand` |

同时修改了：

- `include/rl_sim.hpp` 的消息类型和回调声明。
- `src/rl_sim.cpp` 的发布器、订阅器和回调实现。
- `CMakeLists.txt` 中的 `my_interfaces` 依赖与链接。
- C++ 标准从 C++14 提升到 C++20，以匹配当前工具链和依赖。

### 6.2 绕过 controller_manager

原 rl_sar 通过 `StartJointController()` 启动 `controller_manager` 和 `robot_joint_controller`。本仿真节点已经直接提供 `/joint_states`，也直接接收 `/joint_command`，因此当前 ROS 2 仿真路径注释掉了该调用：

```cpp
// this->StartJointController(this->ros_namespace, this->params.joint_names);
```

这不是删除 ROS 2 控制能力，而是把控制器管理交给自写仿真节点，避免再启动一套重复的 controller manager。

### 6.3 新增 Black 策略与状态机

新增目录：

```text
rl_sar/src/rl_sar/policy/black/
├── base.yaml
├── fsm.hpp
└── legged_gym/
    ├── config.yaml
    └── best.pt
```

`fsm.hpp` 通过 `BlackFSMFactory` 注册 `black` 类型。运行参数 `robot_name:=black` 后，rl_sar 会自动创建 Black 状态机。

### 6.4 模型参数适配

`base.yaml` 的关键参数：

- `dt: 0.005`：控制周期 200 Hz。
- `decimation: 4`：模型周期 50 Hz。
- `num_of_dofs: 12`：12 个关节。缺少此参数曾导致 `InitRL() failed: bad conversion`。
- `joint_mapping: [0,1,2,3,4,5,9,10,11,6,7,8]`：修正 Black 腿部与训练环境的关节顺序。
- `default_dof_pos`：策略训练时使用的默认站立姿态。
- `fixed_kp/fixed_kd`：非 RL 状态下的固定增益。
- `torque_limits`：关节力矩限幅。

`legged_gym/config.yaml` 的关键参数：

- `model_name: best.pt`。
- `num_observations: 45`。
- `observations_history: [0,1,2,3,4,5]`：6 帧历史。
- `action_scale: 0.25`：增量动作缩放。
- `rl_kp: 40`、`rl_kd: 1.2`：RL 状态 PD 增益。
- `commands_scale: [2.0, 2.0, 0.25]`。
- `ang_vel_scale: 0.25`、`dof_vel_scale: 0.05`。

### 6.5 导航模式

导航模式默认关闭。只有按 `N` 切换为开启后，`RunModel()` 才使用 `/cmd_vel`：

```cpp
if (control.navigation_mode)
{
    obs.commands = cmd_vel;
}
else
{
    obs.commands = keyboard_or_gamepad_commands;
}
```

退出 `RL_Locomotion` 后，`/cmd_vel` 立即不再参与控制；重新进入 `RL_Locomotion` 时，导航模式开关保持退出前的状态。

## 7. 与旧控制器的对比

### 7.1 旧 `rl_controller_node`

旧节点已经能够直接调用 `best.pt` 让机器狗行走，但实现方式更接近“单文件原型”：

- 模型路径、关节映射、缩放系数、KP/KD 和默认姿态主要写在 C++ 中。
- 只使用一个约 50 Hz 的 `policy_timer_`，同时完成观测拼接、模型推理和关节命令发布。
- 历史帧通过手工数组搬移维护，270 维输入在节点内部拼接。
- 没有 Passive、GetUp、GetDown 等完整状态机。
- 没有独立的底层控制循环，模型推理和关节发布耦合在同一回调中。
- 导航指令处理与模型逻辑耦合，缺少明确的导航开关。

### 7.2 旧 `controller_node`

旧 `controller_node` 是传统控制节点：

- 使用手写 `RobotController::update()` 生成关节指令。
- 通过 `SetMode` 服务在 DAMPING、STAND、LIE、MARCH、WALK 等模式间切换。
- 使用固定 500 Hz 定时器发布控制量。
- 不依赖神经网络，不需要观测缓存和模型推理。

它与 `rl_sar` 的共同点是都遵循“订阅状态 → 生成指令 → 发布命令”的 ROS 控制节点基本模型；不同点是 `rl_sar` 把策略、状态机、配置和 ROS 接口拆开，并增加模型推理专用循环。

### 7.3 对比总结

| 维度 | 旧 `rl_controller_node` / `controller_node` | 移植后的 rl_sar |
| --- | --- | --- |
| 代码组织 | 单节点集中处理 | ROS 接口、RL SDK、观测缓存、FSM、配置分层 |
| 模式管理 | 手工模式或没有完整 FSM | Passive / GetUp / Locomotion / GetDown 状态机 |
| 运行频率 | 模型和命令发布基本同频 | 200 Hz 控制循环 + 50 Hz 推理循环 |
| 参数来源 | C++ 硬编码为主 | YAML 配置驱动 |
| 模型调用 | 构造节点时直接加载或单回调调用 | 进入 RL_Locomotion 时延迟加载，`RunModel → Forward → model.forward` |
| 历史观测 | 手写数组移位 | `ObservationBuffer` 统一管理 |
| 动作后处理 | 节点内部手写 | `ComputeOutput()` 统一缩放、叠加默认姿态、PD 和限幅 |
| 状态安全 | 逻辑相对简单 | 姿态插值、固定增益、力矩限幅、状态门控 |
| 扩展性 | 增加机器人需要改多个地方 | 新增 policy 目录和 FSM 工厂即可扩展 |

### 7.4 可以借鉴的工程思路

1. 把接口与算法分开：ROS 回调只负责收发和状态映射，模型计算放在 RL SDK。
2. 把机器人差异放进配置：关节顺序、默认姿态、缩放和增益不应散落在 C++ 中。
3. 把运行阶段显式化：Passive、GetUp、Locomotion、GetDown 分开，避免一上来就执行策略。
4. 控制与推理解耦：慢速模型循环和快速控制循环通过队列连接。
5. 统一数据快照：使用互斥锁保护最新状态，避免回调与定时器同时读写。
6. 延迟初始化：进入 Locomotion 才加载模型，失败时回到 Passive。
7. 先验证接口再调策略：先确认话题类型、QoS、频率和关节顺序，再处理模型输入维度。
8. 区分物理频率与渲染帧率：500 Hz 是物理步进/话题频率，不等于 MuJoCo 窗口 FPS。

## 8. 编译

### 8.1 环境

本作业在以下环境完成验证：

- Ubuntu 22.04
- ROS 2 Humble
- C++20
- Python 3.10 + MuJoCo
- PyTorch C++ / LibTorch
- `yaml-cpp`、TBB、SDL2、OpenGL
- `colcon`

如果 CMake 找不到 PyTorch，可先执行：

```bash
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$(python3 -c 'import torch; print(torch.utils.cmake_prefix_path)')"
```

### 8.2 编译仿真工作空间

```bash
cd 第4次作业--rl_sar的移植/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 8.3 编译 rl_sar 工作空间

必须先把 `ros2_ws` 加入环境，因为 rl_sar 需要找到 `my_interfaces`：

```bash
cd 第4次作业--rl_sar的移植/rl_sar_ws
source /opt/ros/humble/setup.bash
source ../ros2_ws/install/setup.bash
colcon build --symlink-install \
  --packages-select robot_msgs robot_joint_controller rl_sar \
  --cmake-args -DUSE_CMAKE=OFF
source install/setup.bash
```

如果使用原始构建脚本，也可以执行：

```bash
cd 第4次作业--rl_sar的移植/rl_sar_ws
source /opt/ros/humble/setup.bash
source ../ros2_ws/install/setup.bash
bash build.sh robot_msgs robot_joint_controller rl_sar
```

注意：代码通过编译期宏 `CMAKE_CURRENT_SOURCE_DIR` 定位 `policy/black/` 下的 YAML 和 `best.pt`。如果移动了源码目录，应重新编译。

## 9. 运行

以下三个终端都先执行：

```bash
source /opt/ros/humble/setup.bash
source <本作业目录>/ros2_ws/install/setup.bash
```

### 终端 1：启动参数服务

rl_sar 通过 `param_node` 读取机器人和 Gazebo 模型名称：

```bash
ros2 run demo_nodes_cpp parameter_blackboard \
  --ros-args -r __node:=param_node \
  -p robot_name:=black -p gazebo_model_name:=black
```

### 终端 2：启动 MuJoCo 仿真

```bash
ros2 run dog_cpp sim \
  "$(ros2 pkg prefix dog_cpp)/share/dog_cpp/scenes/flat_scene.xml"
```

### 终端 3：启动 rl_sar 控制器

```bash
source <本作业目录>/rl_sar_ws/install/setup.bash
cd <本作业目录>/rl_sar_ws
./build/rl_sar/rl_sim
```

如果构建后已经生成 ROS 2 可执行安装项，也可以使用：

```bash
ros2 run rl_sar rl_sim
```

### 终端 4：可选，键盘速度控制

在 rl_sar 终端按 `N` 开启导航模式，然后在另一个终端发送速度命令：

```bash
source <本作业目录>/ros2_ws/install/setup.bash
ros2 run dog_teleop rl_keyboard
```

键盘控制节点使用：

- `W/S`：前进/后退。
- `A/D`：左移/右移。
- `Q/E`：左转/右转。
- 空格：速度清零。
- `+/-`：调整速度倍率。
- `R`：重置。
- `H`：帮助。

## 10. 操作顺序

在 rl_sar 终端中：

```text
0 → 1 → N → 使用 /cmd_vel 行走
```

详细说明：

1. 启动后默认为 Passive。
2. 按 `0` 进入 GetUp，机器人先到趴下准备姿态，再平滑起身到默认站立姿态。
3. 起身达到 100% 后按 `1` 进入 RL_Locomotion，加载 `best.pt`。
4. 按 `N` 开启导航模式，之后 `/cmd_vel` 才生效。
5. 按 `9` 退出 RL_Locomotion，进入 GetDown，随后自动回到 Passive。
6. 按 `P` 可紧急切回 Passive。
7. 重新执行 `0 → 1` 可再次进入行走；导航模式开关保持上次状态。

## 11. 验证命令

检查话题类型：

```bash
ros2 topic list -t
ros2 topic info /joint_states -v
ros2 topic info /imu -v
ros2 topic info /joint_command -v
```

检查频率：

```bash
ros2 topic hz /joint_states
ros2 topic hz /imu
ros2 topic hz /joint_command
```

本机验证时 `/joint_states` 和 `/imu` 可稳定在约 500 Hz。MuJoCo 窗口的显示帧率可能只有 60～100 FPS，这是物理更新频率与图形渲染频率不同造成的，不代表物理循环异常。

检查命令内容：

```bash
ros2 topic echo /joint_command --once
```

预期结果：

- `/joint_states` 有持续发布者，rl_sar 启动后也有订阅者。
- `/joint_command` 在 Passive、GetUp、RL_Locomotion、GetDown 下均有输出。
- 进入 RL_Locomotion 后终端出现模型加载成功信息。
- 按 `N` 后终端显示 `Navigation mode: ON`。
- 发送 `/cmd_vel` 后机器人能够平稳向前运动。

## 12. 已解决的问题与排障

### 12.1 `/joint_states` 显示没有订阅者

原因是 rl_sar 仍按原 `robot_msgs::msg::RobotState` 订阅，消息类型与仿真节点的 `my_interfaces/msg/JointState` 不一致。统一消息类型和话题名后解决。

排查顺序：

1. 用 `ros2 topic info /joint_states -v` 确认实际类型。
2. 确认 C++ 回调模板参数与话题类型完全一致。
3. 确认发布端与订阅端 QoS 兼容。
4. 重新编译并重新启动节点。

### 12.2 `InitRL() failed: bad conversion`

原因是 `policy/black/base.yaml` 缺少 `num_of_dofs`，导致 YAML 读取后维度转换失败。在配置中显式加入 `num_of_dofs: 12`，并确认关节数组长度均为 12。

### 12.3 机器人抖动或不动

重点检查：

- `joint_mapping` 是否与训练环境的关节顺序一致。
- `default_dof_pos` 是否与训练姿态一致。
- `action_scale`、`rl_kp`、`rl_kd`、`torque_limits` 是否正确。
- 模型输入是否确实为 6 帧 × 45 维。
- 观测缩放是否与训练配置一致。
- `GetUp` 是否已经达到 100%，再进入 RL_Locomotion。
- 是否存在消息类型正确但关节顺序错误的问题。

本作业最终已验证：起身动作正常，按 `1` 后进入策略控制，方向正确，步态平稳。

### 12.4 仿真窗口卡顿

MuJoCo 的物理步进可以稳定在 500 Hz，但窗口显示受 CPU/GPU、桌面合成器和其他 ROS 节点影响。排查时先关闭其他节点，只启动仿真观察；不要把 `ros2 topic hz` 的测量值直接当成窗口 FPS。

### 12.5 状态机运算符优先级

起身完成条件必须加括号：

```cpp
if (rl.running_percent == 1.0f &&
    (rl.control.current_keyboard == Input::Keyboard::Num1 ||
     rl.control.current_gamepad == Input::Gamepad::RB_DPadUp))
```

否则 `&&` 与 `||` 的优先级会让状态提前切换。

### 12.6 退出 RL_Locomotion 后 `/cmd_vel` 的控制边界

这是 FSM 与导航模式耦合问题。当前设计保证：

- 离开 `RL_Locomotion` 后，模型输出不再进入控制链，`/cmd_vel` 立即失效。
- `N` 只切换导航模式，不直接切换 FSM 状态。
- 重新进入 Locomotion 时保留导航开关状态。

## 13. 可继续改进的方向

1. 把策略侧 ROS 话题名、QoS 和机器人名称全部参数化。
2. 增加模型输入/输出维度启动自检，在 InitRL 阶段给出更明确的错误。
3. 给模型推理增加独立线程和超时监控，避免模型卡住时阻塞控制。
4. 增加关节状态时间戳检查，防止使用过期观测。
5. 将仿真节点和策略节点拆成可复用 launch 文件。
6. 增加离线回放和 CSV 记录，对比动作、关节位置、力矩和 IMU。
7. 将 rl_sar 的 FSM、配置和观测缓存思想反哺旧 `rl_controller_node`，逐步替换硬编码实现。

## 14. 提交说明

本目录提交内容包括：

- 自写 MuJoCo 仿真包 `dog_cpp`。
- 自定义消息包 `my_interfaces`。
- 键盘控制包 `dog_teleop`。
- 已移植的 `rl_sar` 源码及 Black 策略目录。
- `robot_msgs`、`robot_joint_controller` 依赖。
- `best.pt` 模型权重。
- 原 rl_sar 构建脚本和作业要求文档。

不提交编译产物、日志、缓存和编辑器临时文件。

## 15. 参考资料与说明

- rl_sar 原始工程采用 Apache-2.0 许可，相关版权和许可声明保留在源码中。
- 本作业重点是推理部署和 ROS 2 接口移植，不包含强化学习训练过程。
- 模型权重 `best.pt` 由作业提供，仅用于本课程实验。
