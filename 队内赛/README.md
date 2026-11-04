# Black 地形状态机：限高杆、木桥与高墙仿真

本目录把本地 mjlab 中 Black 的基础行走、爬升和跳跃策略组织为一个可交互状态机，并在用户提供的地形 `assets/scene_terrain.xml` 上提供 GUI/无界面仿真。固定动作包括站立、趴下、低姿态爬行、恢复、木桥 B 迈步动作和高墙腾越；其中 `CRAWL_FORWARD` 可从 0.310 m 高的限高杆下方穿过，`WALL_CROSS` 已实测可越过 0.30 m 高墙并在墙北侧恢复站立。当前新的 45 维基础策略在木桥回归中于桥中段发生横滚超限，`BRIDGE_B_CROSS` 尚需继续调参，详见第 5.2 节。

## 1. 与第四次作业的关系

上游第四次作业位于 `第4次作业--rl_sar的移植`，其 rl_sar 部署链路采用单帧 45 维观测和 6 帧历史缓存。本目录当前的基础策略已替换为：

```text
assets/policies/student_model_299.pt
```

该文件是 RSL-RL distillation checkpoint，不是 TorchScript。checkpoint 的 `actor_state_dict` 包含观测归一化参数、`128 -> 256 -> 128 -> 12` MLP 和单层 GRU；GRU 隐藏维度为 128。加载器按 `strict=True` 严格加载全部 actor 参数，并以确定性输出（而非采样）执行 12 维动作。

当前父环境 actor 观测的前 45 维顺序为：

```text
ang_vel(3) + gravity_vec(3) + joint_pos_rel(12) + joint_vel(12)
+ previous_action(12) + commands(3) = 45
```

`src/black_terrain_fsm/policies.py` 中的 `FlatRSLRLPolicy` 完成以下部署适配：

- 从 122 维地形 actor 观测中只取前 45 维，刻意排除后 77 维 height scan；
- 使用单帧输入，不构建 6 帧或 270 维输入；
- 在 GRU 内部跨控制步保留隐藏状态，并在策略切换与环境 reset 时清零；
- 使用 checkpoint 内的观测归一化参数和 mjlab 原生关节/动作顺序，不再执行 HIM 导出的关节重排和默认姿态偏移。

`CLIMB`、`JUMP` 仍使用原有的 122 维 GRU RSL-RL checkpoint；`LOCOMOTION`、自动流程中的基础行走和木桥 B 动作使用上述 45 维 flat student。旧的 `HimTorchScriptPolicy` 加载器仍保留用于兼容旧配置，但当前 `configs/policies.json` 不再使用它。动作裁剪仍固定为 `clip_actions=None`。

## 2. 状态机

| 状态 | 动作来源 | 说明 |
|---|---|---|
| `HOLD` | 关节目标保持 | 保持切换瞬间的实测关节位置 |
| `STAND` | 固定姿态插值 | 站立并保持 |
| `LOCOMOTION` | 45 维 GRU student | 使用单帧 45 维观测和 GRU 内部隐藏状态行走；前进、后退、横移和转向命令均直接发送给该策略，并支持多轴同时叠加 |
| `CLIMB` | 爬升策略 | 使用 climb checkpoint |
| `JUMP` | 跳跃策略 | 自主垂直跳跃，不是横杆或沟壑穿越动作 |
| `LIE_DOWN` | 固定姿态插值 | 趴下到低姿态 |
| `CRAWL_FORWARD` | 爬行控制器 | 降姿后以四拍步态低姿态前进 |
| `BRIDGE_B_CROSS` | 基础策略单动作 | 从木桥 B 北端前脚贴边起步，设计目标是连续迈步到南端平台；当前新基础策略回归失败，见 5.2 节 |
| `WALL_CROSS` | 短时辅助全身腾越 | 起跳、收腿、前向腾越、制动落地并恢复站立 |
| `RECOVER` | 固定姿态插值 | 从低姿态恢复到站立 |
| `AUTOPILOT` | 组合控制 | `stand -> align -> lower -> crawl -> recover -> stand` |
| `EMERGENCY_STOP` | 关节目标保持 | 仅软件目标保持，不是硬件急停 |

`AUTOPILOT` 从横杆南侧的指定 staging 点开始，不在本版本中负责从红色出生点长距离自动导航。红色出生点自动接近横杆仍受窄走廊、基础策略最小转弯半径和碎木障碍影响，尚未可靠解决；该段请使用 `manual` 场景人工驾驶。

`BRIDGE_B_CROSS` 只针对本地图木桥 B 和指定北端起点，不是通用视觉过桥算法。旧版“跨宽板、降姿、分步转身、跨窄板”的多阶段木桥路线已经删除；当前动作只有一个连续阶段 `walk`：

- 机器人从北端平台 `(5.70, -1.32, 0.72)`、yaw `-90 deg` 起步，前进方向为 `-y`；
- 起步时检查两只前脚的 `y` 坐标，要求它们位于桥缘 `y=-1.50` 的 `±0.04 m` 范围内；
- 只向当前 45 维基础策略发送固定前进命令 `vx=0.70 m/s`，并根据桥轴航向误差发送受限的 `wz` 修正，最大 `0.12 rad/s`；
- 全程监测横向漂移、航向、根高度、pitch、roll 和 20 s 超时；
- 根节点到达 `y<=-3.55 m` 后停止命令，切回 `STAND`，并记录 `walk -> complete` 和诊断量。

这是一种“贴边起步后连续迈步”的动作，不包含平台转身，也不切换专用木桥策略。该动作在前一版 HIM 基础策略上曾经通过；替换为当前 flat checkpoint 后，机器人在桥中段横滚超限，尚未恢复通过能力。

`WALL_CROSS` 只针对本图中的 0.30 m 垂直高墙和墙南侧指定起点，子阶段如下：

| 子阶段 | 行为 |
|---|---|
| `settle` | 从放置姿态平滑插值到 `STAND_JOINTS` |
| `launch` | 保持腿部伸展，施加短时向上和前向冲量 |
| `tuck` | 离地后平滑收腿，避免腿部或支撑体扫到墙面 |
| `vault` | 保持收腿姿态并施加短时前向助推，俯仰/横滚角闭环 |
| `deploy` | 越墙后优先伸腿到落地站姿，避免前足或躯干先着地 |
| `land` | 水平与垂直速度缓冲、双轴姿态稳定；至少两足接地且连续稳定后才结束 |
| `done` | 姿态、速度、高度、足端接触和墙北侧落点均满足后切回 `STAND` |

这不是基础/爬升/跳跃策略直接攀爬垂直墙。基础、爬升和跳跃 checkpoint 在该墙前单独使用都无法越过；本技能采用了作用在躯干上的短暂外部冲量，因此应明确标记为“辅助全身腾越”，不能当作真实机器人上可直接执行的纯本体控制结果。

## 3. Checkpoint 配置

三个策略在 `configs/policies.json` 中配置：

```text
base           assets/policies/student_model_299.pt
               kind=rsl_rl_flat45
climb          assets/policies/climb_model_59.pt
               runner_cfg=black_gru_local_teacher_platform_recovery_v9_ppo_runner_cfg
jump           assets/policies/jump_model_299.pt
               runner_cfg=black_gru_trench_autonomous_vertical_jump_local_teacher_ppo_runner_cfg
```

三个 checkpoint 均已随本目录提供，`configs/policies.json` 使用相对于 `队内赛/` 包根的路径，不依赖某台机器的训练日志目录。运行环境仍需安装或引用与这三个模型匹配的本地 mjlab/rsl_rl，以便重建对应的 `RNNModel` 和 runner 配置。程序不会修改 checkpoint、mjlab 源码或训练日志。

## 4. 地形、限高杆、木桥与高墙

仿真使用 `assets/scene_terrain.xml` 和其引用的两张高度图。

### 4.1 限高杆

- 横杆中心：`(5.70, -9.00, 0.325)` m
- 横杆半径：`0.015` m
- 横杆下缘：`0.310` m
- 两根立柱中心：`x=5.20` 和 `x=6.20`，`y=-9.00`
- 爬行 staging 点：`(5.70, -10.22, 0.51)`，朝向 `+y`（90 deg）

爬行策略先用平滑插值进入折叠姿态，再执行 FL -> RR -> FR -> RL 的四拍步态。每次只有一条腿摆动，另外三条腿支撑。碰撞包络在横杆区域逐控制步测量，只有根节点跨过横杆平面、全程未接触横杆/立柱且包络低于下缘时才记为安全穿越。

### 4.2 木桥

实际地图关键尺寸：

```text
木桥 B 北端固定起点： (5.70, -1.32, 0.72)，yaw = -90 deg
北端平台：            x=5.20..6.20, y=-1.50..0.50
桥缘目标：            y=-1.50（两只前脚贴边）
三块窄板 x 范围：     5.25..5.45, 5.55..5.75, 5.85..6.05
三块窄板 y 范围：     -3.00..-1.50
南端平台：            x=5.20..6.20, y=-4.00..-3.00
桥面顶高：            约 0.20 m
```

动作路线是从北端平台沿 `-y` 方向连续走过木桥 B 的窄板段，到达南端平台后停止并恢复站立。起点根据实测足端位置标定，使 FL/FR 前脚刚好落在北端桥缘；运行中由基础策略连续迈步，由航向反馈抑制横向漂移。

### 4.3 高墙

实际地图关键尺寸：

```text
墙体中心：       (1.80, -7.00, 0.15) m
墙体 x 范围：    1.30..2.30 m
墙体 y 厚度：    -7.025..-6.975 m
墙高：           0.30 m
固定起点：       (1.80, -7.55, 0.51) m
起点朝向：       +y（90 deg）
成功落点区域：   x=1.30..2.30, y>-6.50, z>0.35 m
```

`WALL_CROSS` 同时监测六块 `black_support_collision_00..05` 与墙面的接触，以及其它 Black 碰撞体与墙面的接触。任一支撑碰撞体碰墙即判失败；其它腿部碰撞体会写入遥测，便于判断腾越包络是否干净。

## 5. 已实测结果

以下结果由本目录程序在当前机器上实际运行得到，不是手工构造值。

### 5.1 限高杆回归

- 场景：`bar`
- 状态序列：`CRAWL_FORWARD -> RECOVER -> STAND`
- 仿真时长：52.66 s，最终状态 `STAND`
- `bar_clean_crossed=true`、`bar_contact=false`
- 最高碰撞包络约 `0.237765 m`，相对 0.310 m 下缘的实测净空约 `0.072235 m`

### 5.2 木桥 B 当前回归：失败

- 场景：`bridge`，从北端 `(5.70, -1.32, 0.72)`、yaw `-90 deg` 自动开始
- 状态：启动后进入 `BRIDGE_B_CROSS`，但未完成，最终仍停留在该状态
- 策略：全程只使用新的 45 维 `rsl_rl_flat45` 基础策略
- 动作阶段：`walk -> failed`，失败原因 `roll_limit_exceeded`
- 失败位置：约 `(5.496, -2.195)` m，尚未到达南端平台
- 桥面诊断：初始前脚贴边检查通过，最大横向漂移约 `0.211 m`，最大航向误差约 `2.77 deg`
- 失败时最大 roll 约 `0.3869 rad`（`22.17 deg`），超过木桥动作的 `22 deg` 安全阈值；最终 JSON `success=false`
- 证据：`results/bridge_after_base_flat45.json` / `results/bridge_after_base_flat45.csv`

这一结果说明 checkpoint 已成功加载并驱动机器人，但新 flat 基础策略与木桥 B 的窄板连续迈步不完全兼容，不能声称当前版本已经过桥。

上一版 HIM 基础策略的木桥历史成功记录未随本次上传包含，不能作为当前 checkpoint 的通过依据。数据均为 MuJoCo/mjlab 仿真，不是真实机器人验证。

### 5.3 高墙完整腾越

- 场景：`wall`，从 `(1.80, -7.55, 0.51)`、yaw 90 deg 自动开始
- 状态序列：`WALL_CROSS -> STAND`
- 仿真时长：3.72 s（186 个控制步），最终状态 `STAND`
- 最终位置约 `(1.80, -6.34, 0.49)` m，越过墙北侧并与墙保持足够距离
- `wall_cleared=true`、`wall_support_contact=false`、`wall_leg_contact=false`
- 全程无 NaN，无墙面支撑体或腿部碰撞，最终 JSON `success=true`
- 子阶段实际转换时间：`settle 0.00 s -> launch 1.40 s -> tuck 1.52 s -> vault 1.72 s -> deploy 2.00 s -> land 2.20 s -> done 2.68 s`
- 落地改进：恢复阶段先伸腿再接地，并加入水平制动、垂直缓冲和俯仰/横滚双轴闭环。落地探针中最低根高由约 `0.26 m` 提高到 `0.455 m`，最大俯仰角由约 `21.7 deg` 降到 `4.2 deg`，首次接地后不再出现约 `0.30 m` 的二次弹起。

正式结果文件：

- `results/wall_cross_soft_landing.json` / `results/wall_cross_soft_landing.csv`：加入落地缓冲与足端接触判据后的当前成功回归。
- `results/wall_after_base_flat45.json` / `results/wall_after_base_flat45.csv`：替换当前基础策略后再次验证高墙动作的结果。
- 上一版高墙和木桥历史成功结果未随本次上传包含，不能作为当前 checkpoint 的通过依据。

## 6. 运行方式

默认使用当前机器的 mjlab Python：

```bash
/home/wangqikai/mjlab/.venv/bin/python
```

如路径不同，可设置 `MJLAB_PYTHON`。

从木桥 B 北端执行前脚贴边迈步仿真：

```bash
cd quadruped-training-WQK10099/队内赛
scripts/run_headless.sh bridge
```

从木桥 B 北端打开 GUI，机器人会按 `BRIDGE_B_CROSS` 尝试自动迈步穿越；当前新基础策略回归结果为失败：

```bash
scripts/run_gui.sh bridge
```

从高墙南侧运行无界面高墙腾越：

```bash
scripts/run_headless.sh wall
```

从高墙南侧打开 GUI，并按 `J` 启动或重新触发高墙动作：

```bash
scripts/run_gui.sh wall
```

打开 GUI 后恢复到站立，并从木桥 B 北端手动按 `B` 启动木桥动作：

```bash
scripts/run_gui.sh manual --spawn 5.70 -1.32 0.72 --yaw-deg -90
```

从红色出生点打开交互式 GUI：

```bash
scripts/run_gui.sh manual
```

也可以直接调用入口并覆盖出生点、状态、时长和设备：

```bash
scripts/run_headless.sh manual --state LOCOMOTION --duration 20
scripts/run_headless.sh bar --spawn 5.7 -10.22 0.51 --yaw-deg 90
scripts/run_headless.sh autopilot --device cuda:0
scripts/run_headless.sh bridge --duration 100 --output-prefix results/bridge_test
scripts/run_headless.sh wall --duration 15 --output-prefix results/wall_test
```

每次运行会生成 CSV 遥测和 JSON 汇总。使用 `--output-prefix` 指定输出前缀；否则写入 `results/<scenario>_<timestamp>.{csv,json}`。

## 7. GUI 打开与操作

GUI 需要机器有图形桌面或可用的 X11/Wayland 转发。启动后 MuJoCo viewer 会显示地形和 Black 机器人；可用鼠标在 viewer 中旋转、平移和缩放视角。若要在原点启动机器人，使用：

```bash
scripts/run_gui.sh manual --spawn 0 0 0.51 --yaw-deg 0
```

GUI 入口会拦截 MuJoCo viewer 原生的 `Space` 暂停、`Right` 单步、`A` 环境显示切换和 `R` 调试可视化开关，避免它们与地形控制键冲突。

启动后还会显示一个置顶的 `Black Control` 控制面板：点击 `RESET TO ORIGIN` 会请求机器人回到原点，点击 `STOP / ZERO SPEED` 会停止手动运动并将低姿恢复为站立。也可以按 `R` 或 `Home` 执行原点复位。复位会将机器人瞬移回 `(0, 0, 0.51)`、朝向 `0°`、清空速度并恢复 `STAND`；实际位姿写入在仿真线程中执行，不会从 viewer 回调线程直接修改物理状态。

高墙固定动作操作步骤：

1. 启动 `scripts/run_gui.sh wall`，机器人位于墙南侧 `(1.80, -7.55, 0.51)` 并朝向 `+y`。
2. 场景默认已经进入 `WALL_CROSS`；若使用 `manual --spawn 1.8 -7.55 0.51 --yaw-deg 90`，按 `J` 启动。
3. 运行期间不要按前进/转向键，轨迹会自行完成起跳、收腿、腾越和恢复。
4. 成功结束后状态自动回到 `STAND`，JSON 中 `wall.stage` 应为 `done`。

木桥 B 固定动作操作步骤：

1. 启动 `scripts/run_gui.sh bridge`，或按上面的 `manual --spawn ...` 命令打开指定起点。
2. 在 viewer 窗口中确认机器人位于木桥 B 北端，两只前脚刚好落在 `y=-1.50` 桥缘。`bridge` 场景默认已经进入 `BRIDGE_B_CROSS`；`manual` 场景默认站立。
3. `manual` 场景按 `B` 启动木桥 B 动作。
4. 运行期间不要叠加前进、横移或转向命令；动作会用固定前进速度和小幅航向修正连续迈步。
5. 若成功到达南端，状态会自动回到 `STAND`，JSON 中 `bridge.stage` 为 `complete`；当前新基础策略在桥中段失败，`bridge.stage` 为 `failed`。

键盘动作：

| 按键 | 功能 |
|---|---|
| `1` | `STAND` |
| `2` | `LOCOMOTION` |
| `B` | `BRIDGE_B_CROSS`，从木桥 B 北端前脚贴边连续迈步到南端 |
| `C` | `CLIMB`，切换爬升策略 |
| `J` | `WALL_CROSS`，高墙辅助腾越 |
| `L` | `LIE_DOWN`，平滑趴下到低姿态 |
| `X` | `CRAWL_FORWARD`，低姿爬行前进 |
| `↑` | 前进；第一次按为 0.10 m/s，之后每按一次增加 0.20 m/s，最高 0.90 m/s |
| `↓` | 站立状态直接后退；第一次按为 0.10 m/s，之后每按一次增加 0.20 m/s，最高 0.50 m/s |
| `←` | 左横移；第一次按为 0.10 m/s，之后每按一次增加 0.20 m/s，最高 0.60 m/s |
| `→` | 右横移；第一次按为 0.10 m/s，之后每按一次增加 0.20 m/s，最高 0.60 m/s |
| `Q` | 向基础策略发送左转命令；第一次按为 0.10 rad/s，之后每按一次增加 0.05 rad/s，最高 0.60 rad/s；可与前进、后退、横移叠加 |
| `E` | 向基础策略发送右转命令；第一次按为 0.10 rad/s，之后每按一次增加 0.05 rad/s，最高 0.60 rad/s；可与前进、后退、横移叠加 |
| `A` / `D` | 左 / 右横移别名，与 `←` / `→` 使用同一套重复调速逻辑 |
| `空格` | 立即把前进、后退、横移和转向速度清零；若处于 `LIE_DOWN` 或 `CRAWL_FORWARD`，则先恢复到 `STAND` |
| `R` / `Home` | 重置到原点 `(0, 0, 0.51)`，朝向 `0°` 并恢复 `STAND` |

所有方向键都是“重复按压调速”：第一次按某个速度轴时从该轴的默认值开始，连续按同一方向键会累加步长；改变该轴方向时从新方向的默认值开始。前进/后退、横移和转向是三个独立速度轴，按另一组键不会清除当前命令，因此可以形成“前进 + 转向”“后退 + 横移”“前进 + 横移 + 转向”等组合。速度到上限后继续按不会超过上限，按空格会把三轴速度全部清零并把调速状态复位。

`↑`、`↓`、横移和 `Q/E` 都会进入 `LOCOMOTION`，把当前三轴命令 `(vx, vy, wz)` 一起发送给新的 45 维基础策略。只按 `Q/E` 时是原地转向；先按 `↑` 再按 `Q/E` 时会在前进过程中转向。不会再进入趴下、低姿反向爬行或分步转身控制器。当前 checkpoint 已用 0.80 m/s 前进做过短时实测；低速组合和极限转向尚未逐一重测，实际位移需要在 GUI 中观察。

第 7 节下面列出的横移、后退和转向数值均来自上一版 HIM 基础策略的历史回归，仅作操作逻辑与参考量级说明。替换为当前 flat student 后，应重新验证这些命令，不应把旧数值视为当前保证。

横移键只修改横向速度 `vy`，不会自动清除已有的前进/后退 `vx` 或转向 `wz`。坐标约定为机器人自身左侧是正 `y`，因此 `←` 对应 `vy>0`、`→` 对应 `vy<0`。横移在 0.10-0.60 m/s 内调速；已验证 0.24 m/s 和 0.60 m/s 的纯横移没有翻倒。

转向现在不再降姿，也不再调用 `StepTurnController`。按 `Q` / `E` 只修改 `wz`，并保留当前 `vx/vy`：没有前进或横移命令时是原地转向，有前进命令时是行进转向。首次为 0.10 rad/s，重复按同一转向键每次增加 0.05 rad/s，最高 0.60 rad/s。较小的 0.10-0.15 rad/s 命令响应较弱；纯转向在 0.60 rad/s 上限的 4 秒实测中，左转约 40.2°、右转约 34.9°，根高度保持在 0.43 m 左右，未翻倒，但左右响应仍不完全对称。

无界面验证横移：

```bash
scripts/run_headless.sh manual --state LOCOMOTION --spawn 0 0 0.51 \
  --yaw-deg 0 --strafe-vy 0.24 --duration 4
scripts/run_headless.sh manual --state LOCOMOTION --spawn 0 0 0.51 \
  --yaw-deg 0 --strafe-vy -0.24 --duration 4
```

实测 0.24 m/s 纯横移各 4 秒时，正方向约向 `+y` 移动 0.55 m，负方向约向 `-y` 移动 0.65 m，根高度保持在约 0.41 m，没有翻倒；在 0.32 m/s 的 4 秒单次测试中也保持稳定。由于基础策略的横向响应略有左右不对称，位移绝对值不会完全一致。

无界面验证直接后退：

```bash
scripts/run_headless.sh manual --state LOCOMOTION --spawn 0 0 0.51 \
  --yaw-deg 0 --backward-speed 0.30 --duration 4
```

`-0.10 m/s` 的 4 秒实测约后退 0.015 m，响应较弱；`-0.30 m/s` 约后退 0.50 m，最终根高度约 0.412 m，没有翻倒。后退速度上限 `-0.50 m/s` 的 4 秒实测约后退 1.43 m，最终根高度约 0.42 m，未翻倒。所有命令的状态序列都只有 `LOCOMOTION`。

无界面验证转向：

```bash
scripts/run_headless.sh manual --state LOCOMOTION --spawn 0 0 0.51 \
  --yaw-deg 0 --turn-wz 0.12 --duration 4
```

`Q/E` 直接命令不会自动进入 `RECOVER`；需要停止转向时按空格把 `wz` 清零。上述纯转向 4 秒实测中根高度约 0.426-0.427 m，状态序列只有 `LOCOMOTION`。

无界面验证“前进 + 转向”叠加：

```bash
scripts/run_headless.sh manual --state LOCOMOTION --spawn 0 0 0.51 \
  --yaw-deg 0 --forward-speed 0.50 --turn-wz 0.30 --duration 4
```

4 秒实测中机器人从原点行进到约 `(0.95, 0.56)`，偏航达到约 `54.8°`，最低根高度约 `0.403 m`，状态序列只有 `LOCOMOTION` 且未翻倒。三个速度轴可以同时发送，但组合速度越高，策略负载越大，建议先在 GUI 中从低速开始验证。

## 8. 验证范围与已知限制

- 新基础策略已通过严格参数加载和最小接口检查；本次无界面 `LOCOMOTION` 测试以 0.80 m/s 前进 4.0 s，2 s 时约到 `x≈1.25 m`，4 s 末约在 `(1.117, -0.182, 0.470)` m。动作来源全部为 `base`，4 s 内未翻倒，但后段存在回退和横向漂移，不能描述为姿态完全稳定。
- 替换后重新执行的三项回归为：`LOCOMOTION` 短时行走通过、`WALL_CROSS` 高墙动作通过、`BRIDGE_B_CROSS` 木桥动作失败。`bar` 和 `autopilot` 尚未在新 checkpoint 上重新验证。GUI 原点启动、方向键、空格和 `R` / `Home` 原点复位沿用既有自动化检查，尚未用新 checkpoint 做人工鼠标交互验收。
- 方向键使用重复按压调速：`↑/↓` 为前进/后退，`←/→` 为左右横移，`Q/E` 为转向。后退和转向均直接向当前基础策略发送命令，不经过低姿或分步转身控制器；后退最高 `0.50 m/s`，转向最高 `0.60 rad/s`。这些上限和组合命令需要在当前 flat student 上重新验证。`3-0` 和旧高墙键 `W` 已从键盘映射中删除。
- `BRIDGE_B_CROSS` 只针对本目录地形 XML 中木桥 B 的指定尺寸和指定北端起点。动作要求两只前脚距桥缘不超过 `0.04 m`，并依赖基础策略的连续行走能力；当前新基础策略在桥中段触发 `roll_limit_exceeded`，因此该动作目前不是可用通过动作。
- `WALL_CROSS` 使用短暂躯干外部冲量和姿态闭环，只针对本图 0.30 m 高墙与指定起点；改变墙高、墙厚、起点距离、朝向或质量参数后必须重新调参和验证。新版 `deploy -> land` 流程已用 seed 1-5 重复验证，均以 `STAND` 结束；落地后的 4 s 诊断中四足保持接地，根高稳定在约 `0.49 m`。
- `JUMP` checkpoint 是自主垂直跳跃策略，不能替代低姿态爬行穿越限高杆，也不作为木桥沟壑的主要通过手段。
- `EMERGENCY_STOP` 只保持软件关节目标，不构成真实机器人的硬件安全急停。
- 自动流程只在横杆南侧 staging 点起跑；从红色出生点自动接近横杆尚未可靠完成。
- 运行日志中可能出现 MuJoCo Warp 的 `solver iterations limit` 提示，这是接触求解器告警；上述成功运行仍以 JSON 中的路线里程碑、根高度、接触和包络判据为准。
- 当前木桥回归使用 `--device cpu` 和单环境，结果为失败；上一版 HIM 基础策略的木桥成功记录未随本次上传包含。改变 checkpoint、地形、初始位姿、速度命令或算力后端后都需要重新验证。


## 9. 手动示教与动作复现

可以录制完整手动操作并让机器人复现，但需要区分三种方式：

1. **关节轨迹回放**：记录每个控制时刻的 12 个关节目标，按时间回放。实现最简单，但对起点、地面接触和扰动非常敏感，纯开环只能在完全相同条件下使用。
2. **状态-动作示教 + 行为克隆**：记录观测、动作、接触和阶段标签，训练策略从观测映射到动作。比纯回放更能容忍小幅误差，但需要足够多的成功示范和失败恢复数据。
3. **技能原语/状态机复现**：把一次操作切分为站立、转向、趴下、爬行、木桥迈步、腾越等阶段，并为每段加入感知闭环。本目录的 `BRIDGE_B_CROSS` 和 `WALL_CROSS` 就是这种工程化形式。

高墙动作不能只靠回放关节角稳定复现，因为起跳速度、墙面接触和落地冲击误差会快速累积。当前实现保留了关键姿态轨迹，但使用高度、姿态、速度、墙面接触和阶段超时做闭环判定。

## 10. 目录结构

```text
队内赛/
├── README.md
├── run_simulation.py
├── configs/policies.json
├── assets/
│   ├── scene_terrain.xml
│   ├── height_field.png
│   ├── unitree_hfield.png
│   └── policies/
│       ├── student_model_299.pt
│       ├── climb_model_59.pt
│       └── jump_model_299.pt
├── scripts/
│   ├── run_gui.sh
│   ├── run_headless.sh
│   └── smoke_skills.py
├── src/black_terrain_fsm/
│   ├── config.py
│   ├── controllers.py
│   ├── mjlab_compat.py
│   ├── policies.py
│   ├── state_machine.py
│   └── terrain.py
└── results/
    ├── base_flat45_smoke.csv / base_flat45_smoke.json
    ├── bar_crawl.csv / bar_crawl.json
    ├── bridge_after_base_flat45.csv / bridge_after_base_flat45.json
    ├── wall_after_base_flat45.csv / wall_after_base_flat45.json
    └── wall_cross_soft_landing.csv / wall_cross_soft_landing.json
```
