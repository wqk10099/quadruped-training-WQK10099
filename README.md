# MuJoCo 四足机器人基础仿真

## 项目目标

使用 Python 将四足机器人的 URDF 模型转换并整理为 MuJoCo 的 MJCF 模型，在 MuJoCo 中加载平坦地形并运行仿真，使机器狗在关节输出力矩为零的条件下稳定趴卧。

本阶段重点不是实现步态控制，而是建立完整的仿真流程和模型理解：

- 从 GitHub 获取四足机器人 URDF 与网格文件；
- 学习 URDF 与 MJCF 的结构差异；
- 将 URDF 转换为 MJCF，并手动检查、整理转换结果；
- 在 MJCF 中为机器人基座添加自由基座；
- 将 12 个关节配置为 `motor` 力矩执行器；
- 在平坦地面场景中加载机器狗；
- 保持 `data.ctrl[:] = 0.0`，验证机器人能够稳定趴卧；
- 阅读 `unitree_mujoco` 等开源项目，进一步优化代码结构、线程和仿真循环设计；
- 选做：使用 C++ 重新完成上述任务。

## 项目状态

- [x] 获取机器人 URDF 和全部 STL 网格文件
- [x] 完成 URDF 到 MJCF 的初步转换
- [x] 整理转换后的 MJCF 模型和网格路径
- [x] 为 `trunk` 基座添加 `<freejoint/>`
- [x] 添加 12 个 `motor` 力矩执行器
- [x] 编写平坦地面场景
- [x] 编写 Python 仿真运行脚本
- [x] 在零力矩输入下观察机器狗趴卧状态
- [ ] 进一步整理程序结构
- [ ] 参考 `unitree_mujoco` 优化仿真与控制循环
- [ ] 选做：使用 C++ 重写核心仿真程序

## 环境与依赖

建议使用 Ubuntu 或兼容的 Linux 环境。

- Python 3
- MuJoCo Python 包
- NumPy（MuJoCo Python 依赖）
- Git（用于源码和版本管理，可选）

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
├── scripts/
│   └── simulate_flat.py
├── .gitignore
└── README.md
```

文件职责：

- `black_description.urdf`：原始 URDF 模型；
- `black_description_mjcf_raw.xml`：URDF 转换后的原始 MJCF 结果；
- `black_description_mjcf.xml`：整理后实际使用的 MJCF 模型；
- `meshes/`：URDF/MJCF 使用的 STL 网格；
- `flat_scene.xml`：平坦地面和灯光组成的场景；
- `simulate_flat.py`：加载场景、推进物理仿真并显示 Viewer 的 Python 程序。

## URDF 来源与转换过程

原始模型来自 GitHub 用户 `N-W-wolf` 的 `Training_Materials` 仓库中的四足机器人资料。请以实际获取到的仓库路径和文件名核对来源。

转换和整理过程如下：

1. 下载 URDF 文件及其 `meshes` 目录；
2. 使用 `https://urdf.enkeebot.com/zh/` 等工具预览和转换 URDF；
3. 保存未经修改的转换结果，例如 `black_description_mjcf_raw.xml`；
4. 检查 `compiler`、`asset`、`mesh` 路径是否正确；
5. 核对每条腿的 `hip`、`thigh`、`calf` 关节名称、轴、范围和父子关系；
6. 在 `trunk` 基座 body 中增加 `<freejoint/>`；
7. 在 `<actuator>` 中为 12 个关节分别添加 `<motor>`；
8. 保存整理后的模型为 `black_description_mjcf.xml`。

当前模型的预期结构为：

| 项目 | 数量 | 说明 |
|---|---:|---|
| 自由基座 | 1 | 位于 `trunk` |
| `hinge` 关节 | 12 | 每条腿 3 个关节 |
| `motor` 执行器 | 12 | 与 12 个关节一一对应 |
| 网格资源 | 14 | STL 模型文件 |

因此模型编译后通常可以得到：

```text
nq = 19   # 自由基座 7 + 12 个关节角
nv = 18   # 自由基座 6 + 12 个关节速度
nu = 12   # 12 个电机执行器
```

## 运行的场景与控制方式

`flat_scene.xml` 通过 `<include>` 引入整理后的机器人模型，并在地面添加 `plane` 类型的地板：

```xml
<include file="../models/black/black_description_mjcf.xml"/>

<worldbody>
    <light pos="0 0 3"/>
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.8 0.8 0.8 1"/>
</worldbody>
```

`simulate_flat.py` 的主要流程为：

1. 加载 `../scenes/flat_scene.xml`；
2. 创建 `MjModel` 和 `MjData`；
3. 将 `data.ctrl[:]` 设置为 `0.0`；
4. 启动 `mujoco.viewer.launch_passive()`；
5. 在循环中调用 `mujoco.mj_step()` 和 `viewer.sync()`；
6. 使用 `time.sleep()` 控制仿真接近实时运行。

运行方式：

```bash
cd ~/mujoco_training/03_robot_dog/project/scripts
python3 simulate_flat.py
```

注意：脚本中的模型路径是相对于当前终端目录的。必须在 `scripts` 目录下运行，否则可能找不到 `../scenes/flat_scene.xml`。

## 当前实现结果

- 模型可以成功转换为 MJCF 并由 MuJoCo 加载；
- 12 个关节均已配置为 `motor`，没有使用位置或速度执行器；
- 仿真脚本保持 `data.ctrl[:] = 0.0`；
- 机器狗由重力驱动，在实际初始姿态和碰撞体配置下能够与地面形成接触；
- 目标结果为机器狗稳定趴卧在平坦地面上，不持续抖动或漂移。

在确认结果时，建议同时观察：

```python
print("nq =", model.nq)
print("nv =", model.nv)
print("nu =", model.nu)

print("time =", data.time)
print("qpos =", data.qpos)
print("qvel =", data.qvel)
print("ctrl =", data.ctrl)
```

重点检查：

- `nu` 是否为 12；
- `data.ctrl` 是否始终为零；
- 机器狗是否出现穿透地面、剧烈弹跳或持续抖动；
- 关节是否超出合理范围；
- 最终姿态是否稳定。

## 已知问题与后续计划

当前阶段的主要问题：

- `simulate_flat.py` 使用相对路径，必须从 `scripts/` 目录运行；
- 仿真代码目前集中在单个脚本中，后续需要拆分模型、控制器和仿真循环；
- URDF 转 MJCF 后仍需要继续核对碰撞体、质量、惯量和 mesh 路径；
- 当前只验证零力矩下的趴卧状态，还没有实现步态或姿态控制；
- 尚未参考 `unitree_mujoco` 完成线程、实时循环和工程结构优化。

后续计划：

1. 阅读 `unitree_mujoco` 的 README、程序入口和主循环；
2. 记录模型加载、状态读取、`data.ctrl` 写入和 `mj_step()` 的位置；
3. 将代码逐步拆分为 `robot.py`、`controller.py`、`simulator.py` 和 `main.py`；
4. 区分物理仿真步和控制更新步，必要时设计独立线程或固定频率循环；
5. 在稳定的模型和零力矩结果基础上，再学习 PD 控制和步态控制；
6. 选做使用 C++ 重新实现核心流程，并比较 Python 与 C++ 的结构和性能。
