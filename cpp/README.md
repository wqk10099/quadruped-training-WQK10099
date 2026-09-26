# C++ Headless MuJoCo Simulation

## 目标

使用 C++ 加载 flat_scene.xml，保持 ctrl 全为零，
推进仿真 20 秒，并输出最终 qpos 和 qvel。

## 依赖

- g++
- CMake（可选）
- MuJoCo Python 包中包含的 C++ 头文件和动态库

## 查找 MuJoCo 开发文件

MUJOCO_DIR=$(python3 -c "import mujoco, pathlib; print(pathlib.Path(mujoco.__file__).resolve().parent)")

## 编译

g++ -std=c++17 simulate_prone.cpp \
  -I"$MUJOCO_DIR/include" \
  "$MUJOCO_DIR"/libmujoco.so.* \
  -Wl,-rpath,"$MUJOCO_DIR" \
  -o simulate_prone

## 运行

从项目根目录执行：

./cpp/simulate_prone "$(pwd)/scenes/flat_scene.xml"

## 预期结果

- nq = 19
- nv = 18
- nu = 12
- time = 20
- max_abs_qvel 接近 0
- ctrl 全部为 0