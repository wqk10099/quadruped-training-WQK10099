#!/usr/bin/env bash
# 一键运行脚本：进入 python/ 目录，并设置 SDK 动态库搜索路径
set -e

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE/python"

# sudo 会清理环境变量，用 `sudo env` 显式传入 LD_LIBRARY_PATH
sudo env LD_LIBRARY_PATH="$HERE/lib:${LD_LIBRARY_PATH}" python3 motor_zero.py
