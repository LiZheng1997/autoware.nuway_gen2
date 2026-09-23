#!/bin/bash
# nUWAy Autoware 1.9.0 运行环境（Orin AGX / JetPack 6.2）
# 用法: source nuway/setup_env.sh
_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$_HERE/install/setup.bash" ] && source "$_HERE/install/setup.bash"

# CUDA 12.8 是 SBSA 包（targets/sbsa-linux），而 JetPack 的 CUDA 是 Tegra 布局
# （targets/aarch64-linux）。ld 缓存默认解析到 JetPack 的 12.6/12.2，
# 会导致 TensorRT 相关组件 dlopen 失败：
#   libcudart.so.12: cannot open shared object file
# 不要把 12.8 设为全系统默认——Jetson 应保留 JetPack 自带 CUDA 作为系统默认。
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/targets/sbsa-linux/lib:${LD_LIBRARY_PATH}
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=/usr/local/cuda-12.8/bin:${PATH}
