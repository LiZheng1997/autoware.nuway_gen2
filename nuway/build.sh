#!/bin/bash
# nUWAy Autoware 1.9.0 原生编译（Orin AGX）。实测 488/488，约 2h23min。
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash

# GCC 11：ROS 2 Humble 的 apt 包由 GCC 11 构建；GCC 9 会对 std::optional
# 的 `return {};` 误报 maybe-uninitialized，叠加 Autoware 的 -Werror 成为硬错误。
export CC=/usr/bin/gcc-11 CXX=/usr/bin/g++-11
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=/usr/local/cuda-12.8/bin:$PATH
# 12 核开满会 OOM：tensorrt_yolox / lidar_centerpoint / bevdet_vendor 的单个
# CUDA 编译单元能吃数 GB。实测下列组合全程可用内存 > 50 GB。
export MAKEFLAGS=-j3

colcon build --symlink-install --continue-on-error --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DCMAKE_CUDA_ARCHITECTURES=87 \
               -DCMAKE_C_COMPILER=/usr/bin/gcc-11 \
               -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
               -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc "$@"
