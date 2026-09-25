#!/bin/bash
# Native build of Autoware 1.9.0 for nUWAy (Orin AGX). Measured: 488/488 packages, about 2 h 23 min.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
source /opt/ros/humble/setup.bash

# GCC 11 is required: the ROS 2 Humble apt packages are built with GCC 11, and GCC 9
# raises a spurious maybe-uninitialized warning on `return {};` for std::optional,
# which Autoware's -Werror turns into a hard error.
export CC=/usr/bin/gcc-11 CXX=/usr/bin/g++-11
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=/usr/local/cuda-12.8/bin:$PATH
# Using all 12 cores runs out of memory: a single CUDA translation unit in
# tensorrt_yolox, lidar_centerpoint or bevdet_vendor can consume several GB. The
# combination below was measured to keep more than 50 GB free throughout.
export MAKEFLAGS=-j3

colcon build --symlink-install --continue-on-error --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DCMAKE_CUDA_ARCHITECTURES=87 \
               -DCMAKE_C_COMPILER=/usr/bin/gcc-11 \
               -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
               -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc "$@"
