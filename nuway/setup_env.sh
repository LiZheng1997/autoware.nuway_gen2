#!/bin/bash
# Runtime environment for nUWAy Autoware 1.9.0 (Orin AGX / JetPack 6.2)
# Usage: source nuway/setup_env.sh
_HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$_HERE/install/setup.bash" ] && source "$_HERE/install/setup.bash"

# CUDA 12.8 is the SBSA package (targets/sbsa-linux), whereas JetPack's CUDA uses the
# Tegra layout (targets/aarch64-linux). The ld cache resolves to JetPack's 12.6/12.2 by
# default, which makes the TensorRT-related components fail to dlopen:
#   libcudart.so.12: cannot open shared object file
# Do not make 12.8 the system-wide default - on Jetson the JetPack CUDA should stay as
# the system default.
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/targets/sbsa-linux/lib:${LD_LIBRARY_PATH}
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=/usr/local/cuda-12.8/bin:${PATH}
