#!/bin/bash
# 在 vcs import 之后、colcon build 之前运行。
# vcs import 会覆盖外部依赖的源码，所以补丁必须以 .patch 形式维护并每次重新施加。
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CB="$ROOT/src/universe/external/cuda_blackboard"
[ -d "$CB" ] || { echo "未找到 $CB —— 请先执行 vcs import"; exit 1; }
if grep -q 'cudaStreamGetDevice' "$CB/src/cuda_mem_pool_context.cpp"; then
  patch -p1 -d "$CB" < "$ROOT/patches/0001-cuda_blackboard-use-cudaGetDevice.patch"
  echo "✔ cuda_blackboard 补丁已施加"
else
  echo "· cuda_blackboard 补丁已在，跳过"
fi
