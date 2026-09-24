#!/bin/bash
# Run after `vcs import`, before `colcon build`.
# vcs import overwrites imported sources, so patches are re-applied every time.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

CB="$ROOT/src/universe/external/cuda_blackboard"
[ -d "$CB" ] || { echo "missing $CB - run vcs import first"; exit 1; }
if grep -q 'cudaStreamGetDevice' "$CB/src/cuda_mem_pool_context.cpp"; then
  patch -p1 -d "$CB" < "$ROOT/patches/0001-cuda_blackboard-use-cudaGetDevice.patch"
  echo "✔ cuda_blackboard patched"
else
  echo "· cuda_blackboard already patched"
fi

CANB="$ROOT/src/nuway/nuway_canbridge"
[ -d "$CANB" ] || { echo "missing $CANB - run vcs import first"; exit 1; }
if ! grep -q 'autoware_utils_diagnostics' "$CANB/nuway_can/package.xml"; then
  patch -p0 -d "$CANB" < "$ROOT/patches/0002-canbridge-declare-missing-deps.patch"
  echo "✔ canbridge dependency declarations added"
else
  echo "· canbridge dependency declarations already present"
fi
