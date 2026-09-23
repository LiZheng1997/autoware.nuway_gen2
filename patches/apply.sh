#!/bin/bash
# Run after `vcs import`, before `colcon build`.
# vcs import overwrites the imported sources, so every patch has to be
# re-applied on each fresh import. Each step is idempotent.
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

AL="$ROOT/src/launcher/autoware_launch"
P="$AL/sensor_kit/sample_sensor_kit_launch/common_sensor_launch/config/distortion_corrector_node.param.yaml"
[ -f "$P" ] || { echo "missing $P - run vcs import first"; exit 1; }
if grep -q 'processing_time_threshold_sec: 0.01' "$P"; then
  patch -p1 -d "$AL" < "$ROOT/patches/0002-relax-distortion-corrector-thresholds.patch"
  echo "✔ distortion corrector thresholds restored to the nUWAy values"
else
  echo "· distortion corrector thresholds already set"
fi
