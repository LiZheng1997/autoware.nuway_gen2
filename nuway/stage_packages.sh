#!/bin/bash
# 把本仓库自带的 nuway 包放进 src/，供 colcon 发现。
#
# 为什么不直接放在 src/ 里：上游 autoware 元仓库的 src/.gitignore 是 `*`，
# 即 src/ 按约定完全由 vcs import 生成；SOP 里也有 `rm -rf src` 重新 import
# 的操作。把版本控制的包放进去会被误删，也与上游约定冲突。
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/src/nuway"
for p in nuway_sensor_kit_launch nuway_vehicle_launch; do
  rm -rf "$ROOT/src/nuway/$p"
  cp -r "$ROOT/nuway_packages/$p" "$ROOT/src/nuway/$p"
  echo "✔ $p → src/nuway/"
done
