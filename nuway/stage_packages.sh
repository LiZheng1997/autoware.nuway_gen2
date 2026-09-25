#!/bin/bash
# Copy the nuway packages bundled with this repository into src/ so colcon finds them.
#
# Why they are not kept in src/ directly: the upstream autoware meta-repository ships a
# src/.gitignore of `*`, meaning src/ is by convention generated entirely by vcs import,
# and the SOP includes `rm -rf src` followed by a fresh import. Version-controlled
# packages placed there would be deleted by that step and would also contradict the
# upstream convention.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$ROOT/src/nuway"
for p in nuway_sensor_kit_launch nuway_vehicle_launch; do
  rm -rf "$ROOT/src/nuway/$p"
  cp -r "$ROOT/nuway_packages/$p" "$ROOT/src/nuway/$p"
  echo "✔ $p → src/nuway/"
done
