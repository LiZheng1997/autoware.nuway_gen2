#!/bin/bash
# Long-route variant of build_map_dual.sh.
#
# It swaps in nuway_campus_full.yaml, which raises cube_side_length from 300 to 1000.
# At 300 the ikd-tree evicts map points and the map is truncated part-way through any
# route longer than about 150 m. Raising it also has a useful side effect: when the
# vehicle turns back, the outbound map points are still resident, so scan-to-map ICP
# registers the return leg against them. That acts as an implicit loop closure and cut
# the vertical loop residual from 5.49 m to 0.07 m.
#
# Note that this alone does not make long maps usable for NDT - see section 7 of
# docs/mapping-and-localization.md.
#
# Usage: build_map_full.sh [duration_s]
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec env \
  OUT_DIR="${OUT_DIR:-/home/lz/campus_map/full}" \
  PCD_NAME="${PCD_NAME:-campus_full.pcd}" \
  TAG="${TAG:-full}" \
  FASTLIO_CONFIG="${FASTLIO_CONFIG:-$SCRIPT_DIR/nuway_campus_full.yaml}" \
  bash "$SCRIPT_DIR/build_map_dual.sh" "${1:-920}"
