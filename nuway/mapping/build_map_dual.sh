#!/bin/bash
# Dual-lidar mapping with spark-fast-lio.
#
# Why both lidars are mandatory: each recorded frame covers roughly a 160 degree sector
# rather than a full sweep, so a single lidar leaves pitch barely observable and the
# ground tilt drifts from 2.0 to 5.1 degrees along the route, giving a median NDT NVTL
# of 0.00. Front and rear merged cover about 320 degrees and reach a median NVTL of 3.09.
#
# The merger must not be replaced by a plain concatenation. The two lidars are 37 ms out
# of sync, and merge_velodyne_clouds.py normalises every point's timestamp to the start
# of the merged scan, which is what lets FAST-LIO de-skew across both sensors.
#
# Gravity alignment is deliberately left off; see the note in mapping_nuway_dual.launch.yaml.
# The finished map is levelled afterwards by georef_map.py.
#
# Usage: build_map_dual.sh [duration_s]
# Environment overrides: OUT_DIR PCD_NAME TAG BAG FASTLIO_WS MERGER VEHICLE_URDF
#                        FASTLIO_LAUNCH FASTLIO_CONFIG
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

DUR=${1:-250}
D=${OUT_DIR:-/home/lz/campus_map/dual}
BAG=${BAG:-/home/lz/campus_data/rosbag2_2025_09_17-16_20_28}
FASTLIO_WS=${FASTLIO_WS:-/home/lz/fast_lio_ws}
MERGER=${MERGER:-/home/lz/VTR-Demo/scripts/merge_velodyne_clouds.py}
VEHICLE_URDF=${VEHICLE_URDF:-/home/lz/ASRL/vtr3/src/config/nuway_vtr_tf.urdf.xml}
FASTLIO_LAUNCH=${FASTLIO_LAUNCH:-$SCRIPT_DIR/mapping_nuway_dual.launch.yaml}
FASTLIO_CONFIG=${FASTLIO_CONFIG:-}
PCD_NAME=${PCD_NAME:-campus_dual.pcd}
TAG=${TAG:-dual}

rm -rf "$D"; mkdir -p "$D"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-71}
source /opt/ros/humble/setup.bash
source "$FASTLIO_WS/install/setup.bash"
trap 'kill -- -$$ 2>/dev/null' EXIT
log(){ echo "[$TAG] $(date +%T) $*"; }

log "TF tree"
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat "$VEHICLE_URDF")" \
  -p use_sim_time:=true > "$D/tf.log" 2>&1 &
sleep 5

log "dual-lidar merge node"
python3 "$MERGER" --ros-args -p use_sim_time:=true > "$D/merge.log" 2>&1 &
sleep 4

log "FAST-LIO2 on the merged cloud"
ros2 launch "$FASTLIO_LAUNCH" ${FASTLIO_CONFIG:+config_path:="$FASTLIO_CONFIG"} \
  > "$D/fastlio.log" 2>&1 &
sleep 10

log "collector + trajectory recorder"
python3 "$SCRIPT_DIR/collect_map.py" "$D/$PCD_NAME" > "$D/collect.log" 2>&1 &
CP=$!
python3 "$SCRIPT_DIR/record_traj.py" "$D" > "$D/traj.log" 2>&1 &
TP=$!
sleep 3

log "replaying first ${DUR}s (front+rear lidar)"
timeout "$DUR" ros2 bag play "$BAG" \
  --topics /lidar/velodyne/front/cloud /lidar/velodyne/rear/cloud /imu/data /gps/fix \
  --clock -r 1.0 > "$D/play.log" 2>&1

log "wrapping up"
sleep 12
kill -INT $CP $TP 2>/dev/null; wait $CP $TP 2>/dev/null
log "merge node: $(grep -c . "$D/merge.log") lines, $(grep -ci warn "$D/merge.log") warnings"
tail -2 "$D/collect.log"
tail -1 "$D/traj.log"
ls -la "$D/$PCD_NAME" 2>/dev/null
echo "DONE_${TAG^^} $(date +%T)"
