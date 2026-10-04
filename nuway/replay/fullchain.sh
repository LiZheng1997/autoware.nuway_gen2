#!/bin/bash
# Replay the campus bag through the whole Autoware stack and verify the chain end to end:
# map -> localization -> perception -> route -> planning -> control.
#
# Settings that are not optional, each one learned from a failure:
#   --start-offset 640   the first ~460 s of the bag are driven in REVERSE, so the lane
#                        (which follows the vehicle heading) cannot be routed there.
#                        640 s puts the ego inside the mapped corridor, driving forward,
#                        by the time the goal is set. find_forward_window.py locates this.
#   reverse=1 on the goal fraction, matching how the lanelet2 map was generated.
#   CycloneDDS on one interface - set by nuway/setup_env.sh. With the rmw_fastrtps_cpp
#                        default across 13 interfaces the planning chain never met
#                        scenario_selector's 1.0 s delay gate at any replay rate.
#
# Usage: fullchain.sh [rate=1.0] [offset_s=640] [probe_s=40] [map_dir]
# No "set -u": ROS 2 setup.bash dereferences unset variables (AMENT_TRACE_SETUP_FILES).
RATE=${1:-1.0}
OFFSET=${2:-640}
PROBE=${3:-40}
MAP=${4:-$HOME/campus_map/autoware_map_lanelet}
BAG=${BAG:-$HOME/campus_data/rosbag2_2025_09_17-16_20_28}
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
OUT=${OUT:-$HOME/campus_map/fullchain_run}
rm -rf "$OUT"; mkdir -p "$OUT"
trap 'kill -- -$$ 2>/dev/null' EXIT

source "$ROOT/nuway/setup_env.sh"
export GNSS_INS_ORIENTATION=false
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-71}
log(){ echo "[fullchain] $(date +%T) $*"; }
ros2 daemon stop >/dev/null 2>&1

log "RMW=$RMW_IMPLEMENTATION on ${NUWAY_DDS_IFACE}, replay ${RATE}x from ${OFFSET}s, map $(basename "$MAP")"
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:="$MAP" \
  vehicle_model:=nuway_vehicle sensor_model:=nuway_sensor_kit \
  perception:=true planning:=true control:=true rviz:=false \
  > "$OUT/launch.log" 2>&1 &
sleep 110
log "process deaths: $(grep -c 'process has died' "$OUT/launch.log")"

python3 "$ROOT/nuway/replay/bag_to_autoware.py" > "$OUT/adapter.log" 2>&1 &
sleep 5
ros2 bag play "$BAG" --clock -r "$RATE" --start-offset "$OFFSET" > "$OUT/play.log" 2>&1 &
for _ in $(seq 1 40); do
  sleep 2
  grep -q 'calibration complete' "$OUT/adapter.log" && break
done
log "$(grep -m1 'calibration complete' "$OUT/adapter.log" | sed 's/.*\]: //')"
sleep 12

for try in 1 2 3; do
  log "triggering GNSS initialisation (attempt $try)"
  timeout 30 ros2 service call /localization/initialize \
    autoware_internal_localization_msgs/srv/InitializeLocalization "{}" >> "$OUT/init.log" 2>&1
  sleep 8
  grep -q 'NDT Activation succeeded' "$OUT/launch.log" && { log "localization initialised"; break; }
done

log "setting goal"
python3 "$ROOT/nuway/replay/set_goal.py" "$MAP/traj_map.csv" 0.50 5 1 2>&1 | tail -2 | sed 's/^/  /'
sleep 8
log "route: $(grep -c 'Route set via' "$OUT/launch.log") set, $(grep -c 'Failed to plan route' "$OUT/launch.log") failed"

log "=== chain ==="
python3 "$ROOT/nuway/tools/chain_probe.py" "$PROBE" 2>&1 | sed 's/^/  /'
log "trajectories dropped as late: $(grep -c 'trajectory is delayed' "$OUT/launch.log")"
log "load $(uptime | sed 's/.*average: //' | cut -d, -f1)  deaths $(grep -c 'process has died' "$OUT/launch.log")"
log "DONE  (logs in $OUT)"
