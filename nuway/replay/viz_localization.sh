#!/bin/bash
# Localization visualization session: brings up the localization stack + replay, for watching in
# RViz. RViz can run on this machine (:0) or on G7 (same DDS domain).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
MAP=${1:-/home/lz/campus_map/autoware_map_dual}
RATE=${2:-0.5}
RVIZ=${3:-0}                 # 1=launch RViz on the Orin itself
D=/home/lz/campus_map/viz
rm -rf "$D"; mkdir -p "$D"
trap 'kill -- -$$ 2>/dev/null' EXIT
source "$REPO_DIR/nuway/setup_env.sh"
export GNSS_INS_ORIENTATION=false
export DISPLAY=${DISPLAY:-:0}
BAG=${BAG:-/home/lz/campus_data/rosbag2_2025_09_17-16_20_28}
log(){ echo "[viz] $(date +%T) $*"; }
ros2 daemon stop >/dev/null 2>&1

log "localization stack (rviz=$RVIZ)"
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:="$MAP" vehicle_model:=nuway_vehicle sensor_model:=nuway_sensor_kit \
  perception:=false planning:=false control:=false rviz:=$([ "$RVIZ" = 1 ] && echo true || echo false) \
  > "$D/launch.log" 2>&1 &
sleep 50
log "died=$(grep -c 'process has died' "$D/launch.log")"

python3 "$SCRIPT_DIR/bag_to_autoware.py" > "$D/adapter.log" 2>&1 &
sleep 4
log "replaying rate=$RATE"
ros2 bag play "$BAG" --clock -r "$RATE" > "$D/play.log" 2>&1 &
for i in $(seq 1 25); do sleep 2; grep -q 'Time offset calibration complete' "$D/adapter.log" && break; done
log "$(grep -m1 'Time offset calibration complete' "$D/adapter.log" | sed 's/.*\]: //')"
sleep 12
for try in 1 2 3; do
  log "triggering GNSS initialization (attempt $try)"
  timeout 30 ros2 service call /localization/initialize \
    autoware_internal_localization_msgs/srv/InitializeLocalization "{}" >> "$D/init.log" 2>&1
  sleep 8
  grep -q 'NDT Activation succeeded' "$D/launch.log" && { log "initialization succeeded, ready to watch"; break; }
done
log "staying up for 600 s (Ctrl-C or kill the process group to stop)"
sleep 600
