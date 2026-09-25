#!/bin/bash
# Short verification run: continuous localization across the whole map, configurable collection
# duration, outputs a per-frame time series.
# Usage: ndt_verify.sh <map_dir> <output_dir> <replay_start_offset_s> <collect_duration_s>
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
MAP=${1:-/home/lz/campus_map/autoware_map_dual}
D=${2:-/home/lz/campus_map/verify}
OFF=${3:-0}
COL=${4:-240}
rm -rf "$D"; mkdir -p "$D"
trap 'kill -- -$$ 2>/dev/null' EXIT

source "$REPO_DIR/nuway/setup_env.sh"
export GNSS_INS_ORIENTATION=false
BAG=${BAG:-/home/lz/campus_data/rosbag2_2025_09_17-16_20_28}
log(){ echo "[ndt] $(date +%T) $*"; }
ros2 daemon stop >/dev/null 2>&1

log "map=$MAP  start=${OFF}s  collect=${COL}s"
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:="$MAP" vehicle_model:=nuway_vehicle sensor_model:=nuway_sensor_kit \
  perception:=false planning:=false control:=false rviz:=false \
  > "$D/launch.log" 2>&1 &
sleep 50
log "died=$(grep -c 'process has died' "$D/launch.log")"

python3 "$SCRIPT_DIR/bag_to_autoware.py" > "$D/adapter.log" 2>&1 &
sleep 4
ros2 bag play "$BAG" --clock --start-offset "$OFF" > "$D/play.log" 2>&1 &
for i in $(seq 1 20); do sleep 2; grep -q 'Time offset calibration complete' "$D/adapter.log" && break; done
log "$(grep -m1 'Time offset calibration complete' "$D/adapter.log" | sed 's/.*\]: //')"
sleep 10

for try in 1 2 3; do
  log "triggering GNSS pose initialization (attempt $try)"
  timeout 30 ros2 service call /localization/initialize \
    autoware_internal_localization_msgs/srv/InitializeLocalization "{}" >> "$D/init.log" 2>&1
  sleep 8
  grep -q 'NDT Activation succeeded' "$D/launch.log" && { log "initialization succeeded"; break; }
done

log "collecting continuously for ${COL}s (covers the whole map)"
python3 "$SCRIPT_DIR/collect_ndt.py" "$COL" "$D/nvtl.csv" > "$D/metrics.txt" 2>&1
log "=== results ==="
echo "P2 process deaths: $(grep -c 'process has died' "$D/launch.log")"
echo "time validation errors: $(grep -c 'Validation error' "$D/launch.log")"
grep -v '^\[' "$D/metrics.txt"
echo "playback finished: $(grep -c 'Playback complete' "$D/play.log")"
echo "DONE_VERIFY $(date +%T)"
