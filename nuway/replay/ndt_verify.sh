#!/bin/bash
# 短途验证: 在整段地图上连续定位, 采集时长可配, 输出逐帧时序。
# 用法: ndt_verify.sh <地图目录> <输出目录> <回放起始偏移s> <采集时长s>
MAP=${1:-/home/lz/campus_map/autoware_map_dual}
D=${2:-/home/lz/campus_map/verify}
OFF=${3:-0}
COL=${4:-240}
rm -rf "$D"; mkdir -p "$D"
trap 'kill -- -$$ 2>/dev/null' EXIT

source /home/lz/aw_verify/nuway/setup_env.sh
export GNSS_INS_ORIENTATION=false
BAG=/home/lz/campus_data/rosbag2_2025_09_17-16_20_28
log(){ echo "[ndt] $(date +%T) $*"; }
ros2 daemon stop >/dev/null 2>&1

log "地图=$MAP  起始=${OFF}s  采集=${COL}s"
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:="$MAP" vehicle_model:=nuway_vehicle sensor_model:=nuway_sensor_kit \
  perception:=false planning:=false control:=false rviz:=false \
  > "$D/launch.log" 2>&1 &
sleep 50
log "死亡=$(grep -c 'process has died' "$D/launch.log")"

python3 /home/lz/campus_map/bag_to_autoware.py > "$D/adapter.log" 2>&1 &
sleep 4
ros2 bag play "$BAG" --clock --start-offset "$OFF" > "$D/play.log" 2>&1 &
for i in $(seq 1 20); do sleep 2; grep -q '时间偏移标定完成' "$D/adapter.log" && break; done
log "$(grep -m1 '时间偏移标定完成' "$D/adapter.log" | sed 's/.*\]: //')"
sleep 10

for try in 1 2 3; do
  log "触发 GNSS 位姿初始化 (第 $try 次)"
  timeout 30 ros2 service call /localization/initialize \
    autoware_internal_localization_msgs/srv/InitializeLocalization "{}" >> "$D/init.log" 2>&1
  sleep 8
  grep -q 'NDT Activation succeeded' "$D/launch.log" && { log "初始化成功"; break; }
done

log "连续采集 ${COL}s (覆盖整段地图)"
python3 /home/lz/campus_map/collect_ndt.py "$COL" "$D/nvtl.csv" > "$D/metrics.txt" 2>&1
log "=== 结果 ==="
echo "P2 进程死亡: $(grep -c 'process has died' "$D/launch.log")"
echo "时间校验错误: $(grep -c 'Validation error' "$D/launch.log")"
grep -v '^\[' "$D/metrics.txt"
echo "回放是否跑完: $(grep -c 'Playback complete' "$D/play.log")"
echo "DONE_VERIFY $(date +%T)"
