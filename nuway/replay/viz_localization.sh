#!/bin/bash
# 定位可视化会话: 起定位栈 + 回放, 供 RViz 观看。RViz 可跑在本机(:0)或 G7(同 DDS 域)。
MAP=${1:-/home/lz/campus_map/autoware_map_dual}
RATE=${2:-0.5}
RVIZ=${3:-0}                 # 1=在 Orin 本机开 RViz
D=/home/lz/campus_map/viz
rm -rf "$D"; mkdir -p "$D"
trap 'kill -- -$$ 2>/dev/null' EXIT
source /home/lz/aw_verify/nuway/setup_env.sh
export GNSS_INS_ORIENTATION=false
export DISPLAY=${DISPLAY:-:0}
BAG=/home/lz/campus_data/rosbag2_2025_09_17-16_20_28
log(){ echo "[viz] $(date +%T) $*"; }
ros2 daemon stop >/dev/null 2>&1

log "定位栈 (rviz=$RVIZ)"
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:="$MAP" vehicle_model:=nuway_vehicle sensor_model:=nuway_sensor_kit \
  perception:=false planning:=false control:=false rviz:=$([ "$RVIZ" = 1 ] && echo true || echo false) \
  > "$D/launch.log" 2>&1 &
sleep 50
log "死亡=$(grep -c 'process has died' "$D/launch.log")"

python3 /home/lz/campus_map/bag_to_autoware.py > "$D/adapter.log" 2>&1 &
sleep 4
log "回放 rate=$RATE"
ros2 bag play "$BAG" --clock -r "$RATE" > "$D/play.log" 2>&1 &
for i in $(seq 1 25); do sleep 2; grep -q '时间偏移标定完成' "$D/adapter.log" && break; done
log "$(grep -m1 '时间偏移标定完成' "$D/adapter.log" | sed 's/.*\]: //')"
sleep 12
for try in 1 2 3; do
  log "触发 GNSS 初始化 (第 $try 次)"
  timeout 30 ros2 service call /localization/initialize \
    autoware_internal_localization_msgs/srv/InitializeLocalization "{}" >> "$D/init.log" 2>&1
  sleep 8
  grep -q 'NDT Activation succeeded' "$D/launch.log" && { log "初始化成功, 可以看了"; break; }
done
log "保持运行 600 s (Ctrl-C 或 kill 进程组结束)"
sleep 600
