#!/bin/bash
# 双雷达 + 重力对齐建图。相对 build_map_short.sh 的三处改动:
#   1. 起 merge_velodyne_clouds.py, 把前后雷达按 TF 合到 base_link(保留逐点 time)
#   2. FAST-LIO 用 mapping_nuway_dual.launch.yaml (合并云配置 + 重力对齐开启)
#   3. 回放放两个雷达话题(原来只放了前雷达)
DUR=${1:-920}
D=/home/lz/campus_map/full
rm -rf "$D"; mkdir -p "$D"
export ROS_DOMAIN_ID=71
source /opt/ros/humble/setup.bash
source /home/lz/fast_lio_ws/install/setup.bash
trap 'kill -- -$$ 2>/dev/null' EXIT
log(){ echo "[full] $(date +%T) $*"; }

log "TF 树"
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat /home/lz/ASRL/vtr3/src/config/nuway_vtr_tf.urdf.xml)" \
  -p use_sim_time:=true > "$D/tf.log" 2>&1 &
sleep 5

log "双雷达合并节点"
python3 /home/lz/VTR-Demo/scripts/merge_velodyne_clouds.py \
  --ros-args -p use_sim_time:=true > "$D/merge.log" 2>&1 &
sleep 4

log "FAST-LIO2 (合并云 + 重力对齐)"
ros2 launch /home/lz/campus_map/mapping_nuway_dual.launch.yaml config_path:=/home/lz/campus_map/nuway_campus_full.yaml > "$D/fastlio.log" 2>&1 &
sleep 10

log "收集器 + 轨迹记录"
python3 /home/lz/collect_map.py "$D/campus_full.pcd" > "$D/collect.log" 2>&1 &
CP=$!
python3 /home/lz/record_traj.py "$D" > "$D/traj.log" 2>&1 &
TP=$!
sleep 3

log "回放前 ${DUR} 秒 (前+后雷达)"
timeout "$DUR" ros2 bag play /home/lz/campus_data/rosbag2_2025_09_17-16_20_28 \
  --topics /lidar/velodyne/front/cloud /lidar/velodyne/rear/cloud /imu/data /gps/fix \
  --clock -r 1.0 > "$D/play.log" 2>&1

log "收尾"
sleep 12
kill -INT $CP $TP 2>/dev/null; wait $CP $TP 2>/dev/null
log "合并节点: $(grep -c . "$D/merge.log") 行, 警告 $(grep -ci warn "$D/merge.log")"
log "重力对齐: $(grep -c 'Gravity alignment complete' "$D/fastlio.log") 次"
grep -m1 'Gravity alignment complete' "$D/fastlio.log" | cut -c1-200
tail -2 "$D/collect.log"
tail -1 "$D/traj.log"
ls -la "$D/campus_full.pcd" 2>/dev/null
echo "DONE_FULL $(date +%T)"
