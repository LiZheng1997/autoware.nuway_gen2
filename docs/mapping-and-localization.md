# nUWAy 点云建图与 NDT 定位

用 campus 数据包在 Orin 上从零建出 Autoware 可用的点云地图，并通过 NDT 定位验收。

**实测结论（2026-09-24/25）**：320 m 的短图**已通过验收** —— NVTL 中位 3.11、全程 99.9% 在
阈值 2.3 之上、NDT 位姿 9.9 Hz 满速、GNSS 不带 RTK 一次初始化成功。
**728 m 与 1100 m 的长图不可用**，原因是 LIO 的非刚性形变，不是参数能调好的（见第 7 节）。

> **路径说明**：本目录脚本里的绝对路径按本机实测环境写死
> （`/home/lz/campus_data/...`、`/home/lz/fast_lio_ws`、`/home/lz/VTR-Demo`、`/home/lz/aw_verify`）。
> 换机器时逐个改，或在脚本顶部加变量覆盖。

---

## 1. 依赖

| 件 | 位置 | 说明 |
|---|---|---|
| spark-fast-lio | `~/fast_lio_ws` | LIO 里程计与配准点云 |
| 双雷达合并节点 | `~/VTR-Demo/scripts/merge_velodyne_clouds.py` | **不要另写** —— 它已正确处理逐点时间 |
| TF 树 | `~/ASRL/vtr3/src/config/nuway_vtr_tf.urdf.xml` | 建图期间的整车 TF |
| Autoware 1.9.0 | `~/aw_verify` | 定位验收 |

## 2. 为什么必须用双雷达

包里每帧点云只有约 12,500 点、时间跨度 44~48 ms —— 是**约 160° 的扇区扫描**，不是整圈。
单用前雷达 = 单个 160° 扇区，俯仰可观测性极弱：

| | 单前雷达 | 双雷达合并 |
|---|---|---|
| 地面倾角沿行程 | 2.0° → 5.1°（累积漂移） | 0.20° → 1.77°（基本恒定） |
| 地面拟合残差 | 1.77 m | 1.14 m |
| NDT NVTL 中位 | **0.00**（完全不收敛） | **3.09** |

前后两台合起来约 320° 覆盖，这是方案成立的关键。**上车采集时不要改雷达的角度窗口。**

另外前后雷达的 header **相差 37 ms、并不同步**，靠 `merge_velodyne_clouds.py` 的
`normalize_time`（把逐点时间归一到合并帧起点并排序）对齐，FAST-LIO 才能跨两台雷达正确去畸变。

## 3. 建图

```bash
bash nuway/mapping/build_map_dual.sh 250      # 时长(秒)
```

它依次起：TF 树 → 合并节点 → FAST-LIO（`mapping_nuway_dual.launch.yaml`）→
收集器(`collect_map.py`) + 轨迹记录(`record_traj.py`) → 回放**两个**雷达话题。
产物：`campus_dual.pcd`（0.2 m 体素）、`odo.csv`、`gps.csv`。

`build_map_full.sh` 是长路线版本，额外用 `nuway_campus_full.yaml` 把
`cube_side_length` 从 300 放大到 1000 —— 300 对超过 150 m 的路线会**把地图半途截断**。
放大后还有个副作用正好有用：车折返时去程的地图点仍在 ikd-tree 里，
scan-to-map ICP 会把回程配上去，相当于一次隐式回环（z 闭合误差 5.49 m → 0.07 m）。

> ⚠ **不要开 `gravity_alignment.enable_gravity_alignment`**。它的 `isMotionStopped()`
> 判据是 `‖acc_ref − acc_curr‖ ≤ 0.2` 即算静止，而 nUWAy 以 1.25 m/s 匀速缓行超不过该阈值，
> 计数器永远归零、节点一帧不发。实测卡死、收集器 0 帧。重力对齐改为事后校平（第 4 节）。

## 4. 体检与地理配准

```bash
python3 nuway/mapping/check_map.py <pcd> 6 <odo.csv>     # 弯道路线必须传 odo.csv
python3 nuway/mapping/georef_map.py <odo.csv> <gps.csv> <pcd> <输出目录> 2.4
```

`check_map.py` 沿**轨迹弧长**分段拟合地面，判读"恒定倾角（可一次旋转校平）"还是"累积漂移"。
按坐标轴分段会把 L 形路线上高程不同的两段揉进同一 bin，拟出 `z0 = −30.58 m` 这种假值。

`georef_map.py` 做四件事：① 用地图地面法向做一次 SE(3) 校平（**仅当倾角恒定时合法**）；
② 里程计与 GNSS 做 RANSAC + Kabsch 的 SE(2) 拟合；③ 地面平移到 z=0；
④ 写 `map_projector_info.yaml`（`LocalCartesianUTM`，高程取 GNSS 中位数减天线高 2.4 m）。
输出目录即 Autoware 的 `map_path`，含 `pointcloud_map.pcd` / `map_projector_info.yaml` /
占位 `lanelet2_map.osm`（NDT 不需要矢量地图，但 map_loader 会加载它；规划需要手绘车道网络）。

`check_drift.py` 用往返重访量累积漂移：用 GNSS 把回程点配到去程的同一物理位置，
再比里程计之差 —— **不需要真值也不需要回环软件**。注意它输出的"水平漂移"列混入了
往返的横向位置差与 GNSS 配对误差（配对距可达 8 m），水平漂移以首尾闭合为准，z 列才干净。

## 5. 回放验收

```bash
bash nuway/replay/ndt_verify.sh <地图目录> <输出目录> <回放起始偏移s> <采集时长s>
```

`bag_to_autoware.py` 是**回放专用适配器**，实车不需要。它补三件事：

1. **时间基准**。包里所有话题的 header 比 bag 录制时刻超前约 **18.2 s**（采集机与传感器整机时钟
   不同步，各传感器之间是一致的），而 `--clock` 按录制时刻发，NDT 的 1 s 容差必然失败。
   适配器自标定该偏移并把转发消息搬到 `/clock` 基准上。
   标定采用**稳定性门控**（预热 6 s + 最近 40 样本极差 <0.1 s 才锁定）——
   带 `--start-offset` 的回放开头会突发投递积压消息，取头 N 个样本会读到偏 30 s 的暂态值。
2. **点云类型**。包是旧 velodyne 驱动录的 `PointXYZIRT`(22 B)，1.9.0 的预处理链要
   `PointXYZIRC`(16 B)。实车上 nebula 驱动原生产出 `PointXYZIRCAEDT`，不存在这个问题。
3. **车速**。包里车速在 `/can_twist_fb`，Autoware 要 `/vehicle/status/velocity_status`
   (`VelocityReport`)；缺了它 gyro_odometer 没有 twist，EKF 只能空转。

另外 frame 名要对齐（包里 `lidar_velodyne_front/rear` → sensor kit 的 `velodyne_front/rear_link`），
外参一律以 `nuway_sensor_kit_description` 的实车标定为准。

> ⚠ **指标采集不要用 `ros2 topic echo`**。`ros2 daemon` 一旦被 `kill -KILL -<PGID>` 连带杀死，
> 后续所有 echo 会启动即崩（`xmlrpc Fault: !rclpy.ok()`），采集文件全是 traceback，
> 于是一路报出假的 "NVTL=0"。用 `collect_ndt.py`（rclpy 直接订阅）。

## 6. 可视化

```bash
bash nuway/replay/viz_localization.sh <地图目录> 0.5 0     # Orin 上起定位栈, 0=不开本机 rviz
# 另一台同网段机器（DDS 跨机发现可用）：
rviz2 -d nuway/replay/nuway_localization.rviz --ros-args -p use_sim_time:=true
```

**`use_sim_time:=true` 不能省** —— 回放用的是包内时间戳，RViz 默认走墙上时间，
两者差一年，TF 查询全部失败、画面空白。
该配置不依赖 Autoware 的自定义 rviz 插件，原生 `rviz2` 即可打开。

## 7. 长图为什么不可用

同一张 728 m 图、同一段数据、**只改全局刚性变换**，在起点区测：

| | 全局拟合校平+SE(2) | 只用起点区拟合 | （参照）250 s 短图 |
|---|---|---|---|
| NVTL 中位 | 2.08 | **3.00** | 3.09 |
| 迭代中位 | 25 | 3 | 2 |
| NDT 位姿帧数 | 148 | 628 | 1170 |

地图点一个没动，NVTL 就回来了 ⇒ **没有任何单一刚性变换能同时服务整张长图**，
每个区域想要的变换都不同（全局与局部拟合的 yaw 差 3.4°）。这是非刚性形变。

**已用实测排除的其他嫌疑**（不要重复排查）：

- `cube_side_length`：250 s + cube 1000 → NVTL 3.09，与 cube 300 逐项一致 —— 无辜
- 收集器的空间哈希碰撞：三张图都是 0.015%，与地图大小无关 —— 无辜
- "往返两遍造成重影"：单程长图（728 m）同样失败（1.88），**该假说被证伪**，至多是次要因素
- 地图太大撑不住：`dynamic_map_loading.map_radius = 150 m`，NDT 只用车周 150 m 建体素

**由此否定的方案**：
- ❌ Autoware 的 divided/tiled 地图 —— 所有瓦片共用一个 map 坐标系，仍是一次刚性变换
- ❌ 复用 VTR3 的 teach 位姿图 —— 实测 3933 顶点 / 3932 边全部是 `TEMPORAL` 且严格连续
  （`SPATIAL` 一次未出现），teach 阶段没有回环也没有位姿图优化，精度来源与自建图相同

**可行方向**：① GNSS 约束的 LIO（形变正是 GNSS 能钉住的，LIO-SAM 一类有现成 GPS 因子）；
② 带回环 + 位姿图优化的 SLAM（GLIM / SC-LIO-SAM）；③ 运营路段缩到约 300 m（已验证可用）。

## 8. 上车采集要求

**必录**

| 话题 | 类型 | 频率 |
|---|---|---|
| `/lidar/velodyne/front/cloud` | PointCloud2 | 10 Hz |
| `/lidar/velodyne/rear/cloud` | PointCloud2 | 10 Hz |
| `/imu/data` | Imu | 200 Hz |
| `/gps/fix` | NavSatFix | 5 Hz |
| `/can_twist_fb` | TwistStamped | 46 Hz |

建议加录 `/CameraFront` `/CameraRear`（感知用）、`/tf_static`。
只录必录集约 5 GB / 15 分钟；加两路相机约 38 GB。

**采集方式（比话题更重要）**

1. 开头**静止 ≥15 s**（FAST-LIO 要静止段估重力与零偏）
2. 闭环要真闭上：回到起点并与起点段**同向重叠 20–30 m**，最后静止 10 s
3. **采集机时钟对准 NTP/PTP** —— 上次整机偏 18.2 s
4. 尽量全程前进（上次去程是倒车）
5. 能开 RTK 就开（现在 `status=0`、σ≈2.9 m、约 30% 离群）
6. **雷达角度窗口保持现状**（每台约 160°，合计约 320°）
7. 车速约 1.3 m/s，转弯处慢一点

## 9. 验收判据

`converged_param_type: 1` + `converged_param_nearest_voxel_transformation_likelihood: 2.3`
是 Autoware 自己的收敛闸。因此：

- **NVTL 有样本但位姿 0 帧** = 每一帧都被判不收敛，是地图问题的明确信号
- 通过线：NVTL 中位 > 2.3 且低于阈值的比例接近 0、NDT 位姿接近 10 Hz 满速、迭代次数个位数
- ⚠ **"NDT 与 GNSS 的水平距离"不能当定位精度用** —— GNSS 本身约 30% 离群
  （配准 RANSAC 内点仅 864/1232）。要给精度需要更好的真值。
