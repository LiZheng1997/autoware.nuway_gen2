# Pointcloud mapping and NDT localization for nUWAy

How to turn a rosbag into a pointcloud map that Autoware's NDT accepts, on the Orin,
and how to prove it works.

**Measured outcome (2026-09-24/25).** A 320 m map **passes acceptance**: median NVTL 3.11,
99.9 % of frames above the 2.3 gate, NDT pose at the full 9.9 Hz, and GNSS without RTK
initialises on the first attempt. Maps of 728 m and 1100 m **do not work**; the cause is
non-rigid warp in the LIO trajectory, which no parameter fixes (section 7).

> **About paths.** The scripts here hard-code absolute paths from the machine they were
> developed on (`/home/lz/campus_data/...`, `/home/lz/fast_lio_ws`, `/home/lz/VTR-Demo`,
> `/home/lz/aw_verify`). Adjust them per machine, or shadow them with variables at the top.

---

## 1. Dependencies

| Component | Location | Role |
|---|---|---|
| spark-fast-lio | `~/fast_lio_ws` | LIO odometry and registered clouds |
| Dual-lidar merger | `~/VTR-Demo/scripts/merge_velodyne_clouds.py` | **Do not rewrite it** — it already handles per-point time correctly |
| TF tree | `~/ASRL/vtr3/src/config/nuway_vtr_tf.urdf.xml` | Vehicle TF during mapping |
| Autoware 1.9.0 | `~/aw_verify` | Localization acceptance |

## 2. Why both lidars are mandatory

Each recorded frame holds about 12,500 points spanning 44–48 ms — that is roughly a
**160 degree sector, not a full sweep**. One front lidar therefore gives a single 160 degree
sector, and pitch is barely observable:

| | Front lidar only | Both lidars merged |
|---|---|---|
| Ground tilt along the route | 2.0° → 5.1° (accumulating) | 0.20° → 1.77° (essentially constant) |
| Ground-fit residual | 1.77 m | 1.14 m |
| Median NDT NVTL | **0.00** (never converges) | **3.09** |

Front and rear together cover about 320 degrees, and that is what makes the approach work.
**Do not change the lidars' angular windows when recording on the vehicle.**

The two lidars are also **37 ms out of sync**. `merge_velodyne_clouds.py` handles this by
normalising each point's timestamp to the start of the merged scan and sorting by time,
which is what lets FAST-LIO de-skew correctly across both sensors.

## 3. Building a map

```bash
bash nuway/mapping/build_map_dual.sh 250      # duration in seconds
```

It starts, in order: the TF tree, the merger, FAST-LIO (`mapping_nuway_dual.launch.yaml`),
the accumulator (`collect_map.py`) and trajectory logger (`record_traj.py`), then replays
**both** lidar topics. Output: `campus_dual.pcd` (0.2 m voxels), `odo.csv`, `gps.csv`.

`build_map_full.sh` is the long-route variant. It swaps in `nuway_campus_full.yaml`, which
raises `cube_side_length` from 300 to 1000 — at 300 the ikd-tree evicts map points and the
map is **truncated part-way** through any route longer than about 150 m. Raising it has a
useful side effect: when the vehicle turns back, the outbound map points are still resident,
so scan-to-map ICP registers the return leg against them. That acts as an implicit loop
closure and cut the vertical loop residual from 5.49 m to 0.07 m.

> ⚠ **Do not enable `gravity_alignment.enable_gravity_alignment`.** Its `isMotionStopped()`
> test treats the platform as stationary whenever `‖acc_ref − acc_curr‖ ≤ 0.2`, and nUWAy
> cruising at 1.25 m/s never exceeds that, so the counter resets forever and the node
> publishes nothing at all. Observed directly: alignment never completed and the accumulator
> received zero frames. Gravity alignment is done afterwards instead, by levelling the
> finished map (section 4).

## 4. Inspection and georeferencing

```bash
python3 nuway/mapping/check_map.py <pcd> 6 <odo.csv>     # odo.csv is required for curved routes
python3 nuway/mapping/georef_map.py <odo.csv> <gps.csv> <pcd> <out_dir> 2.4
```

`check_map.py` fits the ground plane in segments along **trajectory arc length** and reports
whether the tilt is constant (a single rotation can level it) or accumulating. Binning by a
coordinate axis instead puts parts of an L-shaped route at different elevations into the same
bin and produces nonsense such as a fitted ground height of −30.58 m.

`georef_map.py` does four things: (1) one SE(3) levelling rotation taking the map's ground
normal to +z — **valid only when the tilt is constant**; (2) a RANSAC + Kabsch SE(2) fit of
the LIO trajectory to GNSS; (3) a translation putting the ground at z = 0; (4) it writes
`map_projector_info.yaml` (`LocalCartesianUTM`, altitude = median GNSS altitude minus the
2.4 m antenna height). The output directory is what you pass to Autoware as `map_path`; it
holds `pointcloud_map.pcd`, `map_projector_info.yaml` and a placeholder `lanelet2_map.osm`
(NDT needs no vector map, but map_loader loads one; planning needs a hand-drawn lane network).

`check_drift.py` measures accumulated drift from out-and-back revisits: it uses GNSS to pair
each point on the return leg with the same physical location on the outbound leg, then
compares the odometry between them — **no ground truth and no loop-closure software needed**.
Note that its horizontal column also contains the lateral offset between the two passes and
the GNSS pairing error (pairing distance reaches 8 m), so judge horizontal drift from the
start-to-end loop residual; only the z column is clean.

## 5. Replay and acceptance

```bash
bash nuway/replay/ndt_verify.sh <map_dir> <out_dir> <playback_start_offset_s> <collect_s>
```

`bag_to_autoware.py` is a **replay-only adapter**; the vehicle does not need it. It supplies
three things:

1. **A common time base.** Every topic's header stamps run about **18.2 s ahead** of the bag's
   record times — the recording machine's clock and the sensors' clock were not synchronised,
   though the sensors agree with each other. Since `--clock` publishes record time, NDT's 1 s
   tolerance is violated on every frame. The adapter calibrates that offset and shifts each
   forwarded message onto the `/clock` base. Calibration is **gated on stability** (6 s warm-up,
   then the last 40 samples must span less than 0.1 s): a replay started with `--start-offset`
   delivers a burst of backlogged messages first, and sampling the first N messages reads a
   transient that is off by nearly 30 s.
2. **The point type.** The bag was recorded by the old velodyne driver as `PointXYZIRT` (22 B);
   the 1.9.0 preprocessing chain expects `PointXYZIRC` (16 B). On the vehicle the nebula driver
   emits `PointXYZIRCAEDT` natively, so this does not arise.
3. **Vehicle speed.** The bag carries speed on `/can_twist_fb`, while Autoware expects
   `/vehicle/status/velocity_status` (`VelocityReport`). Without it gyro_odometer has no twist
   and the EKF free-runs.

Frame names also have to be reconciled — the bag uses `lidar_velodyne_front/rear`, the sensor
kit uses `velodyne_front/rear_link` — while the extrinsics always come from the vehicle
calibration in `nuway_sensor_kit_description`.

> ⚠ **Do not collect metrics with `ros2 topic echo`.** Once `ros2 daemon` is killed as
> collateral of `kill -KILL -<PGID>`, every subsequent echo dies on startup with
> `xmlrpc Fault: !rclpy.ok()`, the capture files contain nothing but tracebacks, and the run
> reports a bogus "NVTL = 0". Use `collect_ndt.py`, which subscribes through rclpy directly.

## 6. Visualisation

```bash
bash nuway/replay/viz_localization.sh <map_dir> 0.5 0     # localization stack on the Orin; 0 = no local rviz
# From another machine on the same network (cross-host DDS discovery works):
rviz2 -d nuway/replay/nuway_localization.rviz --ros-args -p use_sim_time:=true
```

**`use_sim_time:=true` is not optional.** Playback carries the timestamps recorded in the bag;
RViz defaults to wall time, the two are a year apart, and every TF lookup fails leaving a blank
view. The config uses no Autoware-specific rviz plugins, so a stock `rviz2` opens it.

## 7. Why long maps fail

The same 728 m map, the same sensor data, **only the global rigid transform changed**, measured
at the start of the route:

| | Global levelling + SE(2) | Fitted to the start region only | (reference) 250 s map |
|---|---|---|---|
| Median NVTL | 2.08 | **3.00** | 3.09 |
| Median iterations | 25 | 3 | 2 |
| NDT pose frames | 148 | 628 | 1170 |

Not one map point moved and NVTL recovered, so **no single rigid transform can serve the whole
map**: every region wants a different one (global and local fits differ by 3.4° in yaw). That
is what non-rigid warp means.

**Alternative explanations ruled out by measurement** — do not re-investigate these:

- `cube_side_length`: a 250 s map built with cube 1000 gives NVTL 3.09, matching cube 300 on
  every metric. Not the cause.
- Voxel hash collisions in the accumulator: 0.015 % on all three maps, independent of map size.
  Not the cause.
- Ghosting from driving the corridor twice: the single-pass 728 m map fails just as badly
  (1.88). **This hypothesis was falsified**; at most it is a secondary effect.
- The map simply being too large: `dynamic_map_loading.map_radius` is 150 m, so NDT only
  voxelises the map within 150 m of the vehicle.

**Remedies this rules out:**

- ❌ Autoware's divided/tiled pointcloud map — all tiles share one map frame, so it is still a
  single rigid transform.
- ❌ Reusing the VTR3 teach pose graph — its 3933 vertices and 3932 edges are **all `TEMPORAL`
  and strictly sequential** (`SPATIAL` never appears), so teach performs neither loop closure
  nor pose-graph optimisation and offers no better accuracy than mapping the bag directly.

**Directions that remain:** (1) GNSS-constrained LIO — warp is exactly what GNSS pins down, and
LIO-SAM and similar already carry a GPS factor; (2) SLAM with loop closure and pose-graph
optimisation (GLIM, SC-LIO-SAM); (3) keep the operating route near 300 m, which is verified.

## 8. What to record on the vehicle

**Required**

| Topic | Type | Rate |
|---|---|---|
| `/lidar/velodyne/front/cloud` | PointCloud2 | 10 Hz |
| `/lidar/velodyne/rear/cloud` | PointCloud2 | 10 Hz |
| `/imu/data` | Imu | 200 Hz |
| `/gps/fix` | NavSatFix | 5 Hz |
| `/can_twist_fb` | TwistStamped | 46 Hz |

Also worth recording: `/CameraFront`, `/CameraRear` (perception) and `/tf_static` if published.
The required set alone is about 5 GB per 15 minutes; adding both cameras takes it to about 38 GB.

**How to drive the recording — this matters more than the topic list**

1. Stay **stationary for at least 15 s** at the start; FAST-LIO needs it to estimate gravity
   and the IMU biases.
2. Close the loop for real: return to the start and **overlap the first 20–30 m in the same
   direction of travel**, then stand still for 10 s. Merely ending up nearby is not enough —
   loop closure needs shared observations.
3. **Synchronise the recording machine's clock** (NTP or PTP). The existing bag is off by 18.2 s.
4. Drive forward throughout if possible; the existing bag reverses on the outbound leg.
5. Use RTK if it can be arranged. Today's fixes are `status=0`, σ ≈ 2.9 m, with roughly 30 %
   outliers.
6. **Leave the lidar angular windows as they are** (about 160° each, about 320° combined).
7. Around 1.3 m/s, slower through turns.

## 9. Acceptance criteria

Autoware's own convergence gate is `converged_param_type: 1` together with
`converged_param_nearest_voxel_transformation_likelihood: 2.3`. Therefore:

- **NVTL samples present but zero pose frames** means Autoware rejected every single frame —
  an unambiguous sign that the map, not the measurement tooling, is at fault.
- Passing looks like: median NVTL above 2.3 with almost no frames below it, NDT pose at close to
  the full 10 Hz, and single-digit iteration counts.
- ⚠ **Do not read "distance between NDT and GNSS" as localization accuracy.** GNSS itself is
  roughly 30 % outliers (the georeferencing RANSAC keeps only 864 of 1232 points). Quoting an
  accuracy figure needs better ground truth.
