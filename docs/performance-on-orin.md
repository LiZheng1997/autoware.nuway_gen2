# Autoware 1.9.0 on the AGX Orin: what it costs

All numbers measured on 2026-10-03/04 with the campus bag replayed through the full
stack (`perception:=true planning:=true control:=true`), **RViz off** — that is the
configuration the vehicle will run. Machine: AGX Orin, 12 cores @ 2.2 GHz, MAXN,
61.4 GB RAM. Sampling: `tegrastats --interval 1000`, plus the timings Autoware
publishes about itself.

## Headline

| | idle | 1.0x, rmw_fastrtps_cpp | 1.0x, rmw_cyclonedds_cpp on `lo` |
|---|---|---|---|
| CPU total (12 cores = 1200%) | 4% | 991% = **9.9 cores** | 650% = **6.5 cores** |
| share of time each core is above 80% | 0 | 36–92% | **0–3%** |
| GPU (GR3D) median / p95 | 0 / 0% | 0 / 79% | 2 / 79% |
| RAM | 6.5 GB | 10.4 GB | 10.2 GB |
| VDD_CPU_CV | 1.2 W | 10.9 W | 8.0 W |
| `/planning/trajectory` | — | never produced | **10.06 Hz** |
| `/control/command/control_cmd` | — | never produced | **67.05 Hz** |
| trajectories dropped as late | — | constant | **0** |

Nothing but the DDS implementation and interface binding differs between the last two
columns: same map, same replay window, same goal, no RViz either way.

**The Orin runs the complete stack in real time with 46% CPU to spare.** An earlier
revision of this document concluded the opposite; that conclusion was drawn before the
DDS configuration was examined and is wrong.

## Why the default configuration was so expensive

`RMW_IMPLEMENTATION` was never set, so Humble fell back to `rmw_fastrtps_cpp` while
Autoware is tuned for CycloneDDS (`rmw_cyclonedds_cpp` was installed all along). The
Orin has **13 network interfaces** — `lo wlP1p1s0 can0 can1 eth0 l4tbr0 usb0 usb1
docker0 flannel.1 cni0` and two `veth` — and with `ROS_LOCALHOST_ONLY=0` the ~68 DDS
participants of one launch discovered each other across every one of them. The run left
**790 shared-memory segments** in `/dev/shm`, and leaked 588 of them after the processes
exited.

Fixed in `nuway/setup_env.sh` + `nuway/cyclonedds.xml`. `NUWAY_DDS_IFACE` selects the
interface (default `lo`); point it at the NIC that reaches MainPC if topics have to
leave the box. Naming one interface is the part that matters.

## Where the CPU goes

`nuway/tools/cpu_attrib.py` samples `/proc/<pid>/stat` twice and maps each process to a
subsystem through its `__ns:=` argument. At 1.0x:

| subsystem | procs | fastrtps | cyclonedds | change |
|---|---|---|---|---|
| planning | 13 | 175.0% | 42.1% | −76% |
| localization | 10 | 169.2% | 72.7% | −57% |
| `pointcloud_container` etc. | 4 | 135.0% | 73.2% | −46% |
| sensing | 6 | 114.3% | 27.0% | −76% |
| system (diagnostics, state monitors) | 12 | 114.0% | 47.8% | −58% |
| replay harness (**not present on the vehicle**) | 2 | 94.5% | 83.5% | −12% |
| control | 3 | 67.7% | 24.2% | −64% |
| perception | 4 | 46.6% | 13.6% | −71% |
| api / map / launch | 6 | 33.7% | 17.7% | −47% |

perception looks small because the point-cloud work sits in `pointcloud_container`
(RSS 2.1 GB) and CenterPoint's inference runs on the GPU.

Still available to cut, if more headroom is ever needed:
`parking_container` 26.2% (campus drives lanes; freespace parking is never used) and
the three evaluator nodes, about 35% combined (pure metrics).

## Algorithms were never the problem

At 1.0x every per-frame node kept full rate with no dropped frames:

| node | rate | median | p95 |
|---|---|---|---|
| `lidar_centerpoint` | 9.5 Hz | 21.9 ms | 32.8 ms |
| `scan_ground_filter` | 9.5 Hz | 17.7 ms | 31.8 ms |
| `voxel_grid_based_euclidean_cluster` | 9.5 Hz | 4.7 ms | 11.2 ms |
| `pointcloud_based_occupancy_grid_map` | 9.5 Hz | 3.1 ms | 9.8 ms |
| `voxel_based_compare_map_filter` | 9.5 Hz | 3.3 ms | 9.1 ms |
| `pose_twist_fusion_filter` (EKF) | 38.9 Hz | 0.8 ms | 18.0 ms |

GPU sits at a median of 2% and RAM at 10 of 61 GB. CPU was the only constrained
resource, and most of what it was spending went to DDS rather than to any algorithm.

## Reproducing

```bash
source nuway/setup_env.sh
nuway/replay/fullchain.sh 1.0 640 40          # whole chain, prints every stage's rate
python3 nuway/tools/cpu_attrib.py 60          # CPU by subsystem, run while the above is up
python3 nuway/tools/perf_probe.py 120         # per-node processing times
timeout 120 tegrastats --interval 1000 > t.log && python3 nuway/tools/tegra_stats.py t.log
```

## Measurement traps hit along the way

- `/planning/scenario_planning/trajectory` **does not exist** in 1.9.0. The final output
  is `/planning/trajectory`. Three separate measurements reported "no trajectory" purely
  because of this.
- Route topics are `transient_local` latched, so `ros2 topic hz` reports nothing for them
  however healthy the chain is. `nuway/tools/chain_probe.py` discovers topics by message
  type and subscribes with both durabilities for this reason.
- `pgrep -f <name>` matches the grep command's own command line. Use
  `ps -eo pid,etimes,comm` when checking for leftovers.
- The Orin's `:0` auto-locks and screen recordings come out black (29.5 kB files).
  `xset s off -dpms` is not enough; disable the GNOME screensaver lock as well.
