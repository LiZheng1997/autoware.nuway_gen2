# nUWAy Autoware 1.9.0 — Orin Native Build Branch

This branch is based on upstream **tag `1.9.0`** (`10718787`, 2026-07-15), not `main`.
1.9.0 is a tag, not a branch, and **the tag name has no `v` prefix**.

Running alongside the Docker dual-machine setup in `uwa-rev/autoware_on_nUWAy`, this branch takes the
**Orin AGX single-machine native build** route: versions are pinned and reproducible, with no dependency on the rolling universe-devel image.

## Quick Start

```bash
git clone -b nuway/1.9.0 git@github.com:LiZheng1997/autoware.nuway_gen2.git
cd autoware.nuway_gen2

mkdir -p src
vcs import src < repositories/autoware.repos   # 32 upstream repos, all pinned to tag/hash
vcs import src < repositories/nuway.repos      # nuway_canbridge

bash nuway/stage_packages.sh                   # copy bundled sensor_kit / vehicle into src/
bash patches/apply.sh                          # ★ required, see below

sudo cp nuway/60-autoware-dds.conf /etc/sysctl.d/ && sudo sysctl --system
./setup-dev-env.sh universe -y --no-nvidia     # CUDA 12.8 must be installed separately, see docs
bash nuway/build.sh                            # ~2h23min, 488+7 packages
source nuway/setup_env.sh
```

## Why `patches/apply.sh` cannot be skipped

`cuda_blackboard` calls `cudaStreamGetDevice`, and this API **requires the CUDA 12.8 driver at runtime**,
but JetPack 6.x ships at most the 12.6 driver → the call returns `cudaErrorCallRequiresNewerDriver (36)`
→ `pointcloud_container` aborts immediately on startup → the entire point cloud pipeline disappears.

The downstream symptoms are highly misleading (`ndt_scan_matcher: No InputSource`,
`align server failed`) — they look like a sensor or map configuration error, but the root cause is a single CUDA call.

It's an **external dependency** pulled in by `vcs import`, and every import overwrites it,
so it has to be maintained as a patch and reapplied every time.

## This branch's delta relative to upstream

| Path | Contents |
|---|---|
| `repositories/nuway.repos` | nuway_canbridge (`canbridge_cpp@autoware`); URL changed from the `lee.github.com` alias to the standard form |
| `nuway_packages/nuway_sensor_kit_launch/` | Sensor kit (VLP-16 x2 / camera x2 / GNSS / IMU) |
| `nuway_packages/nuway_vehicle_launch/` | Vehicle description |
| `patches/` | One-line cuda_blackboard patch; canbridge's missing dependency declaration + apply script |
| `nuway/` | Environment variables, DDS sysctl, build script |
| `docs/build-on-orin.md` | Full SOP: 10 steps + 13 gotchas + troubleshooting index |

The nuway suite's original `common_sensor_launch` has been removed — 1.9.0's `autoware_launch` ships its own package of the same name (a newer version, 0.52.0 vs 0.50.0, the difference being an upstream refactor), and keeping both would abort colcon with a duplicate. The one nuway customization it carried (the distortion-correction threshold) is preserved as `patches/0002`.

`nuway_packages/` carries a `COLCON_IGNORE` to keep colcon from discovering
both it and the copy under `src/nuway/` and failing with a duplicate package name error. The two packages under it come from `uwa-rev/autoware_launch.nuway @ d11bea6 (2026-03-30)`,
**and are shipped with this repo rather than pulled via vcs import**: that upstream repo also contains a forked `autoware_launch`,
and importing it would collide with the package of the same name that ships with 1.9.0, aborting colcon with
`Duplicate package names not supported`. Splitting this out into its own repo is recommended going forward.

## What common_cuda_sensor_launch is

It's a zero-code, pure integration package (`ament_auto_package(INSTALL_TO_SHARE launch config)`);
all the algorithms come from TIER IV's upstream `autoware_cuda_pointcloud_preprocessor`.

The upstream README states the package's purpose plainly: "reimplement the crop / distortion-correction /
ring-outlier-filter of `autoware_pointcloud_preprocessor` on the GPU." So nuway commenting out the four CPU
nodes and swapping in a single CUDA node follows the upstream design rather than inventing something new.

What nuway adds on top of what upstream 1.9.0 doesn't yet provide is two things:

1. **Composing the CUDA node into the sensor container** (same process as the driver, zero-copy for multi-MB point clouds).
   Upstream only has a launch for the node as a standalone process, with the output topic defaulting to `test` — that's a smoke test, not a production wiring.
2. **The real vehicle's body crop box** (`crop_box.x∈[-0.5, 2.83]`, `y∈±0.748`, `z∈[0, 2.4]`,
   `negative: true`). Upstream's `crop_box.*` are all 0.

Removed files that are either byte-identical to upstream or never loaded by any launch:
`robosense_Bpearl/Helios.launch.xml`, `ring_outlier_filter_node.param.yaml`,
`distortion_corrector_node.param.yaml`. The smaller the diff surface, the easier future Autoware upgrades.

## RTK / NTRIP status

The `ntrip` package **has been removed from this branch** (see the commit that removed it — its commit message records the full implementation analysis).
Reason for removal: it doesn't build, isn't referenced by any launch file, and RTK isn't on the critical path —
Autoware localizes against the point cloud map with NDT, GNSS only supplies the initial pose, and `pose_initializer`'s
`pose_error_threshold` is 5 m, so meter-level GNSS accuracy is enough for NDT to converge (verified on hardware: `status=0`, σ≈2.9 m, initializes successfully on the first try).

If RTK is needed later, recover it with `git show <that commit>^:nuway_packages/nuway_sensor_kit_launch/ntrip/...`,
or reintroduce it from `uwa-rev/autoware_on_nUWAy`. Reviving it takes three steps:
`sudo apt install ros-humble-mavros-msgs ros-humble-rtcm-msgs` (both are available via apt, just not installed),
add `rtcm_msgs` to `package.xml` (it's already in CMakeLists but missing from package.xml),
and switch to a reachable caster.

⚠ **The original `ntrip-param.yaml` had a username and password committed in plaintext; it's in the git history now, and deleting the file doesn't erase it.
Those credentials must be rotated.** When restoring this, read them from environment variables instead — don't write them into the repo again.

## Known TODOs

- [ ] **Vehicle geometry needs verification**: the geometry fields in `vehicle_info.param.yaml` match Autoware's
      `sample_vehicle` (only `max_steer_angle` differs). `max_steer_angle: 0.70`
      has been confirmed by real-vehicle testing and is kept as-is. See `VEHICLE_LIMITS.md` for the raw CAN-side limits.
- [ ] **Point cloud format**: 1.9.0's `concatenate_and_time_sync_node` requires `PointXYZIRC`,
      but the existing recorded bags are `PointXYZIRT` (intensity float32 + ring + time). Need to confirm
      the actual driver's output format on the vehicle.
- [ ] **EZ10 steers on both front and rear axles**, while Autoware's default bicycle model assumes front-wheel-only steering, so the computed turning radius comes out conservative.
- [ ] The tuning results from `autoware_launch.nuway` (`nuway_preset.yaml`, etc.) haven't been migrated yet;
      they need to be compared item-by-item against 1.9.0's config.
