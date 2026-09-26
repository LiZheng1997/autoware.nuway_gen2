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
| `docs/mapping-and-localization.md` | Mapping and NDT localization: the toolchain, the measurements, and what was ruled out |
| `docs/vehicle-bringup.md` | **On-vehicle bring-up order**, per-vehicle sensor differences, and what to check at each stage |
| `nuway/vehicle_preflight.sh` | Readiness check on the vehicle (read-only unless `--apply-net`) |

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

## RTK / NTRIP

The `ntrip` package is built and wired for **AUSCORS**, the free national CORS
service run by Geoscience Australia. Register once at
<https://gnss.ga.gov.au/registration>; approval is automatic and takes minutes.

Mountpoints nearest UWA Crawley, measured from the AUSCORS source table:

| Mountpoint | Distance | Observations | Constellations | Ephemerides | Rate |
|---|---|---|---|---|---|
| `CUT000AUS0` | 7.3 km | MSM7 | GPS, GLO, GAL, QZS, BDS | 1019, 1020 | 12 kbit/s |
| `SALT00AUS0` | 6.9 km | MSM4 | GPS, GLO, QZS, BDS | none | 3.9 kbit/s |
| `PERT00AUS0` | 21.0 km | not measured | | | |

Those figures come from pulling 15 s off each mount, not from the source table.
`CUT000AUS0` is the default even though it is 400 m further: MSM7 rather than
MSM4, Galileo actually present where Salter Point advertises it but does not
broadcast it, and embedded GPS and GLONASS ephemerides so a cold-started
receiver need not wait on the satellites' own navigation message before fixing.
Both are single-base mounts needing no GGA feedback, and a 7 km baseline sits
comfortably inside the 20-30 km where single-base RTK holds 2-3 cm.

Credentials are read from the environment, never committed:

```bash
export NTRIP_USER='<your AUSCORS username>'
read -s NTRIP_PASS && export NTRIP_PASS
ros2 launch ntrip ntrip_launch.py
```

`NTRIP_HOST` and `NTRIP_MOUNTPOINT` override the defaults the same way.

On the SBG vehicle nothing else is needed: `sbg_params.yaml` already sets
`rtcm.subscribe: true` and `nmea.publish: true`, and the topic names line up with
this client, so corrections reach the receiver and GGA flows back out.

A NovAtel vehicle needs a different arrangement. Its ROS driver publishes
`/gps/fix` but exposes no RTCM input - inspecting `libnovatel_oem7_driver.so`
shows subscriptions only to its own messages - so run the receiver's built-in
NTRIP client instead (`NTRIPCONFIG` over an NCOM port), or pipe RTCM into its
serial port. Note also that a single antenna gives no INS heading, so
`use_gnss_ins_orientation` must stay false and `gnss_poser` derives heading from
successive positions.

RTK remains optional for localization: NDT matches against the pointcloud map and
GNSS only supplies the initial pose, where `pose_initializer` accepts a 5 m error.
An unaugmented fix - status 0, sigma about 2.9 m - already initialises on the
first attempt.

Two defects had to be fixed before this package would build, both of them
mis-declared dependencies rather than anything to do with the caster.
`mavros_msgs` was declared and included but never used, since the publisher is
`rtcm_msgs::msg::Message`; `rtcm_msgs` was named in CMakeLists but missing from
`package.xml`, so rosdep never installed it. An earlier note here blamed apt for
not carrying `mavros_msgs`, which was wrong on both counts.

⚠ An early revision of `config/ntrip-param.yaml` committed a real username and
password, and they remain in commit `2d3e8c9` - the first commit on this branch,
and the only one that carries them. Deleting the file did not reach them.

They were credentials for the `UWA_Campus` mountpoint on a caster that no longer
exists, so there is no service left on which to change them. History was
deliberately not rewritten: it would have renumbered twelve commits, force-pushed
over published history, and still left the old objects reachable by SHA for a
while. The exposure that remains is an email address and a password that must not
be reused anywhere else.

Credentials no longer enter the repository at all - `ntrip_launch.py` reads them
from the environment.

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
