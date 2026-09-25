# Native Build of Autoware 1.9 on Orin

Building Autoware 1.9.0 from source on an AGX Orin Developer Kit, without Docker.

**Verified result**: 488/488 packages built successfully in 2h 23min, zero failures (2026-09-22).

---

## 1. Applicable Environment

| Item | Version verified in this document |
|---|---|
| Hardware | Jetson AGX Orin Developer Kit (p3737-0000 + p3701-0005, 64 GB) |
| OS | JetPack 6.2 / L4T **R36.4.3** / kernel 5.15.148-tegra |
| JetPack component | nvidia-jetpack **6.2+b77** |
| Driver-side CUDA | 12.6 (bundled with JetPack, **do not touch**) |
| Build-side CUDA | **12.8** (installed separately, see Step 2) |
| TensorRT | 10.3.0.30 (bundled with JetPack) |
| cuDNN | 9.3.0.75 (bundled with JetPack) |
| GCC | **11.4.0** |
| Autoware | tag **1.9.0** (488 packages) |
| Disk | reserve at least 20 GB during the build (install 351 MB + build 4.3 GB + dependencies) |

---

## 2. Environment Requirements and Rationale

These four prerequisites **must be satisfied first**, or the build is guaranteed to fail. Their order cannot be swapped — each one only becomes visible once the previous one is resolved.

### GCC must be version 11

All of ROS 2 Humble's apt binary packages are built with GCC 11. If the system's default compiler is not 11:

- GCC 9 falsely reports `maybe-uninitialized` for `std::optional` + `return {};`, which combined with Autoware's `-Werror` turns straight into a hard build error
- Binaries built with GCC 9 carry ABI risk against ROS libraries built with GCC 11

```bash
gcc --version    # must be 11.x
```

If it isn't, see Step 1. **Note** that Ubuntu 22.04 ships CMake 3.22, so the route of disabling `-Werror` via `CMAKE_COMPILE_WARNING_AS_ERROR` (which needs ≥3.24) is not available.

### Build-side CUDA must be 12.8

Autoware 1.9.0 states this explicitly in `ansible/roles/cuda/defaults/main.yaml`:

```yaml
# CUDA 12.8 on Ubuntu 22.04 (humble / Jetson Orin via JetPack 6).
cuda_version: "{{ '13.0' if ansible_distribution_version == '24.04' else '12.8' }}"
cuda_repo_distro: "{{ 'ubuntu2404' if ... else 'ubuntu2204' }}"
```

**The entire JetPack 6.x line only ships CUDA 12.6** (verified: in the r36.4 repository, all three nvidia-cuda versions — 6.1+b123 / 6.2+b77 / 6.2.1+b38 — depend on `cuda-12-6`). So bumping the L4T point release will never get you 12.8; the toolchain has to be installed separately from NVIDIA's `ubuntu2204` CUDA repository.

The difference is concrete: `cuda_blackboard` 0.4.0 uses `cudaStreamGetDevice()`, a function that does not exist in the headers of either CUDA 12.6 or 12.2.

> ### ⚠️ But installing 12.8 only gets you a **build that compiles** — the one-line patch in Section 7 must be applied as well
>
> `cudaStreamGetDevice` needs not only the 12.8 **headers** but also the 12.8 **driver** at runtime. And **the newest driver JetPack 6.x ships is CUDA 12.6 — there is no way to get a 12.8 driver**. Measured:
>
> ```
> runtime=12080  driver=12060
> cudaStreamGetDevice  -> 36 (cudaErrorCallRequiresNewerDriver)   dev=-1
> cudaGetDevice        ->  0 (cudaSuccess)                        dev=0
> ```
>
> The result is that all 488 packages build successfully, yet `pointcloud_container` **aborts on startup**, taking out the entire point cloud preprocessing/concatenation chain. See Section 5, "`cuda_blackboard` must be patched on JetPack", for details.
>
> What's more, `cudaStreamGetDevice` appears **exactly once across all 488 packages** (that one line in `cuda_blackboard`). In other words, once that patch is applied, the entire CUDA 12.8 SBSA detour is quite possibly unnecessary and JetPack's bundled 12.6 would do (confirmed by source inspection, but not re-verified with a full rebuild).

### Keep driver-side CUDA / TensorRT / cuDNN as shipped by JetPack

Install only the CUDA **toolchain**, never `cuda-drivers` — installing the desktop driver package on a Jetson breaks the Tegra graphics and GPU stack. That is exactly why `setup-dev-env.sh` has a `--no-cuda-drivers` option.

For TensorRT, ansible pins `10.3.0.26-1+cuda12.5` by default, while JetPack 6.2 ships `10.3.0.30-1+cuda12.5`. **There's no need to force them to match**: both are TRT 10.3, and it's the JetPack version that is actually built for Orin's integrated GPU. Before relying on it, it's worth test-building an engine once:

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=$HOME/autoware_data/lidar_centerpoint/pts_voxel_encoder_centerpoint.onnx \
  --saveEngine=/tmp/t.plan --skipInference
# expected: "Engine built in ... sec" + "&&&& PASSED"
```

### The build tree must be cleaned after switching compiler or CUDA

`build/*/CMakeCache.txt` caches the CUDA version and compiler paths. If it isn't cleared after switching environments, you get self-contradictory errors like "`/usr/local/cuda` is clearly 12.8, yet it reports Found unsuitable version 12.6."

---

## 3. Steps

### Step 0: Preflight checks

```bash
cat /proc/device-tree/model            # should read Jetson AGX Orin Developer Kit
head -1 /etc/nv_tegra_release          # R36 REVISION: 4.3
nproc; free -g; df -h /                # 12 cores / 61 G / reserve >=20 G
sudo nvpmodel -q | tail -2             # MAXN recommended
docker ps                              # should be empty: containers competing for CPU will slow down the build
```

### Step 1: Switch to GCC 11

`gcc` and `g++` are independent alternatives; they cannot be linked together with `--slave`.

```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 110
sudo update-alternatives --set gcc /usr/bin/gcc-11
sudo update-alternatives --set g++ /usr/bin/g++-11

gcc --version && g++ --version && cc --version && c++ --version   # all four must be 11.x
```

### Step 2: Install the CUDA 12.8 toolchain

```bash
# Add the NVIDIA CUDA repo (aarch64 uses the sbsa path)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/sbsa/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# Dry-run first to confirm the plan contains no driver package
sudo apt-get -s install cuda-toolkit-12-8 | grep -E "cuda-drivers|nvidia-driver|nvidia-kernel"
#   ↑ must produce no output

sudo apt-get install -y cuda-toolkit-12-8
sudo update-alternatives --set cuda /usr/local/cuda-12.8

# Verify
readlink -f /usr/local/cuda                                   # /usr/local/cuda-12.8
/usr/local/cuda/bin/nvcc --version | tail -2 | head -1        # release 12.8
grep -rl cudaStreamGetDevice /usr/local/cuda-12.8/include/    # must hit cuda_runtime_api.h
dpkg -l | grep -E "^ii  (cuda-drivers|nvidia-driver)"         # must produce no output
```

### Step 3: Fetch the source

```bash
git clone https://github.com/autowarefoundation/autoware.git ~/autoware
cd ~/autoware
git fetch --tags
git checkout 1.9.0
mkdir src
vcs import src < repositories/autoware.repos      # 32 repositories

find src -name package.xml | wc -l                # expect 488
```

> **Don't leave an old checkout next to `src`.** Any directory inside the workspace that contains packages with the same names (e.g. an old `src` renamed to `src.bak`) will make colcon abort immediately with
> `ERROR:colcon:colcon build: Duplicate package names not supported`, **compiling not a single package**. Move old checkouts out of the workspace,
> or drop a `COLCON_IGNORE` file inside them.

### Step 4: Install the dependency environment

```bash
cd ~/autoware
./setup-dev-env.sh universe -y --no-nvidia
```

`--no-nvidia` skips the `cuda` and `tensorrt` roles, protecting the toolchain installed in Step 2 and the TRT/cuDNN that JetPack already ships.

If you only want to skip the driver and let ansible install CUDA/TRT itself, use `--no-cuda-drivers` instead.

**Success criterion**: `failed=0` at the end.

> Under the hood, `--no-nvidia` is `--extra-vars prompt_install_nvidia=n`. ansible **prints the `TASK [...]` header for skipped tasks too** —
> only the next line, `skipping: [localhost]`, tells you what actually happened. Don't treat a task header as proof that it ran.

Then rosdep:

```bash
source /opt/ros/humble/setup.bash
rosdep update --include-eol-distros
rosdep install -y --from-paths src --ignore-src --rosdistro humble
# success criterion: "#All required rosdeps installed successfully"
```

### Step 5: Build

```bash
cd ~/autoware
source /opt/ros/humble/setup.bash
export CC=/usr/bin/gcc-11 CXX=/usr/bin/g++-11
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=/usr/local/cuda-12.8/bin:$PATH
export MAKEFLAGS=-j3

colcon build --symlink-install --continue-on-error --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DCMAKE_CUDA_ARCHITECTURES=87 \
               -DCMAKE_C_COMPILER=/usr/bin/gcc-11 \
               -DCMAKE_CXX_COMPILER=/usr/bin/g++-11 \
               -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc
```

Parameter choices:

- **`--parallel-workers 4` + `MAKEFLAGS=-j3`**: running all 12 cores flat out triggers OOM. A single CUDA compilation unit in `tensorrt_yolox`, `lidar_centerpoint`, or `bevdet_vendor` can consume several GB, and having the build interrupted mid-way by the OOM killer costs far more than running a bit slower. Measured peak available memory stayed above 50 GB throughout.
- **`-DCMAKE_CUDA_ARCHITECTURES=87`**: Orin is SM 87; generating code for only this one architecture saves a considerable amount of time.
- **`--continue-on-error`**: what a first build needs is **the complete list of failures in one pass**, not to get stuck on the first one.

**Check once within the first 2 minutes after kicking off the build**, and look for positive evidence of progress — not just "the log hasn't shown an error yet":

```bash
grep -c "Starting >>>" log/latest_build/../../colcon_build.log   # should be growing
grep -c "Finished <<<" colcon_build.log
grep -cE "^ERROR:colcon" colcon_build.log                        # should be 0
```

For a long build, attach a heartbeat check and **alert after two consecutive checks with no progress** — a stalled build should not depend on a human staring at the terminal to notice.

### Step 6: Verify

```bash
grep "^Summary:" colcon_build.log     # Summary: 488 packages finished [2h 23min 17s]
grep -c "^Failed   <<<" colcon_build.log   # 0

source install/setup.bash
ros2 pkg list | wc -l                  # 849

# The CUDA build artifact is not an empty shell: follow the symlink to the real file and confirm all dependencies resolve
L=$(readlink -f install/autoware_lidar_centerpoint/lib/libautoware_lidar_centerpoint_cuda_lib.so)
ls -lL "$L"                            # about 3.9 MB
ldd "$L" | grep -c "not found"         # 0
```

> Under `--symlink-install`, the `.so` files in `install/` are symlinks, so `ls -l` only shows a link length of around 92 bytes.
> You must `readlink -f` to follow through to the real file before judging its size, otherwise you'll wrongly conclude "the build produced an empty file."

### Step 7: Make Autoware find CUDA 12.8 at runtime (**a successful build ≠ a working run**)

Skip this step and all 488 packages will still build successfully, but **15 CUDA nodes fail to load the moment you start**:

```
[ERROR] Failed to load library: Could not load library dlopen error:
  libcudart.so.12: cannot open shared object file: No such file or directory
  → traffic_light_fine_detector / car_traffic_light_classifier
  → pedestrian_traffic_light_classifier / lidar_centerpoint ...
```

The reason is that **the two CUDA installations use different directory layouts**, and the dynamic linker's cache doesn't honor the build-time `-L` flag:

| Source | Library directory layout |
|---|---|
| JetPack's CUDA (Tegra package) | `/usr/local/cuda-12.6/targets/**aarch64-linux**/lib` |
| Separately installed CUDA 12.8 (**SBSA package**) | `/usr/local/cuda-12.8/targets/**sbsa-linux**/lib` |

After switching `/usr/local/cuda` to 12.8, there's a leftover beyond `/etc/apt`: `/etc/ld.so.conf.d/000_cuda.conf` (the `000` filename prefix gives it top scan priority) still points at `/usr/local/cuda/targets/aarch64-linux/lib` — a directory that **no longer exists** under 12.8. Since the ld cache was generated before the switch, `libcudart.so.12` in the cache still points to a **dangling path**, and `dlopen` is bound to fail.

**Refresh the cache first, then see which version it resolves to:**

```bash
sudo ldconfig
ldconfig -p | grep 'libcudart.so.12 '
```

In this document's environment, after refreshing, the soname landed on **12.2** (because the directory referenced by `gds-12-2.conf` still exists). **That's not enough** —

```bash
# only 12.8 provides the cudaStreamGetDevice that cuda_blackboard needs
for v in 12.2/targets/aarch64-linux 12.6/targets/aarch64-linux 12.8/targets/sbsa-linux; do
  printf 'cuda-%-30s ' "$v"
  nm -D --defined-only /usr/local/cuda-$v/lib/libcudart.so.12 \
    | grep -qw cudaStreamGetDevice && echo yes || echo no
done
# cuda-12.2/...  no
# cuda-12.6/...  no
# cuda-12.8/...  yes
```

**Do not make 12.8 the system-wide default.** The Jetson should keep JetPack's own CUDA as the system default (camera, multimedia, and DeepStream all depend on it). Just add one path to Autoware's runtime environment:

```bash
# place alongside `source install/setup.bash` (add to ~/.bashrc or a launch script)
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/targets/sbsa-linux/lib:${LD_LIBRARY_PATH}
```

**Verification criterion** (measured: 15 → 0):

```bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-rosbag \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit rviz:=false > launch.log 2>&1

grep -c 'cannot open shared object file' launch.log    # must be 0
```

### Step 8: Configure DDS (`setup-dev-env.sh` **will not** do this for you)

A single Autoware launch brings up **140 nodes**, with each point cloud frame several MB, all traveling over the UDP loopback. Ubuntu's default receive buffer is only 208 KB, so packet loss is inevitable — and `setup-dev-env.sh` never lays down any DDS-related sysctl settings (`rmem_max` is nowhere to be found under `/etc/sysctl.d/`).

The symptom is very misleading, because it **produces no errors, it just silently drops data** — and unevenly at that:

```
# Same bag, three lidars, same measurement window
/sensing/lidar/right/velodyne_packets   5.972       ← has data
/sensing/lidar/top/velodyne_packets     no data      ← main lidar, missing
/sensing/lidar/left/velodyne_packets    no data

# downstream then reports a cascade of timeouts
[ERROR] [localization.twist_estimator.gyro_odometer]: IMU msg is timeout.
[ERROR] [localization.twist_estimator.gyro_odometer]: Vehicle twist msg is timeout.
```

It's easy to misdiagnose this as "the compiled packages are broken" or "the sensor configuration doesn't match." **Check the buffer size before you suspect the code.**

```bash
sysctl -n net.core.rmem_max      # default 212992 -- nowhere near enough
```

**Fix** (the officially required values):

```bash
sudo tee /etc/sysctl.d/60-autoware-dds.conf >/dev/null <<'EOF'
net.core.rmem_max=2147483647
net.core.wmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sudo sysctl --system
```

**Keep the default RMW, FastDDS.** A common suggestion online is to "switch to CycloneDDS" — testing in this environment shows **don't just copy that advice**: after writing a `cyclonedds.xml` that restricts `<Interfaces>` to `lo` and sets `SocketReceiveBufferSize min="10MB"`, then setting `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, **almost every node SIGABRTs during initialization**:

```
[ERROR] [autoware_ndt_scan_matcher_node-26]: process has died [pid ..., exit code -6, ...]
[ERROR] [autoware_ekf_localizer_node-30]:   process has died [pid ..., exit code -6, ...]
[ERROR] [imu_corrector_node-22]:            process has died [pid ..., exit code -6, ...]
(a dozen-plus other nodes fail the same way with exit code -6)
```

Reverting to FastDDS restores normal operation. Switching RMW is a separate tuning topic — **verify it independently, don't mix it into this SOP's steps** — otherwise the SIGABRTs get wrongly blamed on the build artifacts. What actually fixes the data loss is the four sysctl settings above.

> **While diagnosing, also shut down the ros2 CLI daemon.** `ros2 topic hz` / `node list` go through `ros2-daemon` by default, which caches the topology graph; a daemon left over from a previous launch will show you a stale graph in the new run, manifesting as "the topic is clearly publishing but hz can't find it." Always add `--no-daemon` when measuring, or stop the daemon by PID first.

### Step 9: Functional Verification Results (measured A/B)

Using `logging_simulator` + the official `sample-map-rosbag` / `sample-rosbag`, **a single playback** (without `--loop`):

```bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-rosbag \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit rviz:=false > launch.log 2>&1 &
# wait for the "Loaded node" count in the log to exceed 95 (more reliable than ros2 node list)
ros2 bag play $HOME/autoware_map/sample-rosbag --clock -r 0.5
```

| Criterion | Before fix | After fix |
|---|---|---|
| Component nodes loaded (`grep -c 'Loaded node'`) | 97 | **97** |
| `dlopen` failures (before Step 7) | **15** | **0** |
| Process deaths (before the `cuda_blackboard` patch) | **1** (always `pointcloud_container`, every run) | **0** |
| `cudaError` | 1 | **0** |
| `msg is timeout` (before/after sysctl tuning) | 38 | 22 → 28 |
| `No InputSource` (missing lidar input) | 10 | **1** |

Once the patch takes effect, the warnings themselves become evidence that the pipeline is working:

```
352  [lidar_centerpoint]: Fail to preprocess and skip to detect          ← CenterPoint is receiving point clouds and running inference
331  [sensing.lidar.concatenate_data]: transformed_raw_points[.../top/...]
173  [sensing.lidar.concatenate_data]: transformed_raw_points[.../left/...]    ← all three lidars are arriving
134  [sensing.lidar.concatenate_data]: transformed_raw_points[.../right/...]
166  [lidar_centerpoint]: Could not find a connection between 'base_link' and 'map'
```

That last line is **expected**: `map → base_link` only exists once localization initialization succeeds.

**What hasn't been made to work yet**: the full NDT convergence and planning closed loop. There are two reasons, neither on the build side — ① in a headless environment, automatic GNSS initialization returns `The GNSS pose is out of dimension` / `align server failed`; ② the official sample-rosbag has internally inconsistent timestamps that repeatedly clear the TF buffer (see Pitfall #12). To get it working, open RViz and set the initial pose manually, or switch to your own recorded data.

### Step 10: Verify Perception (CenterPoint) in isolation — use a static TF to decouple it from localization

When localization hasn't initialized, `map → base_link` doesn't exist, and CenterPoint keeps logging `Fail to preprocess and skip to detect`, which looks like perception is broken. **In reality it's just a missing TF.** Publishing one static transform is enough to verify perception on its own:

```bash
# after logging_simulator is up, before playing back the rosbag
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
  --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id base_link \
  --ros-args -p use_sim_time:=true &
```

A static TF is "always valid" in TF2, so it's unaffected by the sample bag's timestamp issue.

**Measured results** (three independent runs, Orin AGX 64GB, `centerpoint_tiny`):

| Criterion | Without static TF | With static TF |
|---|---|---|
| `Could not find a connection between 'base_link' and 'map'` | 166 | **0** |
| `Fail to preprocess and skip to detect` | 352 | 139 |
| `/perception/object_recognition/detection/centerpoint/objects` message count | **0** | **105 / 135 / 127** (three runs) |
| Cumulative detected objects | 0 | 104 / 184 / **202** |

**Inference performance** (`.../lidar_centerpoint/debug/processing_time_ms`, n=128):

```
min = 18.4 ms    median = 26.4 ms    p95 = 36.8 ms    max = 306.9 ms (first-frame TRT warm-up)
→ compute-side equivalent ceiling ≈ 37.9 Hz
actual publish period median = 142.8 ms (7.0 Hz) — limited by the input point cloud rate (bag replayed at -r 0.5), not a compute bottleneck
```

A median inference time of 26 ms leaves ample headroom for a 10 Hz lidar.

**The TensorRT engine is built on the spot**; this can be confirmed with file timestamps (this is the strongest verification — it simultaneously exercises the CUDA 12.8 build artifacts, TRT 10.3, the ONNX parser, and SM 87 kernel selection):

```bash
find ~/autoware_data -name '*.engine' -printf '%TY-%Tm-%Td %TH:%TM  %s  %f\n' | sort
# ml_models/lidar_centerpoint/pts_voxel_encoder_centerpoint_tiny.engine       95 KB
# ml_models/lidar_centerpoint/pts_backbone_neck_head_centerpoint_tiny.engine  10.4 MB
```

> **Dependency note**: `libautoware_lidar_centerpoint_component.so` has a direct `NEEDED libcuda_blackboard.so` — so without the one-line patch in Section 5, CenterPoint crashes along with `pointcloud_container`.
> Conversely, `autoware_tensorrt_plugins` (missing spconv, Pitfall #8) **does not affect CenterPoint**; it only affects the four packages `autoware_bevfusion` / `autoware_ptv3` / `autoware_tensorrt_vad` / `autoware_diffusion_planner`.

**Message type for the debug topics**: `processing_time_ms` / `cyclic_time_ms` use `autoware_internal_debug_msgs/msg/Float64Stamped` (1.9.0 has migrated off the old `tier4_debug_msgs`; the source has 234 references to the new package versus only 1 to the old one). Subscribing with the wrong type doesn't raise an error — you simply never receive any messages.

> **One lesson about measurement methodology**: don't probe from outside the graph with `ros2 topic hz`. On a fully-loaded Orin with 140 nodes, a newly created DDS participant can take tens of seconds to complete discovery, and the timeout fires first — so topics that are clearly publishing all show up as having no data. That's exactly how you get the self-contradictory result of "only one of three lidars has data." **Use in-graph evidence instead**: each node's own warnings in the startup log, `topic_state_monitor`'s verdicts, and the `process has died` count.

### When re-testing repeatedly, always confirm "only one instance is running"

`ros2 launch` starts nodes as **grandchild processes**; the common `trap 'kill $(jobs -p)'` pattern in scripts only kills direct children, so **the nodes survive**. After several rounds of repeated testing, you end up accumulating multiple identically-named nodes simultaneously subscribing to and publishing on the same topic.

Observed consequences (in this document's environment, **6** `lidar_centerpoint` instances accumulated, the oldest having survived 5896 seconds):

| Metric affected | Symptom |
|---|---|
| Frames processed | 27018 frames vs. the data actually containing only 9047 frames — **inflated 3x** |
| Detection count | The same object reported redundantly by multiple instances |
| Inference latency | Multiple instances contend for the GPU, making the measured latency meaningless |

**Diagnosis**: filtering by `comm` misses these — a node's `comm` gets truncated to `autoware_lidar_`, which doesn't match patterns like `ros2` / `python3`. Check the **full command line** instead:

```bash
ps -eo pid,etimes,args | grep -iE 'centerpoint|robot_state_publisher|static_transform|bag play' | grep -v '[g]rep'
```

**Root fix**: launch the test script with `setsid` (making the script itself the process group leader), and kill the entire process group in the trap:

```bash
trap 'kill -- -$$ 2>/dev/null' EXIT INT TERM
```

> One more thing in passing: the `kill` line **must not contain any full literal occurrence of the target process's name**. `pgrep -f <name>` will match the very shell executing that command too, causing a self-inflicted disconnect. Killing by PID is safest: first list the PIDs with a read-only command, then run a separate command containing only numbers to do the kill.

---

## 4. Pitfalls Encountered

| # | Symptom | Root cause | Fix |
|---|---|---|---|
| 1 | colcon exits after 33 seconds, 0 packages built, `Duplicate package names not supported` | An old checkout (e.g. `src.bak`) was left inside the workspace, causing the same package names to appear twice | Move the old checkout out of the workspace, or drop a `COLCON_IGNORE` file in it |
| 2 | `autoware_motion_utils` fails: `'<anonymous>' may be used uninitialized [-Werror=maybe-uninitialized]` pointing at `return {};` | The system default `gcc` was pinned to 9.5 via `update-alternatives` (commonly a legacy setting kept around to accommodate an older CUDA); a known GCC 9 false positive on `std::optional` combined with Autoware's `-Werror` | Switch to GCC 11 (Step 1). CMake 3.22 can't be worked around with `CMAKE_COMPILE_WARNING_AS_ERROR` |
| 3 | `autoware_pointcloud_preprocessor` / `cuda_blackboard` / `autoware_grid_map_utils` fail at the configure stage: `Could NOT find CUDA: Found unsuitable version "12.6", but required is exact version "12.2"` | The locally-built OpenCV hardcodes `set(OpenCV_CUDA_VERSION "12.2")` in `OpenCVConfig.cmake` with `find_host_package(CUDA ... EXACT REQUIRED)`, propagated through `cv_bridge` to every package that uses OpenCV | See "Known Constraints" in Section 5 |
| 4 | `cuda_blackboard` fails to build: `'cudaStreamGetDevice' was not declared in this scope` | Autoware 1.9.0 pins `cuda_blackboard` 0.4.0, which uses an API that requires CUDA ≥12.8; JetPack 6.x only provides 12.6 | Install the CUDA 12.8 toolchain (Step 2) |
| 5 | `/usr/local/cuda` is already 12.8, yet it still reports `Found unsuitable version "12.6"` | That package's `CMakeCache.txt` was generated under the old environment | `rm -rf build install log`, then rebuild |
| 6 | `setup-dev-env.sh` fails in the agnocast role: `gpg: WARNING: unsafe ownership on homedir '/home/lz/.gnupg'` | `~/.gnupg` is owned by root (left over from some earlier `sudo` invocation that ran gpg while carrying `HOME` along) | `sudo chown -R $USER:$USER ~/.gnupg && chmod 700 ~/.gnupg` |
| 7 | All 488 packages build successfully, but 15 CUDA nodes report `dlopen error: libcudart.so.12: cannot open shared object file` at startup | After switching `/usr/local/cuda` to 12.8 (**SBSA layout** `targets/sbsa-linux/`), the ld cache wasn't refreshed, and `/etc/ld.so.conf.d/000_cuda.conf` still points at the Tegra layout `targets/aarch64-linux/`, so the soname in the cache lands on a dangling path | `sudo ldconfig` + add `LD_LIBRARY_PATH=/usr/local/cuda-12.8/targets/sbsa-linux/lib` to Autoware's runtime environment (Step 7). **Do not** make 12.8 the system-wide default |
| 8 | `source install/setup.bash` reports `not found: ".../autoware_tensorrt_plugins/share/autoware_tensorrt_plugins/local_setup.bash"` | This package calls `return()` after `find_package(spconv)` fails, never reaching `ament_package()`, so `local_setup.bash` is never generated (it "finishes" in 2.6 seconds). `cumm`/`spconv` are not in 1.9.0's repos list | Only affects sparse-convolution models like bevfusion/transfusion; centerpoint and others are unaffected. This warning can be ignored, or spconv can be pulled in separately if needed |
| 9 | No errors, but some topics **silently have no data**, unevenly distributed (only one of three lidars has data); downstream `gyro_odometer: IMU msg is timeout` cascades into a wave of timeouts | `net.core.rmem_max` is still Ubuntu's default 208 KB; 140 nodes plus multi-MB point clouds over the UDP loopback overflow it outright. `setup-dev-env.sh` doesn't lay down the DDS sysctl settings | Tune sysctl per Step 8. **Check the buffer before suspecting the code** |
| 11 | All 488/488 packages build, but exactly 1 process dies at startup (`pointcloud_container`); `ndt_scan_matcher` reports `No InputSource`, and localization initialization reports `align server failed` | `cuda_blackboard` uses `cudaStreamGetDevice`, which requires a **CUDA 12.8 driver**; JetPack 6.x only has 12.6 → `cudaErrorCallRequiresNewerDriver (36)` → the container's constructor throws and aborts | The one-line patch in Section 5 (switch to `cudaGetDevice`) + rebuild `cuda_blackboard` |
| 12 | `tf2_buffer: Detected jump back in time. Clearing TF buffer.` floods the entire graph (55k times for a single playback, 270k times with `--loop`) | **The official `sample-rosbag` has internally inconsistent timestamps**: the sensor timestamps inside the bag read `1585897272` (2020-04-03), while the bag's message timestamps read `1614315746` (2021-02-26), repeatedly clearing the TF buffer. `--loop` makes `/clock` jump backward on top of that, making it worse | Unrelated to this build. At minimum, avoid `--loop` for functional verification; to get full localization working, switch to your own recorded data, or follow the official tutorial and set the initial pose manually in RViz |
| 13 | After switching to CycloneDDS, 60+ nodes exit with `exit code -6` | A custom `cyclonedds.xml` (restricting to the `lo` interface + `SocketReceiveBufferSize`) causes initialization to abort in this environment | Revert to FastDDS (Step 8) |
| 10 | The topic is clearly publishing, yet `ros2 topic hz` says it can't be found | `ros2-daemon` cached the topology graph from a previous launch | Add `--no-daemon` when measuring, or stop the daemon by PID |

### Two Lessons on Diagnostic Method

**A "task header" in the log is not evidence of execution.** ansible prints `TASK [... : Install cuda-drivers]` for skipped tasks too; the `skipping:` line right after it is what actually tells you the outcome. Judging by the header alone leads to wrongly halting a healthy process.

**Verify conclusions against outcomes, not by inferring from the process.** Whether a system has actually been changed should be determined with state-querying commands like `dpkg -l` / `readlink -f` / `apt-mark showhold` / `opencv_version --verbose`, not by "I didn't see that line in the log."

---

## 5. Known Constraints

### OpenCV 4.8.0 cannot be built with nvcc 12.8

Intuitively, OpenCV and Autoware should use the same CUDA version. In practice, **this doesn't work**: rebuilding OpenCV 4.8.0 with CUDA 12.8 fails at the `cudaarithm` / `cudawarping` stage,

```
opencv_contrib/modules/cudev/include/opencv2/cudev/grid/detail/reduce.hpp(379):
  error: no instance of overloaded function "cv::cudev::blockReduce" matches the argument list
```

This is an upstream incompatibility between OpenCV 4.8's `cudev` module and newer nvcc versions (fixed only in 4.9/4.10), and is exactly why the existing SOP builds OpenCV against CUDA 12.2.

**Current approach** (workable but not the final state): keep the OpenCV build against CUDA 12.2, and merely relax its CMake version assertion so that Autoware can configure successfully.

```bash
C=/usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake
sudo cp -n $C $C.bak-cuda122
sudo sed -i 's/set(OpenCV_CUDA_VERSION "12.2")/set(OpenCV_CUDA_VERSION "12.8")/' $C
```

**This change has no runtime consequences**, because Autoware never touches OpenCV's GPU code at all. Supporting evidence (verified on Orin, 2026-09-22):

| Check | Command | Result |
|---|---|---|
| Source includes a CUDA module | `grep -rl 'opencv2/cuda' src` | **0** |
| Source calls `cv::cuda::` | `grep -rl 'cv::cuda::' src` | **0** |
| CMakeLists requests a cuda component | `grep -rniE 'cuda(arithm\|warping\|...)' --include=CMakeLists.txt` | **0** |
| Build artifacts depend on `libopencv_cuda*` | `objdump -p $(readlink -f *.so) \| grep NEEDED` | **0** |
| OpenCV modules actually depended on by the build artifacts | same as above | only `core/imgproc/imgcodecs/calib3d/highgui/dnn/photo/ximgproc` (39 libraries, all CPU-side) |

Autoware's GPU inference goes through TensorRT plus hand-written CUDA kernels; here OpenCV **is used purely as a CPU-side image library**. Those 10 `libopencv_cuda*.so.408` libraries have no callers at all.

Also, this local OpenCV build uses OpenCV's default `CUDA_USE_STATIC_CUDA_RUNTIME=ON`:

```
Extra dependencies: ... cudart_static ... -L/usr/local/cuda-12.2/lib64
```

The CUDA 12.2 runtime is linked in **statically** — not one of the 68 `.so` files has `NEEDED libcudart` — so there's no soname conflict, and that static runtime is never initialized either. Here `OpenCV_CUDA_VERSION` is purely a **build-time assertion**.

> An earlier version of this document stated that "two CUDA runtimes might coexist within the process, and passing `cudaStream_t`/`GpuMat` across the OpenCV boundary would need extra verification." Based on the measurements in the table above, that concern does not apply to Autoware's pipeline, and this has been corrected.

**Should you upgrade to OpenCV 4.9/4.10? Not needed for Autoware, and the cost is high.** The soname would change from `.408` to `.410`, instantly breaking every binary with `NEEDED libopencv_*.so.408`:

| Affected | Count | Can it be rebuilt |
|---|---|---|
| Autoware libraries | 39 | Yes, 2h23min |
| **GMSL camera driver** (`gmsl_node`, `NEEDED libopencv_cudawarping.so.408`) | 1 | Yes, but **requires OpenCV's CUDA module** |
| Metavision SDK 5.1.1 (including Python extension) | 4 | **No, vendor-prebuilt** |

The third item is a hard blocker (the event-camera pipeline depends on it). **4.10 is only genuinely needed in the following two cases**:

1. You need to use `cv::cuda::` on the host and it must be built together with CUDA 12.8 (4.8's `cudev` can't get past nvcc 12.8)
2. You need OpenCV DNN's CUDA backend with cuDNN 9 (4.8 only supports cuDNN 8)

If 4.10 is genuinely needed in the future, the right approach is to **coexist rather than replace**: install it under its own prefix at `/opt/opencv-4.10`, point `OpenCV_DIR` at it for whichever project needs it, and leave the 4.8 install under `/usr/lib/aarch64-linux-gnu` untouched.

### Compatibility with Other Projects on the Same Machine (measured 2026-09-22)

Two other projects on the same Orin also use OpenCV / CUDA, and both **build successfully** under JP6.2 + CUDA 12.8:

| Project | OpenCV dependency | CUDA dependency | Result |
|---|---|---|---|
| **GMSL_Camera_ROS2** | `find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs cudaimgproc cudawarping)`, source includes `#include <opencv2/cudaimgproc>` / `<opencv2/cudawarping>` | Its own `.cu` files, `enable_language(CUDA)` | `Finished <<< gmsl [25.8s]`, `ldd` shows 0 unresolved |
| `vehicle/camera/gst-nvfisheyeundistort/` in **Remote_Driving_System_Prod** (dev) | **None** | CUDA kernel + GStreamer + NvBufSurface | `make` succeeds on the first try, `gst-inspect-1.0 nvfisheyeundistort` recognizes it correctly |

Two takeaways:

1. **The GMSL driver is itself the reason the local 4.8-CUDA build can't be replaced** — its build artifact has a direct `NEEDED libopencv_cudawarping.so.408`.
2. **The 12.8 version pin applied to `OpenCVConfig.cmake` earlier is required for this project too**, not just for Autoware: `/usr/local/cuda` now points at 12.8, and if the pin still said 12.2, its `find_package(OpenCV REQUIRED COMPONENTS ... cudaimgproc)` line would FATAL_ERROR outright. The two projects' requirements are **aligned here, not in conflict**.

The only thing GMSL needs added is an environment variable (no code changes) — it calls `enable_language(CUDA)` without specifying a compiler, so running it as-is reports `No CMAKE_CUDA_COMPILER could be found`:

```bash
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=/usr/local/cuda-12.8/bin:$PATH
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=87
```

The fisheye-undistortion plugin's Makefile doesn't need a single character changed: the two compatibility symlinks it references, `/usr/local/cuda/include` and `/usr/local/cuda/lib64`, **are indeed created by the CUDA 12.8 SBSA package** (pointing at `targets/sbsa-linux/{include,lib}` respectively). `nvbufsurface.h` falls back to its own path, `/usr/src/jetson_multimedia_api/include` (DeepStream isn't installed on this machine, so this fallback is exactly what gets hit).

### `cuda_blackboard` must be patched on JetPack (otherwise the point cloud pipeline crashes on startup)

This is the **most critical issue** uncovered by this round of functional verification, and it **never surfaces at build time**: all 488 packages build successfully, `ldd` shows zero unresolved dependencies, yet one process dies the moment things start — the same one every time:

```
[ERROR] [component_container_mt-1]: process has died [exit code -6,
        cmd '.../component_container_mt --ros-args -r __node:=pointcloud_container ...']

[component_container_mt-1] terminate called after throwing an instance of 'std::runtime_error'
[component_container_mt-1]   what():  cudaErrorCallRequiresNewerDriver (36)
                                      @.../cuda_blackboard/src/cuda_mem_pool_context.cpp#L42
```

`pointcloud_container` hosts cropping, ground segmentation, the occupancy grid, and point cloud concatenation. Once it dies, `/sensing/lidar/concatenated/pointcloud` stops existing, and then:

```
[WARN] [localization.pose_estimator.ndt_scan_matcher]: No InputSource. Please check the input lidar topic
[WARN] [localization.pose_twist_fusion_filter.ekf_localizer]: The node is not activated. Provide initial pose to pose_initializer
[ERROR] [system.service_log_checker]: /api/localization/initialize: status code 4 'align server failed.'
```

— a string of downstream symptoms that look like "wrong sensor configuration" or "wrong map," when **the root cause is actually a single CUDA call**.

**Root cause**: `cudaStreamGetDevice` requires a CUDA **12.8 driver**, and JetPack 6.x tops out at a 12.6 driver (see the measured output in Section 2). Installing the 12.8 toolchain solves the build, but there is no 12.8 driver on the driver side — nor can there be one (installing the desktop driver package on a Jetson is off-limits).

**Fix** (one line, semantically equivalent): the stream was created on the current device just two lines earlier, so `cudaGetDevice` returns that same device.

```cpp
// src/universe/external/cuda_blackboard/src/cuda_mem_pool_context.cpp:42
- CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaStreamGetDevice(stream_, &device_id));
+ CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaGetDevice(&device_id));
```

```bash
cd ~/autoware
cp -n src/universe/external/cuda_blackboard/src/cuda_mem_pool_context.cpp{,.bak-orig}
# after the edit, only this one package needs rebuilding (internal implementation change, ABI unchanged, dependents don't need rebuilding)
colcon build --packages-select cuda_blackboard --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=87 \
               -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc
# about 3 seconds
```

> This is a local modification to an **external dependency** pulled in via `autoware.repos`; it will be overwritten by `vcs import` the next time you upgrade the Autoware version and will need to be reapplied. Consider maintaining it in your own fork, or turning it into a patch file included in your deployment scripts.

### cv_bridge and Autoware use different OpenCV builds (pre-existing state)

`/opt/ros/humble/lib/libcv_bridge.so` links against apt's `libopencv_core.so.4.5d`, while Autoware's own libraries link against the locally-built `.408`. Both sonames end up loaded simultaneously within the same process, and ELF symbol resolution follows the global symbol table in load order.

This is a pre-existing condition of "apt ROS + self-built OpenCV" on Jetson, not something introduced by this build, and **upgrading to 4.10 wouldn't fix it either** (4.10 would still have to coexist with 4.5.4d). All 488 packages currently build and link successfully with 0 unresolved symbols in `ldd`, so this is left as-is for now and only recorded here. Eliminating it entirely would require rebuilding `cv_bridge` from source so it points at the same OpenCV.

Note that the system may have **two** OpenCV configs present, and CMake prefers the architecture-specific path:

| Path | Source | CUDA |
|---|---|---|
| `/usr/lib/aarch64-linux-gnu/cmake/opencv4/` | Locally built and installed (`dpkg -S` finds no owning package) | Yes |
| `/usr/lib/cmake/opencv4/` | apt `libopencv-dev` (JetPack) | No |

### cuDNN 8 and 9 coexisting

JetPack 6.2 installs cuDNN 9.3, but `libcudnn8` 8.9.4 is typically still present (different package name, so apt won't replace it). The unversioned `/usr/include/cudnn_version.h` is pointed at v9 via alternatives. Enabling OpenCV 4.8's DNN-CUDA backend would require explicitly pointing at v8 — but Autoware performs inference through TensorRT and doesn't use OpenCV's DNN backend, so it's typically fine to just set `WITH_CUDNN=OFF`.

### `autoware_individual_params` is not in 1.9.0's repos list — this is expected, **don't try to add it back**

1.9.0's `repositories/autoware.repos` contains only 32 repositories; `ros2 pkg prefix autoware_individual_params` will report MISSING.

**This is not an omission.** That repository was archived in 2025 ([autoware#5975](https://github.com/autowarefoundation/autoware/issues/5975)), and vehicle-related parameters have moved into each sensor kit's own description package:

```
src/launcher/sample_sensor_kit_launch/sample_sensor_kit_description/config/
  ├── imu_corrector.param.yaml
  ├── sensor_kit_calibration.yaml
  └── sensors_calibration.yaml
```

A check across all of 1.9.0's source confirms the reference count for `individual_params` is **0**:

```bash
grep -rl 'individual_params' src --include=*.xml --include=*.py --include=*.yaml | wc -l   # 0
```

Cloning that archived repository would only add an unreferenced `individual_params` package to the workspace (note its package name has no `autoware_` prefix), and could create confusion with the parameters in their new location. For a custom vehicle, put calibration parameters in your own `<vehicle>_sensor_kit_description`; don't resurrect the individual_params path.

---

## 6. Troubleshooting Index

| What you see | Where to go |
|---|---|
| `Duplicate package names not supported` | Pitfall #1 |
| `-Werror=maybe-uninitialized` pointing at `return {};` | Pitfall #2 (switch to GCC 11) |
| `Could NOT find CUDA: ... required is exact version` | Pitfall #3 / Section 5 |
| `cudaStreamGetDevice was not declared` | Pitfall #4 (install CUDA 12.8) |
| `/usr/local/cuda` version doesn't match the error message | Pitfall #5 (clean the build tree) |
| `gpg: unsafe ownership on homedir` | Pitfall #6 |
| The `.so` in `install/` is only 92 bytes | Step 6's note (follow the symlink) |
| `dlopen error: libcudart.so.12: cannot open shared object file` | Pitfall #7 / Step 7 (ld cache + `LD_LIBRARY_PATH`) |
| `not found: ".../autoware_tensorrt_plugins/.../local_setup.bash"` | Pitfall #8 (missing spconv, can be ignored) |
| Topic silently has no data / `IMU msg is timeout` / only one of three lidars has data | Pitfall #9 / Step 8 (208 KB UDP buffer) |
| `ros2 topic hz` can't find a topic that's clearly publishing | Pitfall #10 (add `--no-daemon`) |
| `cudaErrorCallRequiresNewerDriver (36)` | Pitfall #11 / Section 5 (`cuda_blackboard` one-line patch) |
| `pointcloud_container` dies on startup / `No InputSource` / `align server failed` | Pitfall #11 (same as above — these are all downstream symptoms) |
| `tf2_buffer: Detected jump back in time` floods the log | Pitfall #12 (don't use `--loop`) |
| Widespread `exit code -6` across nodes after switching to CycloneDDS | Pitfall #13 (revert to FastDDS) |
| `blockReduce`/`blockReduceKeyVal` overload resolution fails | Section 5 (OpenCV + nvcc 12.8 incompatibility) |

---

*All version numbers and conclusions in this document come from measurements taken on 2026-09-22 on lz@192.168.8.3 (AGX Orin Developer Kit).*
