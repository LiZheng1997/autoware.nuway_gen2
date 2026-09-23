# Orin 原生编译 Autoware 1.9

在 AGX Orin 开发套件上从源码编译 Autoware 1.9.0，不使用 Docker。

**实测结果**：488/488 包全部成功，耗时 2 小时 23 分，零失败（2026-09-22）。

---

## 1. 适用环境

| 项 | 本文验证的版本 |
|---|---|
| 硬件 | Jetson AGX Orin 开发套件（p3737-0000 + p3701-0005，64 GB） |
| 系统 | JetPack 6.2 / L4T **R36.4.3** / 内核 5.15.148-tegra |
| JetPack 组件 | nvidia-jetpack **6.2+b77** |
| 驱动侧 CUDA | 12.6（JetPack 自带，**不要动**） |
| 编译侧 CUDA | **12.8**（单独装，见步骤 2） |
| TensorRT | 10.3.0.30（JetPack 自带） |
| cuDNN | 9.3.0.75（JetPack 自带） |
| GCC | **11.4.0** |
| Autoware | tag **1.9.0**（488 个包） |
| 磁盘 | 编译期间至少预留 20 GB（install 351 MB + build 4.3 GB + 依赖） |

---

## 2. 环境要求与理由

这四项**必须先满足**，否则编译一定失败。它们的顺序不能颠倒——每解决一个才看得见下一个。

### GCC 必须是 11

ROS 2 Humble 的 apt 二进制包全部用 GCC 11 构建。若系统默认编译器不是 11：

- GCC 9 对 `std::optional` + `return {};` 会误报 `maybe-uninitialized`，配合 Autoware 的 `-Werror` 直接变成硬错误
- GCC 9 产物与 GCC 11 编译的 ROS 库存在 ABI 风险

```bash
gcc --version    # 必须是 11.x
```

若不是，见步骤 1。**注意** Ubuntu 22.04 自带 CMake 3.22，因此 `CMAKE_COMPILE_WARNING_AS_ERROR`（需 ≥3.24）这条关闭 `-Werror` 的路走不通。

### 编译侧 CUDA 必须是 12.8

Autoware 1.9.0 在 `ansible/roles/cuda/defaults/main.yaml` 里明确写着：

```yaml
# CUDA 12.8 on Ubuntu 22.04 (humble / Jetson Orin via JetPack 6).
cuda_version: "{{ '13.0' if ansible_distribution_version == '24.04' else '12.8' }}"
cuda_repo_distro: "{{ 'ubuntu2404' if ... else 'ubuntu2204' }}"
```

**JetPack 6.x 全系列只提供 CUDA 12.6**（实测 r36.4 仓库里 nvidia-cuda 的 6.1+b123 / 6.2+b77 / 6.2.1+b38 三个版本都依赖 `cuda-12-6`）。所以升级 L4T 小版本拿不到 12.8，只能从 NVIDIA 的 `ubuntu2204` CUDA 仓库单独安装工具链。

差别是实打实的：`cuda_blackboard` 0.4.0 使用 `cudaStreamGetDevice()`，该函数在 CUDA 12.6 与 12.2 的头文件里都不存在。

> ### ⚠️ 但装了 12.8 也只是能**编过**——必须同时打第 7 节那个一行补丁
>
> `cudaStreamGetDevice` 不只需要 12.8 的**头文件**，运行时还需要 12.8 的**驱动**。而 **JetPack 6.x 的驱动最高是 CUDA 12.6，拿不到 12.8 驱动**。实测：
>
> ```
> runtime=12080  driver=12060
> cudaStreamGetDevice  -> 36 (cudaErrorCallRequiresNewerDriver)   dev=-1
> cudaGetDevice        ->  0 (cudaSuccess)                        dev=0
> ```
>
> 结果就是 488 个包全部编译成功，但 `pointcloud_container` **启动即 abort**，点云预处理/拼接整条链没了。详见第 5 节「cuda_blackboard 在 JetPack 上必须打补丁」。
>
> 而且 `cudaStreamGetDevice` 在**全部 488 个包里只出现一次**（就是 `cuda_blackboard` 那一行）。也就是说，打完那个补丁之后，CUDA 12.8 这整条 SBSA 绕路很可能都不必要，直接用 JetPack 自带的 12.6 即可（源码证据确定，但未重跑完整构建验证）。

### 驱动侧 CUDA / TensorRT / cuDNN 保持 JetPack 自带

只装 CUDA **工具链**，不要装 `cuda-drivers`——在 Jetson 上安装桌面驱动包会破坏 Tegra 图形与 GPU 栈。`setup-dev-env.sh` 的 `--no-cuda-drivers` 选项存在的意义就是这个。

TensorRT 方面，ansible 默认钉 `10.3.0.26-1+cuda12.5`，而 JetPack 6.2 给的是 `10.3.0.30-1+cuda12.5`。**不必强求一致**：同为 TRT 10.3，且 JetPack 版本才是针对 Orin 集成 GPU 构建的。上机前建议实测一次引擎构建：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=$HOME/autoware_data/lidar_centerpoint/pts_voxel_encoder_centerpoint.onnx \
  --saveEngine=/tmp/t.plan --skipInference
# 期望: "Engine built in ... sec" + "&&&& PASSED"
```

### 换过编译器或 CUDA 之后必须清构建树

`build/*/CMakeCache.txt` 会缓存 CUDA 版本与编译器路径。换环境后不清，会出现"`/usr/local/cuda` 明明是 12.8，却报 Found unsuitable version 12.6"这类自相矛盾的错误。

---

## 3. 步骤

### 步骤 0：前置检查

```bash
cat /proc/device-tree/model            # 应为 Jetson AGX Orin Developer Kit
head -1 /etc/nv_tegra_release          # R36 REVISION: 4.3
nproc; free -g; df -h /                # 12 核 / 61 G / 预留 ≥20 G
sudo nvpmodel -q | tail -2             # 建议 MAXN
docker ps                              # 应为空：容器抢 CPU 会拖慢编译
```

### 步骤 1：切到 GCC 11

`gcc`/`g++` 各自是独立的 alternative，不能用 `--slave` 挂在一起。

```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 110
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 110
sudo update-alternatives --set gcc /usr/bin/gcc-11
sudo update-alternatives --set g++ /usr/bin/g++-11

gcc --version && g++ --version && cc --version && c++ --version   # 四个都要是 11.x
```

### 步骤 2：安装 CUDA 12.8 工具链

```bash
# 添加 NVIDIA CUDA 仓库（aarch64 用 sbsa 路径）
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/sbsa/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# 先模拟，确认计划里没有任何驱动包
sudo apt-get -s install cuda-toolkit-12-8 | grep -E "cuda-drivers|nvidia-driver|nvidia-kernel"
#   ↑ 必须无输出

sudo apt-get install -y cuda-toolkit-12-8
sudo update-alternatives --set cuda /usr/local/cuda-12.8

# 验证
readlink -f /usr/local/cuda                                   # /usr/local/cuda-12.8
/usr/local/cuda/bin/nvcc --version | tail -2 | head -1        # release 12.8
grep -rl cudaStreamGetDevice /usr/local/cuda-12.8/include/    # 必须命中 cuda_runtime_api.h
dpkg -l | grep -E "^ii  (cuda-drivers|nvidia-driver)"         # 必须无输出
```

### 步骤 3：取源码

```bash
git clone https://github.com/autowarefoundation/autoware.git ~/autoware
cd ~/autoware
git fetch --tags
git checkout 1.9.0
mkdir src
vcs import src < repositories/autoware.repos      # 32 个仓库

find src -name package.xml | wc -l                # 期望 488
```

> **`src` 同级不要留旧检出。** 任何位于工作区内、含有同名包的目录（例如把旧 `src` 改名成 `src.bak`）都会让 colcon 以
> `ERROR:colcon:colcon build: Duplicate package names not supported` 立即中止，**一个包都不编**。旧检出要移到工作区之外，
> 或在其中放一个 `COLCON_IGNORE` 文件。

### 步骤 4：安装依赖环境

```bash
cd ~/autoware
./setup-dev-env.sh universe -y --no-nvidia
```

`--no-nvidia` 跳过 `cuda` 与 `tensorrt` 两个角色，保护步骤 2 装好的工具链与 JetPack 自带的 TRT/cuDNN。

若只想跳过驱动、让 ansible 自己装 CUDA/TRT，则用 `--no-cuda-drivers` 代替。

**判据**：结尾出现 `failed=0`。

> `--no-nvidia` 的实现是 `--extra-vars prompt_install_nvidia=n`。ansible **对被跳过的任务同样会打印 `TASK [...]` 标题**，
> 下一行才是 `skipping: [localhost]`。不要把任务标题当成"它在执行"的证据。

然后是 rosdep：

```bash
source /opt/ros/humble/setup.bash
rosdep update --include-eol-distros
rosdep install -y --from-paths src --ignore-src --rosdistro humble
# 判据: "#All required rosdeps installed successfully"
```

### 步骤 5：编译

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

参数取舍：

- **`--parallel-workers 4` + `MAKEFLAGS=-j3`**：12 核开满会 OOM。`tensorrt_yolox`、`lidar_centerpoint`、`bevdet_vendor` 的单个 CUDA 编译单元能吃掉数 GB，构建中途被 OOM killer 打断的代价远高于慢一点。实测峰值可用内存始终 >50 GB。
- **`-DCMAKE_CUDA_ARCHITECTURES=87`**：Orin 是 SM 87，只生成这一个架构的代码，省下可观时间。
- **`--continue-on-error`**：首次编译要的是**一次拿到全部失败清单**，而不是卡在第一个。

**点火后前 2 分钟必须查一次**，而且要看正向进展的证据，不能只看"日志还没报错"：

```bash
grep -c "Starting >>>" log/latest_build/../../colcon_build.log   # 应在增长
grep -c "Finished <<<" colcon_build.log
grep -cE "^ERROR:colcon" colcon_build.log                        # 应为 0
```

长时间编译建议挂心跳，**连续两次无进展就报警**——编译卡死不该靠人盯着发现。

### 步骤 6：验证

```bash
grep "^Summary:" colcon_build.log     # Summary: 488 packages finished [2h 23min 17s]
grep -c "^Failed   <<<" colcon_build.log   # 0

source install/setup.bash
ros2 pkg list | wc -l                  # 849

# CUDA 产物不是空壳：跟进 symlink 看真实文件，并确认依赖全部可解析
L=$(readlink -f install/autoware_lidar_centerpoint/lib/libautoware_lidar_centerpoint_cuda_lib.so)
ls -lL "$L"                            # 约 3.9 MB
ldd "$L" | grep -c "not found"         # 0
```

> `--symlink-install` 下，`install/` 里的 `.so` 是符号链接，`ls -l` 只会显示 92 字节左右的链接长度。
> 必须 `readlink -f` 跟进真实文件再判断大小，否则会误判成"编出来是空的"。

### 步骤 7：运行期让 Autoware 找到 CUDA 12.8（**编译通过 ≠ 能跑**）

这一步不做，488 个包全部编译成功，但**一启动就有 15 个 CUDA 节点加载失败**：

```
[ERROR] Failed to load library: Could not load library dlopen error:
  libcudart.so.12: cannot open shared object file: No such file or directory
  → traffic_light_fine_detector / car_traffic_light_classifier
  → pedestrian_traffic_light_classifier / lidar_centerpoint ...
```

原因是**两种 CUDA 的目录布局不同**，而动态链接器的缓存不认编译期的 `-L`：

| 来源 | 库目录布局 |
|---|---|
| JetPack 的 CUDA（Tegra 包） | `/usr/local/cuda-12.6/targets/**aarch64-linux**/lib` |
| 单独装的 CUDA 12.8（**SBSA 包**） | `/usr/local/cuda-12.8/targets/**sbsa-linux**/lib` |

把 `/usr/local/cuda` 切到 12.8 后，`/etc/apt` 之外还有一处遗留：`/etc/ld.so.conf.d/000_cuda.conf`（文件名前缀 000，扫描优先级最高）写的是 `/usr/local/cuda/targets/aarch64-linux/lib` —— 这个目录在 12.8 下**不存在**了，而 ld 缓存是在切换前生成的，于是缓存里 `libcudart.so.12` 仍指向一条**悬空路径**，`dlopen` 必然失败。

**先刷新缓存，再看它落到哪个版本：**

```bash
sudo ldconfig
ldconfig -p | grep 'libcudart.so.12 '
```

在本文的环境里刷新后 soname 落到了 **12.2**（因为 `gds-12-2.conf` 指的目录仍然存在）。**这不够**——

```bash
# 只有 12.8 提供 cuda_blackboard 需要的 cudaStreamGetDevice
for v in 12.2/targets/aarch64-linux 12.6/targets/aarch64-linux 12.8/targets/sbsa-linux; do
  printf 'cuda-%-30s ' "$v"
  nm -D --defined-only /usr/local/cuda-$v/lib/libcudart.so.12 \
    | grep -qw cudaStreamGetDevice && echo 有 || echo 无
done
# cuda-12.2/...  无
# cuda-12.6/...  无
# cuda-12.8/...  有
```

**不要把 12.8 设成全系统默认。** Jetson 应当保留 JetPack 自己的 CUDA 作为系统默认（相机、多媒体、DeepStream 都依赖它）。只给 Autoware 的运行环境加一条路径即可：

```bash
# 和 source install/setup.bash 放在一起（写进 ~/.bashrc 或启动脚本）
export LD_LIBRARY_PATH=/usr/local/cuda-12.8/targets/sbsa-linux/lib:${LD_LIBRARY_PATH}
```

**验证判据**（实测 15 → 0）：

```bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-rosbag \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit rviz:=false > launch.log 2>&1

grep -c 'cannot open shared object file' launch.log    # 必须为 0
```

### 步骤 8：配置 DDS（`setup-dev-env.sh` **不会**替你做）

Autoware 一次启动 **140 个节点**，点云单帧数 MB，全部走 UDP 回环。Ubuntu 默认的接收缓冲只有 208 KB，必然丢包——而 `setup-dev-env.sh` 不会落任何 DDS 相关的 sysctl 配置（`/etc/sysctl.d/` 里查不到 `rmem_max`）。

症状非常有迷惑性，因为它**不报错、只是丢数据**，而且丢得不均匀：

```
# 同一个 bag 里三路雷达，同一测量窗口
/sensing/lidar/right/velodyne_packets   5.972       ← 有
/sensing/lidar/top/velodyne_packets     无数据      ← 主雷达，没有
/sensing/lidar/left/velodyne_packets    无数据

# 于是下游连环报超时
[ERROR] [localization.twist_estimator.gyro_odometer]: IMU msg is timeout.
[ERROR] [localization.twist_estimator.gyro_odometer]: Vehicle twist msg is timeout.
```

很容易被误判成"编译出来的包有问题"或"传感器配置对不上"。**先查缓冲区，再怀疑代码。**

```bash
sysctl -n net.core.rmem_max      # 默认 212992 —— 远远不够
```

**处置**（官方要求值）：

```bash
sudo tee /etc/sysctl.d/60-autoware-dds.conf >/dev/null <<'EOF'
net.core.rmem_max=2147483647
net.core.wmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sudo sysctl --system
```

**RMW 保持默认的 FastDDS。** 网上常见的建议是「换 CycloneDDS」，本环境实测**不要照抄**：写一份把 `<Interfaces>` 限定到 `lo`、带 `SocketReceiveBufferSize min="10MB"` 的 `cyclonedds.xml` 并设 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 之后，**几乎所有节点在初始化阶段就 SIGABRT**：

```
[ERROR] [autoware_ndt_scan_matcher_node-26]: process has died [pid ..., exit code -6, ...]
[ERROR] [autoware_ekf_localizer_node-30]:   process has died [pid ..., exit code -6, ...]
[ERROR] [imu_corrector_node-22]:            process has died [pid ..., exit code -6, ...]
（十余个节点同样 exit code -6）
```

回退到 FastDDS 即恢复正常。换 RMW 属于独立的调优议题，**要单独验证，不要和本 SOP 的步骤混在一起做**——否则 SIGABRT 会被误记到编译产物上。真正解决丢数据的是上面那四个 sysctl。

> **诊断时顺手关掉 ros2 CLI 守护进程。** `ros2 topic hz` / `node list` 默认经由 `ros2-daemon`，它会缓存拓扑图；上一次启动残留的守护进程会让你在新的一轮里看到过期的图，表现为"话题明明在发但 hz 找不到"。测量时一律加 `--no-daemon`，或先按 PID 停掉守护进程。

### 步骤 9：功能验证结果（实测 A/B）

用 `logging_simulator` + 官方 `sample-map-rosbag` / `sample-rosbag`，**单次回放**（不用 `--loop`）：

```bash
ros2 launch autoware_launch logging_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-rosbag \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit rviz:=false > launch.log 2>&1 &
# 等日志里 "Loaded node" 计数 > 95（比 ros2 node list 可靠）
ros2 bag play $HOME/autoware_map/sample-rosbag --clock -r 0.5
```

| 判据 | 修复前 | 修复后 |
|---|---|---|
| 组件节点加载（`grep -c 'Loaded node'`） | 97 | **97** |
| `dlopen` 失败（步骤 7 之前） | **15** | **0** |
| 进程死亡（`cuda_blackboard` 补丁之前） | **1**（每轮都是 `pointcloud_container`） | **0** |
| `cudaError` | 1 | **0** |
| `msg is timeout`（sysctl 调优前后） | 38 | 22 → 28 |
| `No InputSource`（雷达输入缺失） | 10 | **1** |

补丁生效后，告警本身就是链路在工作的证据：

```
352  [lidar_centerpoint]: Fail to preprocess and skip to detect          ← CenterPoint 在收点云做推理
331  [sensing.lidar.concatenate_data]: transformed_raw_points[.../top/...]
173  [sensing.lidar.concatenate_data]: transformed_raw_points[.../left/...]    ← 三路雷达全部到达
134  [sensing.lidar.concatenate_data]: transformed_raw_points[.../right/...]
166  [lidar_centerpoint]: Could not find a connection between 'base_link' and 'map'
```

最后那条是**预期状态**：`map → base_link` 只有在定位初始化成功后才存在。

**尚未跑通的一段**：完整的 NDT 收敛与规划闭环。原因有两个，都不在构建侧——① 无头环境下 GNSS 自动初始化返回 `The GNSS pose is out of dimension`/`align server failed`；② 官方 sample-rosbag 自身时间戳不一致导致 TF 缓冲反复清空（见坑 #12）。要跑通建议开 RViz 手动给初始位姿，或改用自采数据。

### 步骤 10：单独验证感知（CenterPoint）—— 用静态 TF 把它从定位里隔离出来

定位没初始化时，`map → base_link` 不存在，CenterPoint 会刷 `Fail to preprocess and skip to detect`，看起来像感知坏了。**实际上只是缺 TF。**补一个静态变换即可单独验证感知：

```bash
# 在 logging_simulator 起来之后、回放 rosbag 之前
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
  --qx 0 --qy 0 --qz 0 --qw 1 --frame-id map --child-frame-id base_link \
  --ros-args -p use_sim_time:=true &
```

静态 TF 在 TF2 里是「永远有效」的，所以不受样例包时间戳问题影响。

**实测结果**（三轮独立测试，Orin AGX 64GB，`centerpoint_tiny`）：

| 判据 | 无静态 TF | 有静态 TF |
|---|---|---|
| `Could not find a connection between 'base_link' and 'map'` | 166 | **0** |
| `Fail to preprocess and skip to detect` | 352 | 139 |
| `/perception/object_recognition/detection/centerpoint/objects` 消息数 | **0** | **105 / 135 / 127**（三轮） |
| 累计检测目标 | 0 | 104 / 184 / **202** |

**推理性能**（`.../lidar_centerpoint/debug/processing_time_ms`，n=128）：

```
min = 18.4 ms    中位 = 26.4 ms    p95 = 36.8 ms    max = 306.9 ms（首帧 TRT 预热）
→ 计算侧等效上限 ≈ 37.9 Hz
实际发布周期 中位 = 142.8 ms（7.0 Hz）—— 受输入点云速率限制（bag 以 -r 0.5 回放），不是算力瓶颈
```

26 ms 的中位推理耗时对 10 Hz 激光雷达有充足余量。

**TensorRT 引擎是现场构建的**，可用文件时间戳核对（这是最硬的验证——它同时考验 CUDA 12.8 产物、TRT 10.3、ONNX 解析器与 SM 87 kernel 选择）：

```bash
find ~/autoware_data -name '*.engine' -printf '%TY-%Tm-%Td %TH:%TM  %s  %f\n' | sort
# ml_models/lidar_centerpoint/pts_voxel_encoder_centerpoint_tiny.engine       95 KB
# ml_models/lidar_centerpoint/pts_backbone_neck_head_centerpoint_tiny.engine  10.4 MB
```

> **依赖提示**：`libautoware_lidar_centerpoint_component.so` 直接 `NEEDED libcuda_blackboard.so` —— 所以第 5 节那个一行补丁不打，CenterPoint 会随 `pointcloud_container` 一起崩掉。
> 反过来，`autoware_tensorrt_plugins`（spconv 缺失，坑 #8）**不影响 CenterPoint**，只影响 `autoware_bevfusion` / `autoware_ptv3` / `autoware_tensorrt_vad` / `autoware_diffusion_planner` 这四个包。

**调试话题的消息类型**：`processing_time_ms` / `cyclic_time_ms` 用的是 `autoware_internal_debug_msgs/msg/Float64Stamped`（1.9.0 已从旧的 `tier4_debug_msgs` 迁移，源码里 234 处用新包、仅 1 处用旧包）。用错类型订阅不会报错，只是永远收不到消息。

> **测量方法上的一条教训**：不要用 `ros2 topic hz` 从图外探测。140 个节点的满载 Orin 上，新建 DDS participant 完成发现要几十秒，超时先到，于是"明明在发的话题"全显示无数据——三路雷达只有一路有数据这种自相矛盾的结果就是这么来的。**用图内证据**：读启动日志里各节点自己的告警、`topic_state_monitor` 的判定、`process has died` 计数。

### 反复测试时必须确认"只有一个实例在跑"

`ros2 launch` 把节点作为**孙进程**启动，脚本里常见的 `trap 'kill $(jobs -p)'` 只杀直接子进程，**节点会活下来**。反复测几轮之后就会积累出多个同名节点同时订阅同一话题、往同一话题发布。

实测后果（本文环境累积了 **6 个** `lidar_centerpoint` 实例，最老的存活 5896 秒）：

| 被污染的指标 | 表现 |
|---|---|
| 处理帧数 | 27018 帧 vs 数据实际只有 9047 帧，**虚高 3 倍** |
| 检出计数 | 同一目标被多个实例重复上报 |
| 推理耗时 | 多实例抢 GPU，测出来的延迟不可用 |

**排查**：按 `comm` 过滤会漏——节点的 `comm` 被截断成 `autoware_lidar_`，不匹配 `ros2` / `python3` 这类模式。要按**完整命令行**查：

```bash
ps -eo pid,etimes,args | grep -iE 'centerpoint|robot_state_publisher|static_transform|bag play' | grep -v '[g]rep'
```

**根治**：测试脚本用 `setsid` 启动（脚本即成为进程组长），trap 里杀整个进程组：

```bash
trap 'kill -- -$$ 2>/dev/null' EXIT INT TERM
```

> 顺带一条：`kill` 那一行里**不要出现目标进程名的任何完整字面**。`pgrep -f <name>` 会把执行这条命令的 shell 自己也匹配进去，导致自杀断连。按 PID 杀最安全：先用一条只读命令列出 PID，再用另一条只含数字的命令执行 kill。

---

## 4. 踩过的坑

| # | 症状 | 根因 | 处置 |
|---|---|---|---|
| 1 | colcon 33 秒即退出，0 包被编译，`Duplicate package names not supported` | 工作区内留了旧检出（`src.bak` 之类），同名包出现两次 | 旧检出移出工作区，或放 `COLCON_IGNORE` |
| 2 | `autoware_motion_utils` 失败：`'<anonymous>' may be used uninitialized [-Werror=maybe-uninitialized]` 指向 `return {};` | 系统默认 `gcc` 被 `update-alternatives` 锁在 9.5（常见于为旧 CUDA 迁就的历史配置）；GCC 9 对 `std::optional` 的已知误报 + Autoware 的 `-Werror` | 切 GCC 11（步骤 1）。CMake 3.22 无法用 `CMAKE_COMPILE_WARNING_AS_ERROR` 绕过 |
| 3 | `autoware_pointcloud_preprocessor` / `cuda_blackboard` / `autoware_grid_map_utils` 配置阶段失败：`Could NOT find CUDA: Found unsuitable version "12.6", but required is exact version "12.2"` | 本地编译安装的 OpenCV 在 `OpenCVConfig.cmake` 里写死 `set(OpenCV_CUDA_VERSION "12.2")` 且 `find_host_package(CUDA ... EXACT REQUIRED)`，经 `cv_bridge` 传导到所有用 OpenCV 的包 | 见第 5 节"已知约束" |
| 4 | `cuda_blackboard` 编译失败：`'cudaStreamGetDevice' was not declared in this scope` | Autoware 1.9.0 钉 `cuda_blackboard` 0.4.0，其使用的 API 属 CUDA ≥12.8；JetPack 6.x 只给 12.6 | 装 CUDA 12.8 工具链（步骤 2） |
| 5 | `/usr/local/cuda` 已是 12.8，却仍报 `Found unsuitable version "12.6"` | 该包的 `CMakeCache.txt` 是旧环境下生成的 | `rm -rf build install log` 后重编 |
| 6 | `setup-dev-env.sh` 在 agnocast 角色失败：`gpg: WARNING: unsafe ownership on homedir '/home/lz/.gnupg'` | `~/.gnupg` 属主是 root（历史上某次 `sudo` 带着 `HOME` 跑过 gpg 留下的） | `sudo chown -R $USER:$USER ~/.gnupg && chmod 700 ~/.gnupg` |
| 7 | 488 包全部编译成功，但启动时 15 个 CUDA 节点报 `dlopen error: libcudart.so.12: cannot open shared object file` | `/usr/local/cuda` 切到 12.8（**SBSA 布局** `targets/sbsa-linux/`）后没刷新 ld 缓存，而 `/etc/ld.so.conf.d/000_cuda.conf` 指的是 Tegra 布局 `targets/aarch64-linux/`，缓存里 soname 落在悬空路径上 | `sudo ldconfig` + 给 Autoware 运行环境加 `LD_LIBRARY_PATH=/usr/local/cuda-12.8/targets/sbsa-linux/lib`（步骤 7）。**不要**把 12.8 设成全系统默认 |
| 8 | `source install/setup.bash` 报 `not found: ".../autoware_tensorrt_plugins/share/autoware_tensorrt_plugins/local_setup.bash"` | 该包在 `find_package(spconv)` 失败后 `return()`，没走到 `ament_package()`，所以没生成 `local_setup.bash`（2.6 秒"编完"）。`cumm`/`spconv` 不在 1.9.0 的 repos 里 | 仅影响 bevfusion/transfusion 这类稀疏卷积模型；centerpoint 等不受影响。该告警可忽略，或按需另行引入 spconv |
| 9 | 不报错，但部分话题**静默无数据**且分布不均（三路雷达只有一路有），下游 `gyro_odometer: IMU msg is timeout` 连环报超时 | `net.core.rmem_max` 还是 Ubuntu 默认的 208 KB，140 个节点 + 多 MB 点云走 UDP 回环直接溢出。`setup-dev-env.sh` 不落 DDS sysctl 配置 | 按步骤 8 调 sysctl。**先查缓冲区再怀疑代码** |
| 11 | 编译 488/488 全过，但启动后恰好死 1 个进程（`pointcloud_container`），`ndt_scan_matcher` 报 `No InputSource`，定位初始化报 `align server failed` | `cuda_blackboard` 用了 `cudaStreamGetDevice`，该 API 需要 **CUDA 12.8 驱动**，JetPack 6.x 只有 12.6 → `cudaErrorCallRequiresNewerDriver (36)` → 容器构造函数抛异常 abort | 第 5 节的一行补丁（换成 `cudaGetDevice`）+ 重编 `cuda_blackboard` |
| 12 | 全图刷 `tf2_buffer: Detected jump back in time. Clearing TF buffer.`（单次回放 5.5 万次，`--loop` 下 27 万次） | **官方 `sample-rosbag` 自身时间戳不一致**：包内传感器时间戳是 `1585897272`（2020-04-03），而 bag 消息时间戳是 `1614315746`（2021-02-26），TF 缓冲反复被清空。`--loop` 会让 `/clock` 额外回退，雪上加霜 | 与本次构建无关。功能验证至少别用 `--loop`；要跑通完整定位建议改用自采数据，或按官方教程开 RViz 手动给初始位姿 |
| 13 | 换 CycloneDDS 后 60+ 个节点 `exit code -6` | 自写的 `cyclonedds.xml`（限定 `lo` 接口 + `SocketReceiveBufferSize`）在本环境导致初始化 abort | 回退 FastDDS（步骤 8） |
| 10 | 话题明明在发，`ros2 topic hz` 却说找不到 | `ros2-daemon` 缓存了上一轮启动的拓扑图 | 测量加 `--no-daemon`，或按 PID 停掉守护进程 |

### 关于诊断方法的两条经验

**日志里的"任务标题"不是执行证据。** ansible 对跳过的任务同样打印 `TASK [... : Install cuda-drivers]`，紧随其后的 `skipping:` 才是结论。据标题判断会导致误停健康的流程。

**结论要按结果验证，不按过程推测。** 判断系统是否被改动，靠的是 `dpkg -l` / `readlink -f` / `apt-mark showhold` / `opencv_version --verbose` 这类查询当前状态的命令，而不是"日志里没看到某行"。

---

## 5. 已知约束

### OpenCV 4.8.0 无法用 nvcc 12.8 编译

直觉上应该让 OpenCV 与 Autoware 使用同一个 CUDA 版本。实测**做不到**：用 CUDA 12.8 重编 OpenCV 4.8.0 会在 `cudaarithm` / `cudawarping` 阶段失败，

```
opencv_contrib/modules/cudev/include/opencv2/cudev/grid/detail/reduce.hpp(379):
  error: no instance of overloaded function "cv::cudev::blockReduce" matches the argument list
```

这是 OpenCV 4.8 `cudev` 模块与较新 nvcc 的上游不兼容（4.9/4.10 才修），也正是既有 SOP 用 CUDA 12.2 构建 OpenCV 的原因。

**当前采用的做法**（可用但非终态）：保留按 CUDA 12.2 构建的 OpenCV，仅放宽其 CMake 版本断言，使 Autoware 能够配置通过。

```bash
C=/usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake
sudo cp -n $C $C.bak-cuda122
sudo sed -i 's/set(OpenCV_CUDA_VERSION "12.2")/set(OpenCV_CUDA_VERSION "12.8")/' $C
```

**这个改动没有运行时后果**，因为 Autoware 根本不碰 OpenCV 的 GPU 代码。实测依据（2026-09-22，Orin 上验证）：

| 检查项 | 命令 | 结果 |
|---|---|---|
| 源码 include CUDA 模块 | `grep -rl 'opencv2/cuda' src` | **0** |
| 源码调用 `cv::cuda::` | `grep -rl 'cv::cuda::' src` | **0** |
| CMakeLists 请求 cuda 组件 | `grep -rniE 'cuda(arithm\|warping\|...)' --include=CMakeLists.txt` | **0** |
| 产物依赖 `libopencv_cuda*` | `objdump -p $(readlink -f *.so) \| grep NEEDED` | **0** |
| 产物实际依赖的 OpenCV 模块 | 同上 | 仅 `core/imgproc/imgcodecs/calib3d/highgui/dnn/photo/ximgproc`（39 个库，全 CPU 侧） |

Autoware 的 GPU 推理走 TensorRT + 自写 CUDA kernel，OpenCV 在它这里**是当纯 CPU 图像库用的**。那 10 个 `libopencv_cuda*.so.408` 没有任何调用方。

另外本地这套 OpenCV 用的是 OpenCV 默认的 `CUDA_USE_STATIC_CUDA_RUNTIME=ON`：

```
Extra dependencies: ... cudart_static ... -L/usr/local/cuda-12.2/lib64
```

CUDA 12.2 的 runtime 是**静态**链进去的，68 个 `.so` 没有一个 `NEEDED libcudart` —— 所以既不存在 soname 冲突，那份静态 runtime 也永远不会被初始化。`OpenCV_CUDA_VERSION` 在这里纯粹是一个**编译期断言**。

> 早期版本的本文曾写"进程内可能同时存在两个 CUDA 运行时，跨 OpenCV 边界传 `cudaStream_t`/`GpuMat` 需额外验证"。按上表实测，该担心在 Autoware 这条链上不成立，已更正。

**要不要升级到 OpenCV 4.9/4.10？为了 Autoware 不需要，且代价很高。** soname 会从 `.408` 变成 `.410`，所有 `NEEDED libopencv_*.so.408` 的二进制当场失效：

| 受影响 | 数量 | 能否重编 |
|---|---|---|
| Autoware 库 | 39 | 能，2h23min |
| **GMSL 相机驱动**（`gmsl_node`，`NEEDED libopencv_cudawarping.so.408`） | 1 | 能，但**必须有 OpenCV 的 CUDA 模块** |
| Metavision SDK 5.1.1（含 Python 扩展） | 4 | **不能，厂商预编译** |

第三条是硬阻塞（事件相机链路依赖它）。**只有下面两种情况才真的需要 4.10**：

1. 要在 host 上使用 `cv::cuda::` 且必须与 CUDA 12.8 一同编译（4.8 的 `cudev` 过不了 nvcc 12.8）
2. 要使用 OpenCV DNN 的 CUDA 后端 + cuDNN 9（4.8 只支持 cuDNN 8）

若将来确需 4.10，正确做法是**并存而非替换**：装到 `/opt/opencv-4.10` 独立前缀，用 `OpenCV_DIR` 指给需要它的项目，不触碰 `/usr/lib/aarch64-linux-gnu` 下的 4.8。

### 同机其他项目的兼容性（2026-09-22 实测）

同一台 Orin 上还有两个项目用到 OpenCV / CUDA，都在 JP6.2 + CUDA 12.8 下**实测编译通过**：

| 项目 | 对 OpenCV 的依赖 | 对 CUDA 的依赖 | 结果 |
|---|---|---|---|
| **GMSL_Camera_ROS2** | `find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs cudaimgproc cudawarping)`，源码 `#include <opencv2/cudaimgproc>` / `<opencv2/cudawarping>` | 自有 `.cu`，`enable_language(CUDA)` | `Finished <<< gmsl [25.8s]`，`ldd` 0 未解析 |
| **Remote_Driving_System_Prod** (dev) 的 `vehicle/camera/gst-nvfisheyeundistort/` | **无** | CUDA kernel + GStreamer + NvBufSurface | `make` 一次过，`gst-inspect-1.0 nvfisheyeundistort` 正常识别 |

两条要点：

1. **GMSL 驱动是"本地 4.8-CUDA 不可替换"的根据本身**——它的产物直接 `NEEDED libopencv_cudawarping.so.408`。
2. **前面给 `OpenCVConfig.cmake` 打的 12.8 版本钉，对它同样是必需的**，不只是为了 Autoware：`/usr/local/cuda` 现在指向 12.8，钉子若仍写 12.2，它那句 `find_package(OpenCV REQUIRED COMPONENTS ... cudaimgproc)` 会直接 FATAL_ERROR。两个项目的需求在此**一致，不冲突**。

GMSL 唯一需要补的是环境变量（不改代码）——它 `enable_language(CUDA)` 但未指定编译器，裸跑报 `No CMAKE_CUDA_COMPILER could be found`：

```bash
export CUDA_HOME=/usr/local/cuda-12.8
export PATH=/usr/local/cuda-12.8/bin:$PATH
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=87
```

鱼眼去畸变插件的 Makefile 一个字都不用改：它引用的 `/usr/local/cuda/include` 与 `/usr/local/cuda/lib64` 这两个兼容符号链接，**CUDA 12.8 的 SBSA 包确实创建了**（分别指向 `targets/sbsa-linux/{include,lib}`）。`nvbufsurface.h` 走它自己的 fallback `/usr/src/jetson_multimedia_api/include`（本机未装 DeepStream，正好命中）。

### `cuda_blackboard` 在 JetPack 上必须打补丁（否则点云链路启动即崩）

这是本次功能验证挖出来的**最关键问题**，而且它**不会在编译阶段暴露**：488 个包全部编译成功，`ldd` 零未解析依赖，但一启动就有一个进程死亡，每轮都是同一个：

```
[ERROR] [component_container_mt-1]: process has died [exit code -6,
        cmd '.../component_container_mt --ros-args -r __node:=pointcloud_container ...']

[component_container_mt-1] terminate called after throwing an instance of 'std::runtime_error'
[component_container_mt-1]   what():  cudaErrorCallRequiresNewerDriver (36)
                                      @.../cuda_blackboard/src/cuda_mem_pool_context.cpp#L42
```

`pointcloud_container` 托管裁剪、地面分割、占据栅格与点云拼接。它一死，`/sensing/lidar/concatenated/pointcloud` 就不存在，于是：

```
[WARN] [localization.pose_estimator.ndt_scan_matcher]: No InputSource. Please check the input lidar topic
[WARN] [localization.pose_twist_fusion_filter.ekf_localizer]: The node is not activated. Provide initial pose to pose_initializer
[ERROR] [system.service_log_checker]: /api/localization/initialize: status code 4 'align server failed.'
```

——一串看起来像「传感器配置不对」或「地图不对」的下游症状，**根因却是一行 CUDA 调用**。

**根因**：`cudaStreamGetDevice` 需要 CUDA **12.8 的驱动**，JetPack 6.x 最高只有 12.6 驱动（见第 2 节的实测输出）。装 12.8 工具链解决了编译，但驱动侧没有、也不可能有 12.8（在 Jetson 上装桌面驱动包是禁止的）。

**修法**（一行，语义等价）：流就是紧邻两行前在当前设备上创建的，所以 `cudaGetDevice` 拿到的就是同一个 device。

```cpp
// src/universe/external/cuda_blackboard/src/cuda_mem_pool_context.cpp:42
- CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaStreamGetDevice(stream_, &device_id));
+ CUDA_BLACKBOARD_CHECK_CUDA_ERROR(cudaGetDevice(&device_id));
```

```bash
cd ~/autoware
cp -n src/universe/external/cuda_blackboard/src/cuda_mem_pool_context.cpp{,.bak-orig}
# 改完只需重编这一个包（内部实现变更，ABI 不变，依赖方无需重编）
colcon build --packages-select cuda_blackboard --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=87 \
               -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.8/bin/nvcc
# 约 3 秒
```

> 这是对 `autoware.repos` 拉下来的**外部依赖**的本地修改，升级 Autoware 版本时会被 `vcs import` 覆盖，需要重新施加。建议在自己的 fork 里维护，或写成 patch 文件纳入部署脚本。

### cv_bridge 与 Autoware 使用不同的 OpenCV（既有状态）

`/opt/ros/humble/lib/libcv_bridge.so` 链接 apt 的 `libopencv_core.so.4.5d`，而 Autoware 自己的库链接本地编译的 `.408`。两套 soname 会在同一进程内同时加载，ELF 符号解析按加载顺序走全局表。

这是 Jetson 上「apt ROS + 自编译 OpenCV」的既有状态，不是本次构建引入的，**升级到 4.10 也不能解决**（4.10 一样要和 4.5.4d 并存）。当前 488 个包编译链接全部通过、`ldd` 0 个未解析符号，暂不处理，仅记录。要彻底消除只能从源码重编 `cv_bridge` 使其指向同一套 OpenCV。

注意系统里可能存在**两份** OpenCV 配置，CMake 优先选架构相关路径：

| 路径 | 来源 | CUDA |
|---|---|---|
| `/usr/lib/aarch64-linux-gnu/cmake/opencv4/` | 本地编译安装（`dpkg -S` 查不到归属包） | 有 |
| `/usr/lib/cmake/opencv4/` | apt `libopencv-dev`（JetPack） | 无 |

### cuDNN 8 与 9 并存

JetPack 6.2 装的是 cuDNN 9.3，但 `libcudnn8` 8.9.4 通常仍在（包名不同，apt 不会替换）。未版本化的 `/usr/include/cudnn_version.h` 经 alternatives 指向 v9。若要为 OpenCV 4.8 启用 DNN-CUDA 后端，需显式指向 v8——但 Autoware 用 TensorRT 推理、不使用 OpenCV 的 DNN 后端，通常可直接 `WITH_CUDNN=OFF`。

### `autoware_individual_params` 不在 1.9.0 的 repos 清单内 —— 这是正常的，**不要去补**

1.9.0 的 `repositories/autoware.repos` 只含 32 个仓库，`ros2 pkg prefix autoware_individual_params` 会报 MISSING。

**这不是缺漏。**该仓库已于 2025 年归档（[autoware#5975](https://github.com/autowarefoundation/autoware/issues/5975)），车辆相关参数已迁入各 sensor kit 自己的 description 包：

```
src/launcher/sample_sensor_kit_launch/sample_sensor_kit_description/config/
  ├── imu_corrector.param.yaml
  ├── sensor_kit_calibration.yaml
  └── sensors_calibration.yaml
```

实测 1.9.0 全部源码中对 `individual_params` 的引用数为 **0**：

```bash
grep -rl 'individual_params' src --include=*.xml --include=*.py --include=*.yaml | wc -l   # 0
```

去 clone 那个归档仓库只会在工作区里多出一个无人引用的 `individual_params` 包（注意它的包名没有 `autoware_` 前缀），并且有可能与新位置的参数产生混淆。自定义车型请在自己的 `<vehicle>_sensor_kit_description` 里放标定参数，不要复活 individual_params 这条路径。

---

## 6. 排错索引

| 看到什么 | 去哪一节 |
|---|---|
| `Duplicate package names not supported` | 坑 #1 |
| `-Werror=maybe-uninitialized` 指向 `return {};` | 坑 #2（换 GCC 11） |
| `Could NOT find CUDA: ... required is exact version` | 坑 #3 / 第 5 节 |
| `cudaStreamGetDevice was not declared` | 坑 #4（装 CUDA 12.8） |
| `/usr/local/cuda` 版本与报错不符 | 坑 #5（清构建树） |
| `gpg: unsafe ownership on homedir` | 坑 #6 |
| `install/` 里 `.so` 只有 92 字节 | 步骤 6 的说明（跟进 symlink） |
| `dlopen error: libcudart.so.12: cannot open shared object file` | 坑 #7 / 步骤 7（ld 缓存 + `LD_LIBRARY_PATH`） |
| `not found: ".../autoware_tensorrt_plugins/.../local_setup.bash"` | 坑 #8（spconv 缺失，可忽略） |
| 话题静默无数据 / `IMU msg is timeout` / 三路雷达只有一路有 | 坑 #9 / 步骤 8（UDP 缓冲区 208 KB） |
| `ros2 topic hz` 找不到明明在发的话题 | 坑 #10（加 `--no-daemon`） |
| `cudaErrorCallRequiresNewerDriver (36)` | 坑 #11 / 第 5 节（`cuda_blackboard` 一行补丁） |
| `pointcloud_container` 启动即死 / `No InputSource` / `align server failed` | 坑 #11（同上，这些都是下游症状） |
| `tf2_buffer: Detected jump back in time` 刷屏 | 坑 #12（别用 `--loop`） |
| 换 CycloneDDS 后节点大面积 `exit code -6` | 坑 #13（回退 FastDDS） |
| `blockReduce`/`blockReduceKeyVal` 重载解析失败 | 第 5 节（OpenCV + nvcc 12.8 不兼容） |

---

*本文所有版本号与结论均来自 2026-09-22 在 lz@192.168.8.3（AGX Orin 开发套件）上的实测。*
