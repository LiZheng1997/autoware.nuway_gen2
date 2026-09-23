# nUWAy Autoware 1.9.0 — Orin 原生编译分支

本分支基于上游 **tag `1.9.0`**（`10718787`, 2026-07-15），不是 `main`。
1.9.0 是 tag 不是分支，且**标签名没有 `v` 前缀**。

与 `uwa-rev/autoware_on_nUWAy` 的 Docker 双机方案并行，本分支走
**Orin AGX 单机原生编译**：版本锁死、可复现，不依赖滚动的 universe-devel 镜像。

## 快速开始

```bash
git clone -b nuway/1.9.0 git@github.com:LiZheng1997/autoware.nuway_gen2.git
cd autoware.nuway_gen2

mkdir -p src
vcs import src < repositories/autoware.repos   # 上游 32 仓库，全部钉死 tag/hash
vcs import src < repositories/nuway.repos      # nuway_canbridge

bash nuway/stage_packages.sh                   # copy bundled sensor_kit / vehicle into src/
bash patches/apply.sh                          # ★ 必须，见下

sudo cp nuway/60-autoware-dds.conf /etc/sysctl.d/ && sudo sysctl --system
./setup-dev-env.sh universe -y --no-nvidia     # CUDA 12.8 需单独装，见 docs
bash nuway/build.sh                            # 约 2h23min，488+7 包
source nuway/setup_env.sh
```

## 为什么 `patches/apply.sh` 不能省

`cuda_blackboard` 调用 `cudaStreamGetDevice`，该 API **运行时需要 CUDA 12.8 驱动**，
而 JetPack 6.x 最高只有 12.6 驱动 → 返回 `cudaErrorCallRequiresNewerDriver (36)`
→ `pointcloud_container` 启动即 abort → 点云链路整条消失。

下游症状极具误导性（`ndt_scan_matcher: No InputSource`、
`align server failed`），看起来像传感器或地图配置错，根因却是一行 CUDA 调用。

它是 `vcs import` 拉下来的**外部依赖**，每次 import 都会被覆盖，
所以必须以 patch 形式维护、每次重新施加。

## 本分支相对上游的增量

| 路径 | 内容 |
|---|---|
| `repositories/nuway.repos` | nuway_canbridge（`canbridge_cpp@autoware`），URL 已从 `lee.github.com` 别名改为标准形式 |
| `nuway_packages/nuway_sensor_kit_launch/` | 传感器套件（VLP-16 ×2 / 相机 ×2 / GNSS / IMU / ntrip） |
| `nuway_packages/nuway_vehicle_launch/` | 车辆描述 |
| `patches/` | cuda_blackboard 一行补丁 + 应用脚本 |
| `nuway/` | 环境变量、DDS sysctl、编译脚本 |
| `docs/build-on-orin.md` | 完整 SOP：10 个步骤 + 13 个踩坑 + 排错索引 |

nuway 套件原有的 `common_sensor_launch` 已删除——1.9.0 的 `autoware_launch` 自带同名包（版本更新，0.52.0 vs 0.50.0，差异是上游重构），重复会让 colcon 中止。其中唯一的 nuway 定制（失真校正阈值）以 `patches/0002` 保留。

`nuway_packages/` 带有 `COLCON_IGNORE`，避免 colcon 同时发现
它和 `src/nuway/` 里的副本而报 Duplicate package names。其下两个包来源 `uwa-rev/autoware_launch.nuway @ d11bea6 (2026-03-30)`，
**随本仓库分发而非 vcs import**：该仓库同时包含一个 fork 版 `autoware_launch`，
import 进来会与 1.9.0 自带的同名包冲突，导致 colcon
`Duplicate package names not supported` 直接中止。建议后续拆成独立仓库。

## common_cuda_sensor_launch 的定位

零代码的纯集成包（`ament_auto_package(INSTALL_TO_SHARE launch config)`），
算法全部来自 TIER IV 上游的 `autoware_cuda_pointcloud_preprocessor`。

上游 README 写明该包的用途就是「用 GPU 版重新实现 `autoware_pointcloud_preprocessor`
的裁剪 / 去畸变 / 环外点滤波」，所以 nuway 把 CPU 四件套注释掉换成单个 CUDA 节点，
是跟随上游设计而非自创。

nuway 提供的是上游 1.9.0 尚未提供的两件事：

1. 把 CUDA 节点**组合进传感器容器**（与驱动同进程，多 MB 点云零拷贝）。
   上游只有独立节点的 launch，输出话题默认值是 `test`，属冒烟测试而非生产接线。
2. **实车车体裁剪框**（`crop_box.x∈[-0.5, 2.83]`、`y∈±0.748`、`z∈[0, 2.4]`、
   `negative: true`）。上游的 `crop_box.*` 全是 0。

已删除与上游逐字节相同、或未被 launch 加载的文件：
`robosense_Bpearl/Helios.launch.xml`、`ring_outlier_filter_node.param.yaml`、
`distortion_corrector_node.param.yaml`。升级 Autoware 时 diff 面越小越好。

## 已知待办

- [ ] **车辆几何尺寸待核对**：`vehicle_info.param.yaml` 的几何项与 Autoware
      `sample_vehicle` 相同（仅 `max_steer_angle` 不同）。`max_steer_angle: 0.70`
      已由实车测试确认，沿用。CAN 侧原始限制见 `VEHICLE_LIMITS.md`
- [ ] **点云格式**：1.9.0 的 `concatenate_and_time_sync_node` 要求 `PointXYZIRC`，
      而现有录包是 `PointXYZIRT`（intensity float32 + ring + time）。需确认
      实车驱动输出格式
- [ ] **EZ10 是前后双轴转向**，Autoware 默认前轮转向自行车模型，转弯半径偏保守
- [ ] `autoware_launch.nuway` 的调参成果（`nuway_preset.yaml` 等）尚未迁移，
      需与 1.9.0 的 config 逐项比对
