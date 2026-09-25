#!/usr/bin/env python3
"""把 FAST-LIO 建的图配准到 Autoware 的 map 系, 并生成 map_projector_info.yaml。

做四件事:
  1. 用地图自身的地面平面做一次 SE(3) 校平(法向转到 +z)。**仅当倾角沿行程基本恒定时才合法**,
     先用 check_map.py 判读; 若是累积漂移, 旋转救不回, 应换建图方案而不是在这里补。
     旋转同时施加到里程计轨迹, 否则后面的 SE(2) 配准会与地图不一致。
  2. 用同段轨迹的 FAST-LIO 里程计与 GNSS 拟合 SE(2)(平面旋转+平移, 不缩放), RANSAC 抗粗差
  3. 把地面平面平移到 z=0 (Autoware 的 base_link 在地面上)
  4. 写出 map_origin 的经纬高: 高程取 GNSS 中位数减去天线高, 使 GNSS 给出的 base_link z ≈ 0

用法: georef_map.py <odo.csv> <gps.csv> <输入pcd> <输出目录> [天线高m]
"""
import sys, os, math, numpy as np

odo_csv, gps_csv, pcd_in, out_dir = sys.argv[1:5]
ANT_H = float(sys.argv[5]) if len(sys.argv) > 5 else 2.4   # imu_link(GNSS 天线) 距地面
# 诊断用: 只拿开头这段时间的轨迹附近的地面点来拟合校平(地图点仍全部保留)。
# 用来判别"整图无法用一次刚性旋转校平"是不是 NDT 在起点区变差的机制。
LEVEL_WIN_S = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

odo = np.genfromtxt(odo_csv, delimiter=',', names=True)
gps = np.genfromtxt(gps_csv, delimiter=',', names=True)
print(f'odo {len(odo)} 条, gps {len(gps)} 条')

# ---- 读地图 ---------------------------------------------------------------
with open(pcd_in, 'rb') as fh:
    skip = 0
    for _ in range(30):
        line = fh.readline().decode('ascii', 'replace'); skip += 1
        if line.startswith('DATA'):
            break
P = np.loadtxt(pcd_in, skiprows=skip)
print(f'地图 {len(P)} 点')


def ground_points(Q):
    """每个 2 m xy 网格取最低点, 去掉最高的 10%。"""
    g = np.floor(Q[:, :2] / 2.0).astype(np.int64)
    key = g[:, 0] * 100003 + g[:, 1]
    order = np.lexsort((Q[:, 2], key))
    Qs, ks = Q[order], key[order]
    first = np.ones(len(ks), bool); first[1:] = ks[1:] != ks[:-1]
    G = Qs[first]
    return G[G[:, 2] < np.percentile(G[:, 2], 90)]


def fit_normal(G):
    A = np.c_[G[:, 0], G[:, 1], np.ones(len(G))]
    c, *_ = np.linalg.lstsq(A, G[:, 2], rcond=None)
    n = np.array([-c[0], -c[1], 1.0]); n /= np.linalg.norm(n)
    return n, c[2]


# ---- 1. SE(3) 校平: 地面法向转到 +z ---------------------------------------
G_all = ground_points(P)
if LEVEL_WIN_S > 0:
    from scipy.spatial import cKDTree
    T = np.c_[odo['x'], odo['y']][odo['t'] <= odo['t'][0] + LEVEL_WIN_S]
    sel = cKDTree(T[::20]).query(G_all[:, :2], k=1)[0] < 80.0
    print(f'局部校平: 只用前 {LEVEL_WIN_S:.0f} s 轨迹 80 m 内的地面点 '
          f'{sel.sum()}/{len(G_all)}')
    G_all = G_all[sel]
n0, _ = fit_normal(G_all)
tilt0 = math.degrees(math.acos(abs(n0[2])))
v = np.cross(n0, [0.0, 0.0, 1.0])
sn, cs = np.linalg.norm(v), float(np.dot(n0, [0.0, 0.0, 1.0]))
if sn < 1e-9:
    Rlev = np.eye(3)
else:
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    Rlev = np.eye(3) + K + K @ K * ((1 - cs) / sn**2)
P[:, :3] = P[:, :3] @ Rlev.T
otraj = np.c_[odo['x'], odo['y'], odo['z']] @ Rlev.T      # 轨迹同步旋转
n1, _ = fit_normal(ground_points(P))
print(f'SE(3) 校平: 倾角 {tilt0:.2f}° -> {math.degrees(math.acos(abs(n1[2]))):.2f}°')

# ---- GNSS -> 局部 ENU (以首个定位为原点) ----------------------------------
lat0, lon0 = float(np.median(gps['lat'][:20])), float(np.median(gps['lon'][:20]))
a, f = 6378137.0, 1/298.257223563
e2 = f * (2 - f)
s = math.sin(math.radians(lat0))
Rn = a / math.sqrt(1 - e2 * s * s)                 # 卯酉圈曲率半径
Rm = a * (1 - e2) / (1 - e2 * s * s) ** 1.5        # 子午圈曲率半径
east = np.radians(gps['lon'] - lon0) * Rn * math.cos(math.radians(lat0))
north = np.radians(gps['lat'] - lat0) * Rm

# ---- 时间对齐: 把里程计插值到 GNSS 时刻 ------------------------------------
t_lo, t_hi = odo['t'].min(), odo['t'].max()
if LEVEL_WIN_S > 0:
    t_hi = min(t_hi, t_lo + LEVEL_WIN_S)      # 诊断: SE(2) 也只用同一窗口
    print(f'局部配准: SE(2) 只用前 {LEVEL_WIN_S:.0f} s 的轨迹')
m = (gps['t'] >= t_lo) & (gps['t'] <= t_hi)
if m.sum() < 20:
    sys.exit(f'可配对样本太少({m.sum()}), 无法配准')
ox = np.interp(gps['t'][m], odo['t'], otraj[:, 0])
oy = np.interp(gps['t'][m], odo['t'], otraj[:, 1])
ge, gn = east[m], north[m]
print(f'可配对 {m.sum()} 点, 里程计时间跨度 {t_hi-t_lo:.1f} s')


def kabsch_se2(src, dst):
    """求 R,t 使 R@src + t ≈ dst (不缩放)。"""
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    return R, cd - R @ cs


src = np.c_[ox, oy]; dst = np.c_[ge, gn]
best, rng = None, np.random.default_rng(0)
THR = 3.0                      # 内点阈值 m (GNSS σ≈2.9 m)
for _ in range(400):
    idx = rng.choice(len(src), 12, replace=False)
    R, t = kabsch_se2(src[idx], dst[idx])
    r = np.linalg.norm(src @ R.T + t - dst, axis=1)
    nin = (r < THR).sum()
    if best is None or nin > best[0]:
        best = (nin, r < THR)
inl = best[1]
R, t = kabsch_se2(src[inl], dst[inl])              # 用全部内点重估
res = np.linalg.norm(src @ R.T + t - dst, axis=1)
yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
print(f'SE(2) 配准: 内点 {inl.sum()}/{len(src)}  yaw={yaw:+.3f}°  '
      f'残差 中位={np.median(res[inl]):.2f} m 均值={res[inl].mean():.2f} m 最大={res[inl].max():.2f} m')

# ---- 施加 SE(2), 再把地面移到 z=0 ------------------------------------------
xy = P[:, :2] @ R.T + t
z0 = float(np.median(ground_points(P)[:, 2]))
print(f'地面中位高度 z0={z0:.2f} m -> 平移到 0')

os.makedirs(out_dir, exist_ok=True)
out_pcd = os.path.join(out_dir, 'pointcloud_map.pcd')
n = len(P)
with open(out_pcd, 'w') as fh:
    fh.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n')
    fh.write('FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
    fh.write(f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n')
    np.savetxt(fh, np.c_[xy, P[:, 2] - z0, P[:, 3]], fmt='%.3f %.3f %.3f %.1f')
print(f'已写出 {out_pcd}: {n} 点')

alt = float(np.median(gps['alt'])) - ANT_H          # 地面的椭球高
with open(os.path.join(out_dir, 'map_projector_info.yaml'), 'w') as fh:
    fh.write('projector_type: LocalCartesianUTM\nvertical_datum: WGS84\nmap_origin:\n')
    fh.write(f'  latitude: {lat0:.10f}\n  longitude: {lon0:.10f}\n  altitude: {alt:.2f}\n')
print(f'map_origin: lat={lat0:.7f} lon={lon0:.7f} alt={alt:.2f} '
      f'(GNSS 中位 {np.median(gps["alt"]):.2f} − 天线高 {ANT_H})')

osm = os.path.join(out_dir, 'lanelet2_map.osm')
if not os.path.exists(osm):
    with open(osm, 'w') as fh:
        fh.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        fh.write('<!-- 占位矢量地图: NDT 定位不需要, 但 map_loader 会加载。规划需要手绘车道网络。 -->\n')
        fh.write('<osm version="0.6" generator="placeholder">\n')
        fh.write('  <MetaInfo format_version="1.0" map_version="1"/>\n')
        fh.write(f'  <node id="1" lat="{lat0:.10f}" lon="{lon0:.10f}">\n    <tag k="ele" v="0.0"/>\n  </node>\n')
        fh.write('</osm>\n')
    print(f'已写占位 {osm}')
