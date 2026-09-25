#!/usr/bin/env python3
"""点云地图质量体检: 地面平面的整体倾角, 以及沿行驶方向分段的倾角走势。

判读:
  - 各段倾角都小且**接近常数**  -> 可用 / 或一次 SE(3) 旋转即可校平(恒定初始姿态误差)
  - 倾角沿行程**单调增长**      -> 累积漂移, 旋转救不回, 必须换建图方案
用法: check_map.py <pcd> [段数] [odo.csv]

⚠ 给了 odo.csv 就按**轨迹弧长**分段, 否则退化成按最长坐标轴分段。
L 形/带转弯的路线**必须**给 odo.csv —— 按 x 分段会把路线上高程不同的两段揉进同一个 bin,
拟合出来的"倾角/地面高度"是假的(2026-09-24 实测: 全程 L 形路线按 x 分段拟出 z0=−30.58 m)。
odo.csv 必须与该 pcd 同坐标系, 即用**配准前**的原始建图输出。
"""
import sys, numpy as np

path = sys.argv[1]
nseg = int(sys.argv[2]) if len(sys.argv) > 2 else 6
odo_csv = sys.argv[3] if len(sys.argv) > 3 else None

with open(path, 'rb') as f:
    skip = 0
    for _ in range(30):
        line = f.readline().decode('ascii', 'replace')
        skip += 1
        if line.startswith('DATA'):
            break
a = np.loadtxt(path, skiprows=skip, usecols=(0, 1, 2))
x, y, z = a.T
print(f'{path}\n  点数 {len(a)}  '
      f'x∈[{x.min():.1f},{x.max():.1f}]  y∈[{y.min():.1f},{y.max():.1f}]  z∈[{z.min():.1f},{z.max():.1f}]')


def ground_plane(P):
    """每个 2 m xy 网格取最低点作地面候选, 去掉最高的 10%, 最小二乘拟合平面。"""
    if len(P) < 500:
        return None
    g = np.floor(P[:, :2] / 2.0).astype(np.int64)
    key = g[:, 0] * 100003 + g[:, 1]
    order = np.lexsort((P[:, 2], key))
    Ps, ks = P[order], key[order]
    first = np.ones(len(ks), bool)
    first[1:] = ks[1:] != ks[:-1]
    G = Ps[first]
    if len(G) < 60:
        return None
    G = G[G[:, 2] < np.percentile(G[:, 2], 90)]
    A = np.c_[G[:, 0], G[:, 1], np.ones(len(G))]
    c, *_ = np.linalg.lstsq(A, G[:, 2], rcond=None)
    n = np.array([-c[0], -c[1], 1.0])
    n /= np.linalg.norm(n)
    resid = G[:, 2] - (A @ c)
    return n, np.degrees(np.arccos(abs(n[2]))), len(G), c[2], np.std(resid)


r = ground_plane(a)
print(f'\n整图地面: 倾角={r[1]:.2f}°  法向=({r[0][0]:+.4f},{r[0][1]:+.4f},{r[0][2]:.4f})  '
      f'截距 z0={r[3]:.2f} m  残差 std={r[4]:.2f} m  地面点={r[2]}')
print(f'z 的 1%/50%/99% 分位: {np.percentile(z,1):.2f} / {np.percentile(z,50):.2f} / {np.percentile(z,99):.2f}')

if odo_csv:
    from scipy.spatial import cKDTree
    o = np.genfromtxt(odo_csv, delimiter=',', names=True)
    T = np.c_[o['x'], o['y']]
    keep = np.r_[True, np.linalg.norm(np.diff(T, axis=0), axis=1).cumsum() // 2.0 > 0]
    # 每 ~2 m 取一个轨迹点, 并算其弧长
    d = np.r_[0.0, np.linalg.norm(np.diff(T, axis=0), axis=1).cumsum()]
    idx = np.unique(np.searchsorted(d, np.arange(0, d[-1], 2.0)))
    Ts, arc = T[idx], d[idx]
    coord = np.empty(len(a))
    tree = cKDTree(Ts)
    for i in range(0, len(a), 200000):
        coord[i:i+200000] = arc[tree.query(a[i:i+200000, :2], k=1)[1]]
    label = f'轨迹弧长(总 {d[-1]:.0f} m, {len(Ts)} 个采样点)'
else:
    axis = 0 if (x.max() - x.min()) >= (y.max() - y.min()) else 1
    coord = a[:, axis]
    label = f'{"x" if axis==0 else "y"} 轴  ⚠ 未给 odo.csv, 弯道路线此读数不可信'
print(f'\n沿 {label} 分 {nseg} 段:')
print(f'{"段":>3} {"范围":>18} {"点数":>8} {"倾角°":>7} {"法向(nx,ny)":>20} {"地面z0":>8} {"残差":>6}')
edges = np.percentile(coord, np.linspace(0, 100, nseg + 1))
tilts, prev = [], None
for i in range(nseg):
    m = (coord >= edges[i]) & ((coord < edges[i+1]) if i < nseg-1 else (coord <= edges[i+1]))
    s = ground_plane(a[m])
    if s is None:
        print(f'{i:>3}  点太少'); continue
    n, tilt, ng, z0, rs = s
    d = '' if prev is None else f'  Δ={np.degrees(np.arccos(np.clip(prev@n,-1,1))):.2f}°'
    print(f'{i:>3} [{edges[i]:7.1f},{edges[i+1]:7.1f}] {m.sum():>8} {tilt:>7.2f} '
          f'({n[0]:+.4f},{n[1]:+.4f}) {z0:>8.2f} {rs:>6.2f}{d}')
    tilts.append(tilt); prev = n

if len(tilts) >= 3:
    k = np.polyfit(np.arange(len(tilts)), tilts, 1)[0]
    print(f'\n倾角随段号的线性斜率: {k:+.2f}°/段  (整段变化 {k*(len(tilts)-1):+.2f}°)')
    if abs(k * (len(tilts) - 1)) > 1.0:
        print('  判读: 倾角沿行程显著变化 -> **累积漂移**, 一次旋转救不回')
    elif max(tilts) > 1.0:
        print('  判读: 倾角基本恒定但偏大 -> 初始姿态误差, 一次 SE(3) 旋转可校平')
    else:
        print('  判读: 地面平整且各段一致 -> 可用')
