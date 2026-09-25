#!/usr/bin/env python3
"""用往返重访量 LIO 的累积漂移 —— 不需要回环软件, 也不需要真值。

原理: 这条路线是同一走廊往返(GNSS 实测回程点到去程轨迹中位 1.52 m)。
把回程的每个时刻用 **GNSS** 找到去程的同一物理位置, 再看两个时刻的 **里程计** 位置差,
这个差就是该位置处累积的漂移。
G1 = 首尾闭合误差(里程计 vs GNSS), G3 = 漂移随路程的曲线。
用法: check_drift.py <odo.csv> <gps.csv>
"""
import sys, math, numpy as np

odo = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
gps = np.genfromtxt(sys.argv[2], delimiter=',', names=True)

lat0, lon0 = float(gps['lat'][0]), float(gps['lon'][0])
a, f = 6378137.0, 1/298.257223563
e2 = f * (2 - f); s = math.sin(math.radians(lat0))
Rn = a / math.sqrt(1 - e2*s*s); Rm = a * (1 - e2) / (1 - e2*s*s)**1.5
E = np.radians(gps['lon'] - lon0) * Rn * math.cos(math.radians(lat0))
N = np.radians(gps['lat'] - lat0) * Rm

# 里程计插值到 GNSS 时刻
t = gps['t']
m = (t >= odo['t'].min()) & (t <= odo['t'].max())
t, E, N = t[m], E[m], N[m]
OX = np.interp(t, odo['t'], odo['x'])
OY = np.interp(t, odo['t'], odo['y'])
OZ = np.interp(t, odo['t'], odo['z'])

gdist = np.hypot(E, N)
far = int(np.argmax(gdist))                     # 折返点
print(f'样本 {len(t)} 条, 时长 {t[-1]-t[0]:.0f} s, 折返点 t={t[far]-t[0]:.0f}s (离起点 {gdist[far]:.1f} m)')

# ---- G1 首尾闭合 ----------------------------------------------------------
d_odo = math.hypot(OX[-1]-OX[0], OY[-1]-OY[0])
d_gps = math.hypot(E[-1]-E[0], N[-1]-N[0])
path = np.hypot(np.diff(E), np.diff(N)).sum()
print(f'\nG1 首尾闭合: 里程计 {d_odo:.2f} m  GNSS {d_gps:.2f} m  '
      f'差 {abs(d_odo-d_gps):.2f} m / 里程 {path:.0f} m = {100*abs(d_odo-d_gps)/path:.3f}%')
print(f'   z 首尾差: {OZ[-1]-OZ[0]:+.2f} m')

# ---- G3 漂移随路程 --------------------------------------------------------
out = np.c_[E[:far], N[:far]]
print(f'\nG3 回程每点用 GNSS 匹配去程同位置, 比里程计之差:')
print(f'{"回程t(s)":>9} {"GNSS配对距":>10} {"水平漂移m":>10} {"z漂移m":>9} {"离起点m":>8}')
rows = []
for i in range(far, len(t), max(1, (len(t)-far)//12)):
    d = np.hypot(out[:, 0]-E[i], out[:, 1]-N[i])
    j = int(np.argmin(d))
    if d[j] > 8.0:
        continue
    dh = math.hypot(OX[i]-OX[j], OY[i]-OY[j])
    dz = OZ[i] - OZ[j]
    rows.append((t[i]-t[0], d[j], dh, dz, gdist[i]))
    print(f'{rows[-1][0]:>9.0f} {d[j]:>10.2f} {dh:>10.2f} {dz:>9.2f} {gdist[i]:>8.1f}')
if rows:
    H = np.array([r[2] for r in rows]); Z = np.array([r[3] for r in rows])
    print(f'\n   水平漂移: 中位={np.median(H):.2f} m 最大={H.max():.2f} m')
    print(f'   z   漂移: 中位={np.median(Z):+.2f} m 最大绝对={np.abs(Z).max():.2f} m')
    print(f'   起点区(离起点<50 m)的 z 漂移: '
          + ', '.join(f'{r[3]:+.2f}' for r in rows if r[4] < 50) )
