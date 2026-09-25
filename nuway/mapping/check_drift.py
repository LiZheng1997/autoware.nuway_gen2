#!/usr/bin/env python3
"""Measures LIO's cumulative drift using an out-and-back revisit -- no loop-closure software and
no ground truth needed.

Method: this route is an out-and-back along the same corridor (GNSS measured the return path's
median distance to the outbound trajectory at 1.52 m). For every timestamp on the return leg,
use **GNSS** to find the matching physical location on the outbound leg, then take the
difference between the two timestamps' **odometry** positions -- that difference is the
cumulative drift accumulated up to that location.
G1 = start/end loop-closure error (odometry vs GNSS), G3 = drift-vs-distance-traveled curve.
Usage: check_drift.py <odo.csv> <gps.csv>
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

# Interpolate odometry to GNSS timestamps
t = gps['t']
m = (t >= odo['t'].min()) & (t <= odo['t'].max())
t, E, N = t[m], E[m], N[m]
OX = np.interp(t, odo['t'], odo['x'])
OY = np.interp(t, odo['t'], odo['y'])
OZ = np.interp(t, odo['t'], odo['z'])

gdist = np.hypot(E, N)
far = int(np.argmax(gdist))                     # turnaround point
print(f'{len(t)} samples, duration {t[-1]-t[0]:.0f} s, turnaround at t={t[far]-t[0]:.0f}s ({gdist[far]:.1f} m from start)')

# ---- G1 start/end closure --------------------------------------------------
d_odo = math.hypot(OX[-1]-OX[0], OY[-1]-OY[0])
d_gps = math.hypot(E[-1]-E[0], N[-1]-N[0])
path = np.hypot(np.diff(E), np.diff(N)).sum()
print(f'\nG1 start/end closure: odometry {d_odo:.2f} m  GNSS {d_gps:.2f} m  '
      f'diff {abs(d_odo-d_gps):.2f} m / path {path:.0f} m = {100*abs(d_odo-d_gps)/path:.3f}%')
print(f'   z start/end diff: {OZ[-1]-OZ[0]:+.2f} m')

# ---- G3 drift vs. distance traveled -----------------------------------------
out = np.c_[E[:far], N[:far]]
print(f'\nG3 for each return-leg point, GNSS-match it to the same outbound location and diff the odometry:')
print(f'{"back_t(s)":>9} {"match_d(m)":>10} {"drift_h(m)":>10} {"dz(m)":>9} {"dist0(m)":>8}')
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
    print(f'\n   horizontal drift: median={np.median(H):.2f} m max={H.max():.2f} m')
    print(f'   z          drift: median={np.median(Z):+.2f} m max_abs={np.abs(Z).max():.2f} m')
    print(f'   z drift near start (<50 m from start): '
          + ', '.join(f'{r[3]:+.2f}' for r in rows if r[4] < 50) )
