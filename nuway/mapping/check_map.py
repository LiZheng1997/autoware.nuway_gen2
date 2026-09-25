#!/usr/bin/env python3
"""Point-cloud map quality check: overall tilt of the ground plane, and how tilt trends across
segments along the direction of travel.

Diagnosis:
  - Tilt is small in every segment and **roughly constant**  -> usable, or a single SE(3)
    rotation can level it out (constant initial-attitude error)
  - Tilt **monotonically grows** along the route                -> cumulative drift, a rotation
    can't fix it, a different mapping approach is required
Usage: check_map.py <pcd> [n_segments] [odo.csv]

Note: if odo.csv is given, segments are cut by **trajectory arc length**; otherwise it falls
back to binning by the longest coordinate axis.
An L-shaped or turning route **must** be given odo.csv -- binning by x lumps two segments of the
route that sit at different elevations into the same bin, and the fitted "tilt/ground height"
comes out fake (measured 2026-09-24: for a full L-shaped route binned by x, the fit gave
z0=-30.58 m).
odo.csv must be in the same coordinate frame as this pcd, i.e. the raw mapping output
**before registration**.
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
print(f'{path}\n  points {len(a)}  '
      f'x∈[{x.min():.1f},{x.max():.1f}]  y∈[{y.min():.1f},{y.max():.1f}]  z∈[{z.min():.1f},{z.max():.1f}]')


def ground_plane(P):
    """Takes the lowest point in each 2 m xy grid cell as a ground candidate, drops the top 10%,
    and least-squares fits a plane."""
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
print(f'\nWhole-map ground: tilt={r[1]:.2f}°  normal=({r[0][0]:+.4f},{r[0][1]:+.4f},{r[0][2]:.4f})  '
      f'intercept z0={r[3]:.2f} m  residual std={r[4]:.2f} m  ground points={r[2]}')
print(f'z percentiles 1%/50%/99%: {np.percentile(z,1):.2f} / {np.percentile(z,50):.2f} / {np.percentile(z,99):.2f}')

if odo_csv:
    from scipy.spatial import cKDTree
    o = np.genfromtxt(odo_csv, delimiter=',', names=True)
    T = np.c_[o['x'], o['y']]
    keep = np.r_[True, np.linalg.norm(np.diff(T, axis=0), axis=1).cumsum() // 2.0 > 0]
    # Take one trajectory point every ~2 m, and compute its arc length
    d = np.r_[0.0, np.linalg.norm(np.diff(T, axis=0), axis=1).cumsum()]
    idx = np.unique(np.searchsorted(d, np.arange(0, d[-1], 2.0)))
    Ts, arc = T[idx], d[idx]
    coord = np.empty(len(a))
    tree = cKDTree(Ts)
    for i in range(0, len(a), 200000):
        coord[i:i+200000] = arc[tree.query(a[i:i+200000, :2], k=1)[1]]
    label = f'trajectory arc length (total {d[-1]:.0f} m, {len(Ts)} samples)'
else:
    axis = 0 if (x.max() - x.min()) >= (y.max() - y.min()) else 1
    coord = a[:, axis]
    label = f'{"x" if axis==0 else "y"} axis  ⚠ no odo.csv given, this reading is unreliable for a turning route'
print(f'\nSplitting into {nseg} segments along {label}:')
print(f'{"seg":>3} {"range":>17} {"points":>8} {"tilt°":>7} {"normal(nx,ny)":>17} {"z0":>8} {"resid":>6}')
edges = np.percentile(coord, np.linspace(0, 100, nseg + 1))
tilts, prev = [], None
for i in range(nseg):
    m = (coord >= edges[i]) & ((coord < edges[i+1]) if i < nseg-1 else (coord <= edges[i+1]))
    s = ground_plane(a[m])
    if s is None:
        print(f'{i:>3}  too few points'); continue
    n, tilt, ng, z0, rs = s
    d = '' if prev is None else f'  Δ={np.degrees(np.arccos(np.clip(prev@n,-1,1))):.2f}°'
    print(f'{i:>3} [{edges[i]:7.1f},{edges[i+1]:7.1f}] {m.sum():>8} {tilt:>7.2f} '
          f'({n[0]:+.4f},{n[1]:+.4f}) {z0:>8.2f} {rs:>6.2f}{d}')
    tilts.append(tilt); prev = n

if len(tilts) >= 3:
    k = np.polyfit(np.arange(len(tilts)), tilts, 1)[0]
    print(f'\nLinear slope of tilt vs. segment index: {k:+.2f}°/seg  (total change over the route {k*(len(tilts)-1):+.2f}°)')
    if abs(k * (len(tilts) - 1)) > 1.0:
        print('  Diagnosis: tilt changes significantly along the route -> **cumulative drift**, a rotation cannot fix this')
    elif max(tilts) > 1.0:
        print('  Diagnosis: tilt is roughly constant but large -> initial-attitude error, a single SE(3) rotation can level it')
    else:
        print('  Diagnosis: ground is flat and consistent across segments -> usable')
