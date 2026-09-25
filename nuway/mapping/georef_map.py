#!/usr/bin/env python3
"""Registers the map built by FAST-LIO into Autoware's map frame, and generates
map_projector_info.yaml.

Does four things:
  1. Uses the map's own ground plane to do a one-shot SE(3) leveling (rotates the normal to +z).
     **Only valid when tilt is roughly constant along the route** -- check with check_map.py
     first; if it's cumulative drift, a rotation can't fix it and a different mapping approach
     is needed instead of patching it here.
     The rotation is also applied to the odometry trajectory, otherwise the SE(2) registration
     below would be inconsistent with the map.
  2. Fits SE(2) (planar rotation + translation, no scaling) between FAST-LIO odometry and GNSS
     over the same trajectory segment, with RANSAC for outlier rejection
  3. Translates the ground plane to z=0 (Autoware's base_link sits on the ground)
  4. Writes out map_origin's lat/lon/altitude: altitude is the GNSS median minus the antenna
     height, so that the base_link z GNSS reports comes out ~0

Usage: georef_map.py <odo.csv> <gps.csv> <input_pcd> <output_dir> [antenna_height_m]
"""
import sys, os, math, numpy as np

odo_csv, gps_csv, pcd_in, out_dir = sys.argv[1:5]
ANT_H = float(sys.argv[5]) if len(sys.argv) > 5 else 2.4   # imu_link (GNSS antenna) height above ground
# Diagnostic use: fit the leveling only from ground points near the trajectory during this
# initial time window (all map points are still kept).
# Used to check whether "the whole map can't be leveled with a single rigid rotation" is the
# mechanism behind NDT degrading near the start of the route.
LEVEL_WIN_S = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

odo = np.genfromtxt(odo_csv, delimiter=',', names=True)
gps = np.genfromtxt(gps_csv, delimiter=',', names=True)
print(f'odo {len(odo)} rows, gps {len(gps)} rows')

# ---- Read the map -----------------------------------------------------------
with open(pcd_in, 'rb') as fh:
    skip = 0
    for _ in range(30):
        line = fh.readline().decode('ascii', 'replace'); skip += 1
        if line.startswith('DATA'):
            break
P = np.loadtxt(pcd_in, skiprows=skip)
print(f'map has {len(P)} points')


def ground_points(Q):
    """Takes the lowest point in each 2 m xy grid cell, drops the top 10%."""
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


# ---- 1. SE(3) leveling: rotate the ground normal to +z ---------------------
G_all = ground_points(P)
if LEVEL_WIN_S > 0:
    from scipy.spatial import cKDTree
    T = np.c_[odo['x'], odo['y']][odo['t'] <= odo['t'][0] + LEVEL_WIN_S]
    sel = cKDTree(T[::20]).query(G_all[:, :2], k=1)[0] < 80.0
    print(f'local leveling: using only ground points within 80 m of the first {LEVEL_WIN_S:.0f} s '
          f'of trajectory: {sel.sum()}/{len(G_all)}')
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
otraj = np.c_[odo['x'], odo['y'], odo['z']] @ Rlev.T      # rotate the trajectory the same way
n1, _ = fit_normal(ground_points(P))
print(f'SE(3) leveling: tilt {tilt0:.2f}° -> {math.degrees(math.acos(abs(n1[2]))):.2f}°')

# ---- GNSS -> local ENU (origin at the first fix) ----------------------------
lat0, lon0 = float(np.median(gps['lat'][:20])), float(np.median(gps['lon'][:20]))
a, f = 6378137.0, 1/298.257223563
e2 = f * (2 - f)
s = math.sin(math.radians(lat0))
Rn = a / math.sqrt(1 - e2 * s * s)                 # prime-vertical radius of curvature
Rm = a * (1 - e2) / (1 - e2 * s * s) ** 1.5        # meridian radius of curvature
east = np.radians(gps['lon'] - lon0) * Rn * math.cos(math.radians(lat0))
north = np.radians(gps['lat'] - lat0) * Rm

# ---- Time alignment: interpolate odometry to GNSS timestamps ---------------
t_lo, t_hi = odo['t'].min(), odo['t'].max()
if LEVEL_WIN_S > 0:
    t_hi = min(t_hi, t_lo + LEVEL_WIN_S)      # diagnostic: SE(2) also restricted to the same window
    print(f'local registration: SE(2) using only the first {LEVEL_WIN_S:.0f} s of trajectory')
m = (gps['t'] >= t_lo) & (gps['t'] <= t_hi)
if m.sum() < 20:
    sys.exit(f'too few matchable samples ({m.sum()}), cannot register')
ox = np.interp(gps['t'][m], odo['t'], otraj[:, 0])
oy = np.interp(gps['t'][m], odo['t'], otraj[:, 1])
ge, gn = east[m], north[m]
print(f'{m.sum()} matchable points, odometry time span {t_hi-t_lo:.1f} s')


def kabsch_se2(src, dst):
    """Solves for R, t such that R@src + t ≈ dst (no scaling)."""
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, d]) @ U.T
    return R, cd - R @ cs


src = np.c_[ox, oy]; dst = np.c_[ge, gn]
best, rng = None, np.random.default_rng(0)
THR = 3.0                      # inlier threshold, m (GNSS σ≈2.9 m)
for _ in range(400):
    idx = rng.choice(len(src), 12, replace=False)
    R, t = kabsch_se2(src[idx], dst[idx])
    r = np.linalg.norm(src @ R.T + t - dst, axis=1)
    nin = (r < THR).sum()
    if best is None or nin > best[0]:
        best = (nin, r < THR)
inl = best[1]
R, t = kabsch_se2(src[inl], dst[inl])              # re-estimate using all inliers
res = np.linalg.norm(src @ R.T + t - dst, axis=1)
yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
print(f'SE(2) registration: inliers {inl.sum()}/{len(src)}  yaw={yaw:+.3f}°  '
      f'residual median={np.median(res[inl]):.2f} m mean={res[inl].mean():.2f} m max={res[inl].max():.2f} m')

# ---- Apply SE(2), then translate the ground to z=0 --------------------------
xy = P[:, :2] @ R.T + t
z0 = float(np.median(ground_points(P)[:, 2]))
print(f'median ground height z0={z0:.2f} m -> translating to 0')

os.makedirs(out_dir, exist_ok=True)
out_pcd = os.path.join(out_dir, 'pointcloud_map.pcd')
n = len(P)
with open(out_pcd, 'w') as fh:
    fh.write('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n')
    fh.write('FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
    fh.write(f'WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n')
    np.savetxt(fh, np.c_[xy, P[:, 2] - z0, P[:, 3]], fmt='%.3f %.3f %.3f %.1f')
print(f'wrote {out_pcd}: {n} points')

alt = float(np.median(gps['alt'])) - ANT_H          # ellipsoidal height of the ground
with open(os.path.join(out_dir, 'map_projector_info.yaml'), 'w') as fh:
    fh.write('projector_type: LocalCartesianUTM\nvertical_datum: WGS84\nmap_origin:\n')
    fh.write(f'  latitude: {lat0:.10f}\n  longitude: {lon0:.10f}\n  altitude: {alt:.2f}\n')
print(f'map_origin: lat={lat0:.7f} lon={lon0:.7f} alt={alt:.2f} '
      f'(GNSS median {np.median(gps["alt"]):.2f} - antenna height {ANT_H})')

osm = os.path.join(out_dir, 'lanelet2_map.osm')
if not os.path.exists(osm):
    with open(osm, 'w') as fh:
        fh.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        fh.write('<!-- Placeholder vector map: not needed for NDT localization, but map_loader '
                  'will load it. Planning needs a hand-drawn lane network. -->\n')
        fh.write('<osm version="0.6" generator="placeholder">\n')
        fh.write('  <MetaInfo format_version="1.0" map_version="1"/>\n')
        fh.write(f'  <node id="1" lat="{lat0:.10f}" lon="{lon0:.10f}">\n    <tag k="ele" v="0.0"/>\n  </node>\n')
        fh.write('</osm>\n')
    print(f'wrote placeholder {osm}')
