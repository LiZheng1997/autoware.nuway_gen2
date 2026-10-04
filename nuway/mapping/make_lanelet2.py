#!/usr/bin/env python3
"""Generate an Autoware-usable lanelet2 map from the trajectory we actually drove.

Why this is legitimate: the route is one we drove ourselves, and after georef_map.py
puts the FAST-LIO odometry through the same transform chain as the point cloud, the
trajectory sits on the map's ground to within a median of -0.02 m (sd 0.02 m). That
trajectory is the lane centreline.

Coordinates: map_projector_info.yaml declares LocalCartesianUTM, which Autoware loads
with lanelet::projection::UtmProjector(origin) - UTM minus the origin's UTM offset. So
the inverse here must also be UTM (pyproj), matching what the loader does on the way
back in; round-trip residual measured at 0 mm. Going through the ENU approximation that
georef_map.py uses for GNSS would introduce the ~0.63 deg grid convergence at this
longitude. Node `ele` carries the map-frame z directly, because UtmProjector.forward
passes gps.ele through untouched rather than subtracting the origin altitude.

Direction matters: route_handler rejects a start lanelet whose heading differs from the
ego pose by more than yaw_threshold = pi/2. The first 460 s of the campus bag were
driven in REVERSE (longitudinal velocity median -1.28 m/s), so a lane laid along the
direction of travel points 180 deg away from where the vehicle faces and no route can
ever be planned. Pass reverse=1 for recordings like that - the lane must follow the
vehicle's heading. Use find_forward_window.py to locate a forward-driving stretch.

Usage: make_lanelet2.py <traj_map.csv> <map_projector_info.yaml> <out.osm>
       [lane_width_m=3.5] [point_spacing_m=2.0] [lanelet_length_m=25] [reverse=0]
"""
import math
import sys

import numpy as np
from pyproj import CRS, Transformer

traj_csv, proj_yaml, out_osm = sys.argv[1:4]
LANE_W = float(sys.argv[4]) if len(sys.argv) > 4 else 3.5
STEP = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0
SEG_LEN = float(sys.argv[6]) if len(sys.argv) > 6 else 25.0
REVERSE = bool(int(sys.argv[7])) if len(sys.argv) > 7 else False
SMOOTH_M = 4.0        # smoothing window in metres: kills odometry jitter, keeps curves

org = {}
for line in open(proj_yaml):
    for k in ("latitude", "longitude", "altitude", "projector_type"):
        if line.strip().startswith(k + ":"):
            v = line.split(":", 1)[1].strip()
            org[k] = v if k == "projector_type" else float(v)
assert org.get("projector_type") == "LocalCartesianUTM", \
    f"only LocalCartesianUTM is supported, got {org.get('projector_type')}"
lat0, lon0 = org["latitude"], org["longitude"]

zone = int((lon0 + 180) / 6) + 1
epsg = 32700 + zone if lat0 < 0 else 32600 + zone          # WGS84 / UTM, S or N
fwd = Transformer.from_crs(CRS("EPSG:4326"), CRS(f"EPSG:{epsg}"), always_xy=True)
inv = Transformer.from_crs(CRS(f"EPSG:{epsg}"), CRS("EPSG:4326"), always_xy=True)
E0, N0 = fwd.transform(lon0, lat0)
print(f"origin lat={lat0:.7f} lon={lon0:.7f} -> UTM {zone}{'S' if lat0 < 0 else 'N'} "
      f"(EPSG:{epsg})  E0={E0:.3f} N0={N0:.3f}")


def to_lonlat(x, y):
    return inv.transform(E0 + x, N0 + y)


tr = np.genfromtxt(traj_csv, delimiter=",", names=True)
P = np.c_[tr["x"], tr["y"], tr["z"]]
if REVERSE:
    P = P[::-1]
    print("centreline reversed to follow the vehicle heading")
d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1))]
keep = np.r_[True, np.diff(d) > 1e-6]                      # drop repeats while standing
P, d = P[keep], d[keep]
print(f"{len(P)} poses, {d[-1]:.1f} m")

s_new = np.arange(0.0, d[-1], STEP)
C = np.c_[np.interp(s_new, d, P[:, 0]),
          np.interp(s_new, d, P[:, 1]),
          np.interp(s_new, d, P[:, 2])]

w = max(3, int(round(SMOOTH_M / STEP)) | 1)
pad = w // 2
padded = np.vstack([np.repeat(C[:1], pad, 0), C, np.repeat(C[-1:], pad, 0)])
ker = np.ones(w) / w
C = np.c_[np.convolve(padded[:, 0], ker, "valid"),
          np.convolve(padded[:, 1], ker, "valid"),
          np.convolve(padded[:, 2], ker, "valid")]
print(f"centreline {len(C)} points, spacing {STEP} m, smoothing {w} pts ({w*STEP:.0f} m)")

T = np.gradient(C[:, :2], axis=0)
T /= np.linalg.norm(T, axis=1, keepdims=True)
left_n = np.c_[-T[:, 1], T[:, 0]]                          # +90 deg = left
L = np.c_[C[:, :2] + left_n * (LANE_W / 2), C[:, 2]]
R = np.c_[C[:, :2] - left_n * (LANE_W / 2), C[:, 2]]

per = max(2, int(round(SEG_LEN / STEP)))
segs = [(a, min(a + per, len(C) - 1)) for a in range(0, len(C) - 1, per)]
segs = [s for s in segs if s[1] - s[0] >= 1]
print(f"{len(segs)} lanelets of about {SEG_LEN} m")

nid, wid, rid = 1, 10000, 20000
node_lines, id_left, id_right = [], {}, {}


def add_node(i, side):
    global nid
    table = id_left if side == "L" else id_right
    if i in table:
        return table[i]        # consecutive lanelets must share the endpoint node ids,
    p = (L if side == "L" else R)[i]          # otherwise routing will not connect them
    lon, lat = to_lonlat(p[0], p[1])
    node_lines.append(
        f'  <node id="{nid}" lat="{lat:.10f}" lon="{lon:.10f}">\n'
        f'    <tag k="ele" v="{p[2]:.3f}"/>\n'
        f'  </node>')
    table[i] = nid
    nid += 1
    return table[i]


way_lines, rel_lines = [], []
for a, b in segs:
    idx = list(range(a, b + 1))
    for ids in ([add_node(i, "L") for i in idx], [add_node(i, "R") for i in idx]):
        nd = "\n".join(f'    <nd ref="{k}"/>' for k in ids)
        way_lines.append(
            f'  <way id="{wid}">\n{nd}\n'
            f'    <tag k="type" v="line_thin"/>\n'
            f'    <tag k="subtype" v="solid"/>\n'
            f'  </way>')
        wid += 1
    rel_lines.append(
        f'  <relation id="{rid}">\n'
        f'    <member type="way" role="left" ref="{wid-2}"/>\n'
        f'    <member type="way" role="right" ref="{wid-1}"/>\n'
        f'    <tag k="type" v="lanelet"/>\n'
        f'    <tag k="subtype" v="road"/>\n'
        f'    <tag k="location" v="urban"/>\n'
        f'    <tag k="one_way" v="yes"/>\n'
        f'  </relation>')
    rid += 1

with open(out_osm, "w") as fh:
    fh.write('<?xml version="1.0" encoding="UTF-8"?>\n')
    fh.write(f'<!-- generated from a recorded drive: {len(segs)} lanelets, '
             f'lane width {LANE_W} m, {d[-1]:.1f} m total -->\n')
    fh.write('<osm version="0.6" generator="make_lanelet2.py">\n')
    fh.write('  <MetaInfo format_version="1.0" map_version="1"/>\n')
    fh.write("\n".join(node_lines) + "\n")
    fh.write("\n".join(way_lines) + "\n")
    fh.write("\n".join(rel_lines) + "\n")
    fh.write("</osm>\n")

print(f"wrote {out_osm}: {nid-1} nodes, {wid-10000} ways, {rid-20000} lanelets")
print(f"start map=({C[0,0]:.1f}, {C[0,1]:.1f})  end map=({C[-1,0]:.1f}, {C[-1,1]:.1f})")

check = []
for i in (0, len(C) // 2, len(C) - 1):
    lon, lat = to_lonlat(L[i, 0], L[i, 1])
    e, n = fwd.transform(lon, lat)
    check.append(math.hypot(e - E0 - L[i, 0], n - N0 - L[i, 1]))
print(f"UTM round-trip residual: max {max(check)*1000:.3f} mm")
