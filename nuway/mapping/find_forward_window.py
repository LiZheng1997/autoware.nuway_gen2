#!/usr/bin/env python3
"""Find stretches where the vehicle drove FORWARD through a mapped corridor.

The campus bag is driven out and back over the same road without turning around, so
the heading never changes: the first ~460 s are in reverse (longitudinal velocity
median -1.28 m/s) and ~480-900 s are forward (+1.28 m/s). The 250 s used for mapping
falls in the reverse half, which is why a lane laid along the direction of travel
points 180 deg away from the vehicle and route planning always fails.

This projects the whole bag's GNSS into the same local ENU frame the mapping run used,
and reports the time windows that are both inside the mapped corridor and moving
forward. On the campus bag that is 682.2-898.0 s (median 1.6 m from the corridor).

Usage: find_forward_window.py <bag_dir> <mapping_run_gps.csv> [corridor_radius_m=12]
"""
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial import cKDTree

BAG = sys.argv[1]
MAP_GPS = sys.argv[2]
RADIUS = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0

g = np.genfromtxt(MAP_GPS, delimiter=",", names=True)
lat0, lon0 = float(np.median(g["lat"][:20])), float(np.median(g["lon"][:20]))
a, f = 6378137.0, 1 / 298.257223563
e2 = f * (2 - f)
s = np.sin(np.radians(lat0))
Rn = a / np.sqrt(1 - e2 * s * s)
Rm = a * (1 - e2) / (1 - e2 * s * s) ** 1.5


def enu(lat, lon):
    return (np.radians(lon - lon0) * Rn * np.cos(np.radians(lat0)),
            np.radians(lat - lat0) * Rm)


ME, MN = enu(g["lat"], g["lon"])
corridor = cKDTree(np.c_[ME, MN])
print(f"corridor: {len(ME)} GNSS points, "
      f"E[{ME.min():.0f},{ME.max():.0f}] N[{MN.min():.0f},{MN.max():.0f}] m")

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
reader.set_filter(rosbag2_py.StorageFilter(topics=["/gps/fix", "/can_twist_fb"]))
Mg, Mv = get_message(types["/gps/fix"]), get_message(types["/can_twist_fb"])
G, V = [], []
t0 = None
while reader.has_next():
    topic, data, ts = reader.read_next()
    if t0 is None:
        t0 = ts
    rel = (ts - t0) / 1e9
    if topic == "/gps/fix":
        m = deserialize_message(data, Mg)
        G.append((rel, m.latitude, m.longitude))
    else:
        V.append((rel, deserialize_message(data, Mv).twist.linear.x))
G, V = np.array(G), np.array(V)

E, N = enu(G[:, 1], G[:, 2])
dist, _ = corridor.query(np.c_[E, N], k=1)
vel = np.interp(G[:, 0], V[:, 0], V[:, 1])
good = (dist < RADIUS) & (vel > 0.3)
print(f"\nforward and inside the corridor: {good.sum()}/{len(G)} GNSS fixes")
if good.sum() == 0:
    raise SystemExit("no forward pass over this corridor")

t = G[good, 0]
breaks = np.r_[0, np.where(np.diff(t) > 5.0)[0] + 1, len(t)]
print("\ncontiguous windows:")
for i in range(len(breaks) - 1):
    seg = t[breaks[i]:breaks[i + 1]]
    if len(seg) < 10:
        continue
    k = good.nonzero()[0][breaks[i]:breaks[i + 1]]
    print(f"  {seg[0]:6.1f} - {seg[-1]:6.1f} s  (lasts {seg[-1]-seg[0]:5.1f} s)  "
          f"distance to corridor median {np.median(dist[k]):.1f} m  "
          f"speed median {np.median(vel[k]):+.2f} m/s")
