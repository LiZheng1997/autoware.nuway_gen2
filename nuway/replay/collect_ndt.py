#!/usr/bin/env python3
"""Subscribes directly via rclpy to collect NDT metrics -- bypasses the ros2 CLI/daemon.

Lesson from 2026-09-24: ros2 topic echo crashed on startup across the board because the daemon
had been killed by mistake (xmlrpc Fault '!rclpy.ok()'), so all 8 collection files were nothing
but tracebacks, and the script kept reporting NVTL=0 the whole time -- that was a measurement
artifact, not an actual system failure. A threshold check that reads a numeric value should
never go through the CLI.
"""
import sys, math, csv, statistics as st
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
CSV = sys.argv[2] if len(sys.argv) > 2 else None   # per-frame time series, to see where it degrades
P = '/localization/pose_estimator/'


class C(Node):
    def __init__(self):
        super().__init__('collect_ndt')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.d = {k: [] for k in
                  ('nvtl', 'tp', 'iter', 'exe')}
        self.ndt = []; self.gnss = []; self.kin = []
        self.series = []            # (sim_t, nvtl)
        f = lambda k: (lambda m: self.d[k].append(m.data))

        def on_nvtl(m):
            self.d['nvtl'].append(m.data)
            self.series.append((m.stamp.sec + m.stamp.nanosec * 1e-9, m.data))
        self.create_subscription(Float32Stamped, P+'nearest_voxel_transformation_likelihood', on_nvtl, 10)
        self.create_subscription(Float32Stamped, P+'transform_probability', f('tp'), 10)
        self.create_subscription(Int32Stamped,   P+'iteration_num', f('iter'), 10)
        self.create_subscription(Float32Stamped, P+'exe_time_ms', f('exe'), 10)
        self.create_subscription(PoseWithCovarianceStamped, P+'pose_with_covariance',
                                 lambda m: self.ndt.append(m), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance',
                                 lambda m: self.gnss.append(m), 10)
        self.create_subscription(Odometry, '/localization/kinematic_state',
                                 lambda m: self.kin.append(m), 10)


def stats(v):
    if not v:
        return 'no samples'
    return (f'n={len(v)} mean={st.mean(v):.2f} median={st.median(v):.2f} '
            f'min={min(v):.2f} max={max(v):.2f}')


rclpy.init()
n = C()
end = n.get_clock().now().nanoseconds  # sim time may not advance, so time it with the wall clock
import time
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.2)

print(f'collected {DUR:.0f} s (wall)')
print(f'  NVTL (nearest voxel transformation likelihood, >2.3 is good): {stats(n.d["nvtl"])}')
print(f'  TP   (transform probability,          >3.0 is good): {stats(n.d["tp"])}')
print(f'  iteration count (cap 30):                            {stats([float(x) for x in n.d["iter"]])}')
print(f'  per-frame time ms:                                   {stats(n.d["exe"])}')
print(f'  NDT pose frames={len(n.ndt)}  GNSS pose frames={len(n.gnss)}  kinematic_state={len(n.kin)}')

# Horizontal distance between NDT and GNSS (both in the map frame), matched by nearest time
if n.ndt and n.gnss:
    def ts(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    g = sorted(n.gnss, key=ts)
    ds = []
    for a in n.ndt:
        b = min(g, key=lambda x: abs(ts(x) - ts(a)))
        if abs(ts(b) - ts(a)) > 0.5:
            continue
        ds.append(math.hypot(a.pose.pose.position.x - b.pose.pose.position.x,
                             a.pose.pose.position.y - b.pose.pose.position.y))
    if ds:
        print(f'  NDT vs GNSS horizontal distance m: n={len(ds)} median={st.median(ds):.2f} '
              f'mean={st.mean(ds):.2f} max={max(ds):.2f}')
if n.ndt:
    z = [m.pose.pose.position.z for m in n.ndt]
    print(f'  NDT elevation z range m: {min(z):.2f} ~ {max(z):.2f}')

if CSV and n.series:
    t0 = n.series[0][0]
    with open(CSV, 'w', newline='') as fh:
        w = csv.writer(fh); w.writerow(['t_rel', 'nvtl'])
        w.writerows([(round(t - t0, 3), round(v, 4)) for t, v in n.series])
    # Reported by time decile, so you can immediately see which segment drops off
    import numpy as np
    A = np.array(n.series); A[:, 0] -= t0
    print(f'  NVTL over time (deciles, threshold 2.3):')
    edges = np.linspace(0, A[-1, 0], 11)
    for i in range(10):
        m = (A[:, 0] >= edges[i]) & (A[:, 0] < edges[i+1])
        if m.sum():
            v = A[m, 1]
            bad = 100 * (v < 2.3).mean()
            print(f'    {edges[i]:5.0f}-{edges[i+1]:5.0f}s  n={m.sum():4d}  '
                  f'median={np.median(v):.2f}  min={v.min():.2f}  below_thresh={bad:5.1f}%')
    print(f'  wrote {CSV}')
rclpy.shutdown()
