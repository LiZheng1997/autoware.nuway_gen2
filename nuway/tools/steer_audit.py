#!/usr/bin/env python3
"""Audit the steering Autoware actually commands against the EZ10's limits.

VEHICLE_LIMITS.md: exceeding +0.20 rad/s steering rate triggers a full vehicle
emergency stop. vehicle_cmd_gate currently allows steer_rate_lim_for_steer_cmd = 1.0
rad/s and steer_cmd_lim = 1.0 rad, while vehicle_info declares max_steer_angle 0.70.
Config permitting something is not the same as the planner asking for it, so measure.

Reports both the rate the message carries and the rate implied by successive angles.

Usage: steer_audit.py [seconds=60]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from autoware_control_msgs.msg import Control

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
RATE_LIMIT = 0.20          # rad/s, EZ10 emergency-stop threshold
ANGLE_LIMIT = 0.70         # rad, vehicle_info max_steer_angle

samples = []               # (stamp_s, angle, reported_rate)


def main():
    rclpy.init()
    n = Node("steer_audit")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    q = QoSProfile(depth=50)
    q.reliability = ReliabilityPolicy.RELIABLE

    def cb(m):
        t = m.stamp.sec + m.stamp.nanosec * 1e-9
        samples.append((t, m.lateral.steering_tire_angle,
                        m.lateral.steering_tire_rotation_rate))

    n.create_subscription(Control, "/control/command/control_cmd", cb, q)
    t0 = time.time()
    while time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
    rclpy.shutdown()

    if len(samples) < 5:
        print(f"only {len(samples)} commands seen - nothing to audit")
        return

    samples.sort()
    import statistics as st
    ang = [abs(s[1]) for s in samples]
    rep = [abs(s[2]) for s in samples]
    diff = []
    for a, b in zip(samples, samples[1:]):
        dt = b[0] - a[0]
        if 1e-4 < dt < 1.0:
            diff.append(abs(b[1] - a[1]) / dt)

    def line(label, v, limit):
        v = sorted(v)
        p50, p95, mx = v[len(v)//2], v[int(.95*len(v))], v[-1]
        over = sum(1 for x in v if x > limit) / len(v) * 100
        flag = "OVER LIMIT" if mx > limit else "ok"
        print(f"  {label:<34} median {p50:7.4f}  p95 {p95:7.4f}  max {mx:7.4f}"
              f"   {over:5.1f}% above {limit}  [{flag}]")

    print(f"{len(samples)} control commands over {DUR:.0f} s "
          f"({len(samples)/DUR:.1f} Hz)\n")
    line("steering angle (rad)", ang, ANGLE_LIMIT)
    line("rate reported in message (rad/s)", rep, RATE_LIMIT)
    if diff:
        line("rate implied by d(angle)/dt", diff, RATE_LIMIT)
    print(f"\n  angle mean {st.mean([s[1] for s in samples]):+.4f} rad, "
          f"reported-rate mean {st.mean([s[2] for s in samples]):+.4f} rad/s")


main()
