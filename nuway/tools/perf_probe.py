#!/usr/bin/env python3
"""Collect the per-node timings Autoware publishes about itself.

Every heavy node emits */debug/processing_time_ms (one callback), cyclic_time_ms
(interval between callbacks) and pipeline_latency_ms (end to end). These are far more
useful than load average: processing time is essentially independent of replay rate,
so it says what real-time operation would cost.

Measured at 1.0x on 2026-10-04 (all keeping full frame rate, no drops):
  lidar_centerpoint 21.9 ms median / 32.8 p95 @ 9.5 Hz
  scan_ground_filter 17.7 / 31.8 @ 9.5 Hz
  euclidean_cluster 4.7 / 11.2, occupancy_grid_map 3.1 / 9.8, EKF 0.8 / 18.0 @ 38.9 Hz

Usage: perf_probe.py [seconds=120]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
KINDS = ("processing_time_ms", "cyclic_time_ms", "pipeline_latency_ms")


def pct(v, q):
    s = sorted(v)
    return s[min(len(s) - 1, int(q * len(s)))]


def main():
    rclpy.init()
    node = Node("perf_probe")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    start = time.time()
    while time.time() - start < 8:
        rclpy.spin_once(node, timeout_sec=0.2)

    topics = sorted(
        (name, types[0])
        for name, types in node.get_topic_names_and_types()
        if types and any(name.endswith(k) for k in KINDS))
    print(f"{len(topics)} timing topics discovered, sampling {DUR:.0f} s\n")

    seen = {}
    qos = QoSProfile(depth=50)
    qos.reliability = ReliabilityPolicy.RELIABLE
    for name, t in topics:
        try:
            cls = get_message(t)
        except Exception:
            continue
        seen[name] = []
        node.create_subscription(cls, name,
                                 lambda m, k=name: seen[k].append(float(m.data)), qos)

    start = time.time()
    while time.time() - start < DUR:
        rclpy.spin_once(node, timeout_sec=0.1)

    rows = []
    for name, v in seen.items():
        if len(v) < 3:
            continue
        kind = next(k for k in KINDS if name.endswith(k))
        rows.append((kind, name[: -len(kind) - 1].replace("/debug", ""),
                     len(v), pct(v, .5), pct(v, .95), max(v)))

    for kind in KINDS:
        sel = sorted((r for r in rows if r[0] == kind), key=lambda r: -r[4])
        if not sel:
            continue
        print(f"\n=== {kind} (ms) ===")
        print(f"{'node':<66}{'n':>5}{'median':>8}{'p95':>8}{'max':>9}")
        print("-" * 96)
        for _, node_name, cnt, p50, p95, mx in sel[:22]:
            print(f"{node_name:<66}{cnt:>5}{p50:>8.1f}{p95:>8.1f}{mx:>9.1f}")

    rclpy.shutdown()


main()
