#!/usr/bin/env python3
"""Measure every trajectory-class topic on the planning/control chain.

Topic names are discovered by message type rather than typed out, because three
separate measurements were wasted on name and QoS mistakes:
  - /planning/scenario_planning/trajectory does not exist in 1.9.0; the final output
    is /planning/trajectory
  - the route topics are transient_local latched, so `ros2 topic hz` reports nothing
    for them no matter how healthy the chain is
Every topic is therefore subscribed twice, once volatile and once transient_local.

Usage: chain_probe.py [seconds=25]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import LaneletRoute, Path, Trajectory

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0

WANTED = {
    "autoware_planning_msgs/msg/LaneletRoute": LaneletRoute,
    "autoware_planning_msgs/msg/Path": Path,
    "autoware_planning_msgs/msg/Trajectory": Trajectory,
    "autoware_control_msgs/msg/Control": Control,
}
SKIP = ("debug", "candidate", "virtual_wall", "marker", "predicted_trajectory")


def main():
    rclpy.init()
    node = Node("chain_probe")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])

    start = time.time()
    while time.time() - start < 6:          # let discovery settle
        rclpy.spin_once(node, timeout_sec=0.2)

    found = []
    for name, types in node.get_topic_names_and_types():
        if not (name.startswith("/planning") or name.startswith("/control")):
            continue
        if any(s in name for s in SKIP):
            continue
        for t in types:
            if t in WANTED:
                found.append((name, t))
                break
    found.sort()
    print(f"{len(found)} trajectory-class topics discovered, sampling {DUR:.0f} s\n")

    seen = {}
    for name, t in found:
        seen[name] = []
        for durability in (DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL):
            qos = QoSProfile(depth=10)
            qos.reliability = ReliabilityPolicy.RELIABLE
            qos.durability = durability
            node.create_subscription(
                WANTED[t], name,
                lambda m, k=name: seen[k].append(
                    (time.time(),
                     len(getattr(m, "points", getattr(m, "segments", []))))),
                qos)

    start = time.time()
    while time.time() - start < DUR:
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f"{'topic':<62}{'msgs':>6}{'Hz':>8}{'points':>8}")
    print("-" * 86)
    for name, _ in found:
        v = seen[name]
        if not v:
            print(f"{name:<62}{'none':>6}")
            continue
        span = v[-1][0] - v[0][0]
        hz = len(v) / span if len(v) > 1 and span > 0 else 0.0
        pts = sorted(x[1] for x in v)[len(v) // 2]
        print(f"{name:<62}{len(v):>6}{hz:>8.2f}{pts:>8}")

    rclpy.shutdown()


main()
