#!/usr/bin/env python3
"""Read the ADAPI operation-mode and routing state with the right QoS.

Both topics are transient_local latched; `ros2 topic echo` with default (volatile) QoS
reports "does not appear to be published yet" however healthy the system is.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from autoware_adapi_v1_msgs.msg import OperationModeState, RouteState

MODE = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}
ROUTE = {0: "UNKNOWN", 1: "INITIALIZING", 2: "UNSET", 3: "ROUTING", 4: "SET",
         5: "ARRIVED", 6: "CHANGING"}

rclpy.init()
n = Node("opmode_state")
got = {}
q = QoSProfile(depth=1)
q.reliability = ReliabilityPolicy.RELIABLE
q.durability = DurabilityPolicy.TRANSIENT_LOCAL
n.create_subscription(OperationModeState, "/api/operation_mode/state",
                      lambda m: got.__setitem__("mode", m), q)
n.create_subscription(RouteState, "/api/routing/state",
                      lambda m: got.__setitem__("route", m), q)
t0 = time.time()
while time.time() - t0 < 10 and len(got) < 2:
    rclpy.spin_once(n, timeout_sec=0.3)

r = got.get("route")
print(f"route state        : {ROUTE.get(r.state, r.state) if r else 'not received'}")
m = got.get("mode")
if not m:
    print("operation mode     : not received")
else:
    print(f"operation mode     : {MODE.get(m.mode, m.mode)}")
    print(f"autoware control   : {m.is_autoware_control_enabled}")
    print(f"in transition      : {m.is_in_transition}")
    print(f"autonomous available: {m.is_autonomous_mode_available}")
    print(f"stop available     : {m.is_stop_mode_available}")
rclpy.shutdown()
