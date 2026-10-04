#!/usr/bin/env python3
"""Publish a goal pose along the recorded route - the scripted form of RViz's 2D Goal Pose.

routing_adaptor subscribes to ~/input/rough_goal, remapped to /rviz/routing/rough_goal
by rviz_adaptors.launch.xml, and its own 5 Hz timer then calls
/api/routing/set_route_points.

Two things learned the hard way:
  - the fraction is measured along the LANE, so `reverse` must match whatever was passed
    to make_lanelet2.py
  - behavior_planning_container was seen to segfault once with the goal only 14 m from
    the end of the lane, while goal_planner was active. Leaving plenty of lane beyond
    the goal has been reliable; the attribution was never pinned down cleanly, so treat
    it as a precaution rather than a proven rule.

Usage: set_goal.py <traj_map.csv> [fraction=0.95] [publishes=5] [reverse=0]
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

traj_csv = sys.argv[1]
FRAC = float(sys.argv[2]) if len(sys.argv) > 2 else 0.95
TIMES = int(sys.argv[3]) if len(sys.argv) > 3 else 5
REVERSE = bool(int(sys.argv[4])) if len(sys.argv) > 4 else False

tr = np.genfromtxt(traj_csv, delimiter=",", names=True)
P = np.c_[tr["x"], tr["y"], tr["z"]]
if REVERSE:
    P = P[::-1]
d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P[:, :2], axis=0), axis=1))]
i = min(int(np.searchsorted(d, FRAC * d[-1])), len(P) - 2)
j = min(int(np.searchsorted(d, d[i] + 5.0)), len(P) - 1)   # 5 m secant, not a tangent
yaw = math.atan2(P[j, 1] - P[i, 1], P[j, 0] - P[i, 0])

rclpy.init()
node = Node("set_goal")
node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
pub = node.create_publisher(PoseStamped, "/rviz/routing/rough_goal", 3)

msg = PoseStamped()
msg.header.frame_id = "map"
msg.pose.position.x = float(P[i, 0])
msg.pose.position.y = float(P[i, 1])
msg.pose.position.z = float(P[i, 2])
msg.pose.orientation.z = math.sin(yaw / 2)
msg.pose.orientation.w = math.cos(yaw / 2)
print(f"goal: map=({P[i,0]:.1f}, {P[i,1]:.1f}, {P[i,2]:.2f})  "
      f"yaw={math.degrees(yaw):+.1f} deg  {d[i]:.1f}/{d[-1]:.1f} m along the lane")

for _ in range(50):                      # a PoseStamped sent before anyone subscribes
    if pub.get_subscription_count() > 0:  # is simply dropped
        break
    rclpy.spin_once(node, timeout_sec=0.2)
print(f"{pub.get_subscription_count()} subscriber(s)")

for k in range(TIMES):
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    print(f"  published {k+1}")
    start = time.time()
    while time.time() - start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)

rclpy.shutdown()
