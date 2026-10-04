#!/usr/bin/env python3
"""Attribute CPU use to Autoware subsystems.

"The top process is only 38%, there is no hotspot" does not answer "what does each
module cost". This reads /proc/<pid>/stat twice to get real CPU%, then maps every
process to a subsystem through the `__ns:=` / `__node:=` arguments on its command
line; standalone executables fall back to the package name in their install path.

Measured this way at 1.0x replay on 2026-10-04 (rmw_fastrtps_cpp baseline, 950% total):
  planning 175.0 (13 procs) | localization 169.2 (10) | pointcloud_container etc 135.0
  sensing 114.3 | system 114.0 | replay harness 94.5 | control 67.7 | perception 46.6
After switching to CycloneDDS on lo every subsystem dropped 46-76%.

Usage: cpu_attrib.py [seconds=60]
"""
import os
import re
import sys
import time

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
HZ = os.sysconf("SC_CLK_TCK")

NS_RE = re.compile(r"__ns:=(\S+)")
NODE_RE = re.compile(r"__node:=(\S+)")
PKG_RE = re.compile(r"/install/([^/]+)/lib/")

# package-name fragment -> subsystem, for nodes launched without a namespace
PKG_SUBSYS = [
    ("ndt_scan_matcher", "localization"), ("ekf_localizer", "localization"),
    ("gyro_odometer", "localization"), ("pose_initializer", "localization"),
    ("pose_instability", "localization"), ("localization_error", "localization"),
    ("twist2accel", "localization"), ("stop_filter", "localization"),
    ("gyro_bias", "localization"), ("map_projection", "map"), ("map_hash", "map"),
    ("gnss_poser", "sensing"), ("imu_corrector", "sensing"),
    ("vehicle_velocity_converter", "sensing"),
    ("multi_object_tracker", "perception"), ("map_based_prediction", "perception"),
    ("shape_estimation", "perception"), ("detected_object", "perception"),
    ("traffic_light", "perception"), ("crosswalk", "perception"),
    ("perception_analytics", "perception"),
    ("scenario_selector", "planning"), ("planning_evaluator", "planning"),
    ("external_velocity", "planning"), ("goal_pose_visualizer", "planning"),
    ("remaining_distance", "planning"), ("mission_planner", "planning"),
    ("control_evaluator", "control"), ("mrm_handler", "system"),
    ("hazard", "system"), ("duplicated_node", "system"),
    ("processing_time_checker", "system"), ("pipeline_latency", "system"),
    ("service_log_checker", "system"), ("default_adapi", "api"), ("adapi", "api"),
    ("automatic_pose_initializer", "api"), ("routing_adaptor", "api"),
    ("initial_pose_adaptor", "api"), ("manual_lane_change", "api"),
]


def classify(cmd):
    if "bag_to_autoware" in cmd:
        return "replay harness", "bag_to_autoware.py"
    if "rosbag2" in cmd or "bag play" in cmd:
        return "replay harness", "ros2 bag play"
    if re.search(r"bin/ros2 launch", cmd):
        return "launch itself", "ros2 launch"
    if "/rviz2" in cmd or cmd.strip().endswith("rviz2"):
        return "visualisation", "rviz2"
    if "robot_state_publisher" in cmd:
        return "system", "robot_state_publisher"
    if "/ros2 daemon" in cmd or "_ros2_daemon" in cmd:
        return "ros daemon", "ros2 daemon"

    ns = NS_RE.search(cmd)
    node = NODE_RE.search(cmd)
    name = node.group(1) if node else None
    if ns:
        top = ns.group(1).strip("/").split("/")[0]
        known = {"sensing", "localization", "perception", "planning", "control",
                 "system", "map", "vehicle", "api"}
        if top in known:
            return top, name or top
        if top == "simulation":
            return "system", name or top
    pkg = PKG_RE.search(cmd)
    if pkg:
        p = pkg.group(1)
        for key, sub in PKG_SUBSYS:
            if key in p:
                return sub, name or p
        return "other autoware", name or p
    if name:
        for key, sub in PKG_SUBSYS:
            if key in name:
                return sub, name
        return "other autoware", name
    return None, None


def snapshot():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode("utf-8", "replace")
            if not cmd.strip():
                continue
            with open(f"/proc/{pid}/stat") as f:
                parts = f.read().rsplit(")", 1)[1].split()
            jiff = int(parts[11]) + int(parts[12])          # utime + stime
            with open(f"/proc/{pid}/statm") as f:
                rss = int(f.read().split()[1]) * os.sysconf("SC_PAGE_SIZE")
        except Exception:
            continue
        out[pid] = (jiff, cmd, rss)
    return out


first = snapshot()
time.sleep(DUR)
second = snapshot()

rows = []
for pid, (jiff, cmd, rss) in second.items():
    if pid not in first:
        continue
    cpu = (jiff - first[pid][0]) / HZ / DUR * 100.0
    if cpu < 0.05:
        continue
    sub, name = classify(cmd)
    if sub is None:
        continue
    rows.append((sub, name or "?", cpu, rss / 2**20))

total = sum(r[2] for r in rows)
bysub = {}
for sub, _, cpu, rss in rows:
    c, m, n = bysub.get(sub, (0.0, 0.0, 0))
    bysub[sub] = (c + cpu, m + rss, n + 1)

print(f"sampled {DUR:.0f} s   {len(rows)} processes attributed   "
      f"total {total:.0f}% (= {total/100:.1f} cores)\n")
print(f"{'subsystem':<18}{'procs':>6}{'CPU%':>9}{'share':>8}{'RSS MB':>10}")
print("-" * 51)
for sub, (c, m, n) in sorted(bysub.items(), key=lambda x: -x[1][0]):
    print(f"{sub:<18}{n:>6}{c:>9.1f}{100*c/total:>7.1f}%{m:>10.0f}")

print(f"\n{'subsystem':<18}{'node / process':<46}{'CPU%':>8}{'RSS MB':>9}")
print("-" * 82)
for sub, name, cpu, rss in sorted(rows, key=lambda r: -r[2])[:30]:
    print(f"{sub:<18}{name[:45]:<46}{cpu:>8.1f}{rss:>9.0f}")
