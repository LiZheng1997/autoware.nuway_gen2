#!/usr/bin/env python3
"""直接用 rclpy 订阅采 NDT 指标 —— 不经 ros2 CLI/daemon。

2026-09-24 教训: ros2 topic echo 因 daemon 被误杀而全部启动即崩(xmlrpc Fault
'!rclpy.ok()'), 8 个采集文件全是 traceback, 于是脚本一路报 NVTL=0 —— 是测量假象,
不是系统故障。带数值的判据不要经过 CLI。
"""
import sys, math, csv, statistics as st
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from autoware_internal_debug_msgs.msg import Float32Stamped, Int32Stamped

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
CSV = sys.argv[2] if len(sys.argv) > 2 else None   # 逐帧时序, 用来看在哪里退化
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
        return '无样本'
    return (f'n={len(v)} 均值={st.mean(v):.2f} 中位={st.median(v):.2f} '
            f'最小={min(v):.2f} 最大={max(v):.2f}')


rclpy.init()
n = C()
end = n.get_clock().now().nanoseconds  # sim time 可能不推进, 用 wall 计时
import time
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.2)

print(f'采集 {DUR:.0f} s (wall)')
print(f'  NVTL (最近体素变换似然, >2.3 为良好): {stats(n.d["nvtl"])}')
print(f'  TP   (变换概率,       >3.0 为良好): {stats(n.d["tp"])}')
print(f'  迭代次数 (上限 30):                  {stats([float(x) for x in n.d["iter"]])}')
print(f'  单帧耗时 ms:                         {stats(n.d["exe"])}')
print(f'  NDT 位姿帧数={len(n.ndt)}  GNSS 位姿帧数={len(n.gnss)}  kinematic_state={len(n.kin)}')

# NDT 与 GNSS 的水平距离(同为 map 系), 取时间最近配对
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
        print(f'  NDT 与 GNSS 水平距离 m: n={len(ds)} 中位={st.median(ds):.2f} '
              f'均值={st.mean(ds):.2f} 最大={max(ds):.2f}')
if n.ndt:
    z = [m.pose.pose.position.z for m in n.ndt]
    print(f'  NDT 高程 z 范围 m: {min(z):.2f} ~ {max(z):.2f}')

if CSV and n.series:
    t0 = n.series[0][0]
    with open(CSV, 'w', newline='') as fh:
        w = csv.writer(fh); w.writerow(['t_rel', 'nvtl'])
        w.writerows([(round(t - t0, 3), round(v, 4)) for t, v in n.series])
    # 按时间十分位报, 直接看出在哪一段掉下去
    import numpy as np
    A = np.array(n.series); A[:, 0] -= t0
    print(f'  NVTL 随时间(十分位, 阈值 2.3):')
    edges = np.linspace(0, A[-1, 0], 11)
    for i in range(10):
        m = (A[:, 0] >= edges[i]) & (A[:, 0] < edges[i+1])
        if m.sum():
            v = A[m, 1]
            bad = 100 * (v < 2.3).mean()
            print(f'    {edges[i]:5.0f}-{edges[i+1]:5.0f}s  n={m.sum():4d}  '
                  f'中位={np.median(v):.2f}  最小={v.min():.2f}  低于阈值={bad:5.1f}%')
    print(f'  写出 {CSV}')
rclpy.shutdown()
