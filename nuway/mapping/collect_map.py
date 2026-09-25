#!/usr/bin/env python3
"""订阅 FAST-LIO 的 cloud_registered，体素降采样后累积，退出时存成单个 PCD。
不依赖 spark_fast_lio 自身的保存逻辑（该版本只支持周期保存且无退出钩子）。"""
import rclpy, numpy as np, sys, time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

VOXEL = 0.2          # Autoware NDT 常用 0.2~0.5 m
OUT = sys.argv[1] if len(sys.argv) > 1 else "/home/lz/campus_map/campus_short.pcd"

class C(Node):
    def __init__(self):
        super().__init__('map_collector')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.vox = {}          # 体素键 -> 累加点与计数
        self.frames = 0
        self.create_subscription(PointCloud2, '/fast_lio/cloud_registered', self.cb, qos_profile_sensor_data)
        self.create_timer(30.0, lambda: self.get_logger().info(
            f'帧={self.frames} 体素={len(self.vox)}'))
    def cb(self, m):
        a = pc2.read_points(m, field_names=('x','y','z','intensity'), skip_nans=True)
        if len(a) == 0: return
        P = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
        I = a['intensity'].astype(np.float64)
        keys = np.floor(P / VOXEL).astype(np.int64)
        h = (keys[:,0] * 73856093) ^ (keys[:,1] * 19349663) ^ (keys[:,2] * 83492791)
        # 向量化：先在本帧内按体素聚合，再合并到全局字典。
        # 原来的逐点 Python 循环在 12000 点/帧 x 10 Hz 下跟不上。
        order = np.argsort(h, kind='stable')
        hs = h[order]; Ps = P[order]; Is = I[order]
        uniq, start = np.unique(hs, return_index=True)
        sums = np.add.reduceat(Ps, start, axis=0)
        isum = np.add.reduceat(Is, start)
        cnts = np.diff(np.append(start, len(hs)))
        for k, sx, sy, sz, si, c in zip(uniq.tolist(), sums[:,0], sums[:,1], sums[:,2], isum, cnts):
            v = self.vox.get(k)
            if v is None: self.vox[k] = [sx, sy, sz, si, int(c)]
            else:
                v[0]+=sx; v[1]+=sy; v[2]+=sz; v[3]+=si; v[4]+=int(c)
        self.frames += 1

def write_pcd(vox, path):
    # 向量化写出: 全程建图有 ~250 万体素, 逐点 f-string 要一分多钟
    n = len(vox)
    A = np.fromiter((x for v in vox.values() for x in v), dtype=np.float64, count=n*5).reshape(n, 5)
    A[:, :4] /= A[:, 4:5]
    with open(path, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        f.write("FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
        np.savetxt(f, A[:, :4], fmt='%.3f %.3f %.3f %.1f')
    return n

rclpy.init()
n = C()
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    cnt = write_pcd(n.vox, OUT)
    print(f"\n已写入 {OUT}: {cnt} 点 (体素 {VOXEL} m, 累积 {n.frames} 帧)", flush=True)
