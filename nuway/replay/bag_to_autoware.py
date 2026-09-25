#!/usr/bin/env python3
"""campus 包 -> Autoware 1.9.0 回放适配器。

只服务于离线回放, 实车不需要。它补三件事:

1. 时间基准。包里所有话题(GPS/IMU/前后雷达)的 header 戳都比 bag 录制时刻超前约 18.2 s
   —— 采集机时钟与传感器时钟整体不同步, 各传感器之间是一致的。而 `ros2 bag play --clock`
   的 /clock 按录制时刻发布, 于是全系统 sim time 比数据落后 18.2 s, NDT 的 1 s 容差必然失败
   (实测 Validation error, 差值恒为 18.16 s)。这里自标定该偏移并把所有转发消息搬到 /clock 基准上。
2. 点云类型。包用旧 velodyne 驱动录制, 是 PointXYZIRT(point_step 22); 1.9.0 的预处理链要
   PointXYZIRC(16)。实车上 nebula 驱动原生产出 PointXYZIRCAEDT, 不存在这个问题。
3. 车速。包里车速在 /can_twist_fb, Autoware 要 /vehicle/status/velocity_status(VelocityReport);
   缺了它 gyro_odometer 没有 twist, EKF 只能空转。

外参一律用 nuway_sensor_kit_description 的实车标定, 本脚本只做 frame 名字对齐。
"""
import time
import rclpy, numpy as np, statistics
from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from autoware_vehicle_msgs.msg import VelocityReport
from builtin_interfaces.msg import Time as TimeMsg
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener

# autoware::point_types::PointXYZIRC (types.hpp 实查)
FIELDS = [
    PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity',   offset=12, datatype=PointField.UINT8,   count=1),
    PointField(name='return_type', offset=13, datatype=PointField.UINT8,   count=1),
    PointField(name='channel',     offset=14, datatype=PointField.UINT16,  count=1),
]
DT = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
               ('intensity', 'u1'), ('return_type', 'u1'), ('channel', '<u2')])
assert DT.itemsize == 16

FRAME_ALIAS = {                       # 包里的旧命名 -> sensor kit 命名
    'lidar_velodyne_front': 'velodyne_front_link',
    'lidar_velodyne_rear':  'velodyne_rear_link',
}
# 时间偏移标定: 不能直接取头 N 个样本。带 --start-offset 的回放开头会突发投递积压消息,
# 此时 /clock 已跳到位而消息还是旧的, (header − now) 会读出一个偏了近 30 s 的暂态值
# (2026-09-24 实测: 暂态 −11.58 s, 真值 +18.19 s, NDT 因此报 2768 次 Validation error)。
# 改为: 至少预热 WARMUP_S 秒, 且最近 CALIB_SAMPLES 个样本的极差 < CALIB_SPREAD_S 才锁定。
CALIB_SAMPLES = 40
WARMUP_S = 6.0
CALIB_SPREAD_S = 0.10


def quat_to_R(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
                     [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]], dtype=np.float64)


def to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class Adapter(Node):
    def __init__(self):
        super().__init__('bag_to_autoware')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.buf = Buffer(); self.lis = TransformListener(self.buf, self)
        self.tf_cache = {}; self.cache = {}
        self.n_cloud = self.n_imu = self.n_fix = self.n_vel = self.no_tf = 0
        self.offset_ns = None
        self.samples = deque(maxlen=CALIB_SAMPLES)
        self.first_msg_wall = None

        self.pub_cloud = self.create_publisher(
            PointCloud2, '/sensing/lidar/concatenated/pointcloud', qos_profile_sensor_data)
        # 下游 gnss_poser / imu_corrector 用 rclcpp::QoS{1}(RELIABLE) 订阅;
        # 这里若发 BEST_EFFORT 则 QoS 不兼容, 消息不会投递(实测: gnss_poser 无输出,
        # pose_initializer 报 'The GNSS pose has not arrived.')。点云链用的是
        # SensorDataQoS, 所以只有这两路要 RELIABLE。
        self.pub_imu = self.create_publisher(Imu, '/sensing/gnss/imu/data', 10)
        self.pub_fix = self.create_publisher(NavSatFix, '/sensing/gnss/imu/nav_sat_fix', 10)
        self.pub_vel = self.create_publisher(
            VelocityReport, '/vehicle/status/velocity_status', 10)

        for t in ('/lidar/velodyne/front/cloud', '/lidar/velodyne/rear/cloud'):
            self.create_subscription(PointCloud2, t,
                                     lambda m, tt=t: self.on_cloud(m, tt), qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 50)
        self.create_subscription(NavSatFix, '/gps/fix', self.on_fix, 10)
        self.create_subscription(TwistStamped, '/can_twist_fb', self.on_twist, 10)
        self.create_timer(10.0, self.report)

    # ---- 时间基准自标定 -------------------------------------------------
    def calibrated(self, stamp):
        """未标定完成返回 False(该消息丢弃)。要求预热够久且样本已稳定才锁定。"""
        if self.offset_ns is not None:
            return True
        now = self.get_clock().now().nanoseconds
        if now < 1_000_000_000:           # /clock 还没起来
            return False
        wall = time.monotonic()
        if self.first_msg_wall is None:
            self.first_msg_wall = wall
        self.samples.append(to_ns(stamp) - now)
        if wall - self.first_msg_wall < WARMUP_S or len(self.samples) < CALIB_SAMPLES:
            return False
        spread = (max(self.samples) - min(self.samples)) / 1e9
        if spread > CALIB_SPREAD_S:
            if int(wall) % 5 == 0:
                self.get_logger().info(f'时间偏移未稳定(极差 {spread:.3f} s), 继续观察…')
            return False
        self.offset_ns = int(statistics.median(self.samples))
        self.get_logger().info(
            f'时间偏移标定完成: header 比 /clock 超前 {self.offset_ns/1e9:.3f} s '
            f'(预热 {wall-self.first_msg_wall:.1f} s, {len(self.samples)} 样本, 极差 {spread*1000:.0f} ms)')
        return False

    def shift(self, stamp):
        ns = to_ns(stamp) - self.offset_ns
        out = TimeMsg(); out.sec = ns // 1_000_000_000; out.nanosec = ns % 1_000_000_000
        return out

    # ---- 回调 -----------------------------------------------------------
    def xform(self, frame):
        if frame in self.tf_cache:
            return self.tf_cache[frame]
        tr = used = None
        for cand in (frame, FRAME_ALIAS.get(frame)):
            if cand is None:
                continue
            try:
                tr = self.buf.lookup_transform('base_link', cand, rclpy.time.Time())
                used = cand
                break
            except Exception:
                continue
        if tr is None:
            self.no_tf += 1
            if self.no_tf % 50 == 1:
                self.get_logger().warn(
                    f'等待 TF base_link <- {frame} (别名 {FRAME_ALIAS.get(frame)})')
            return None
        R = quat_to_R(tr.transform.rotation)
        t = np.array([tr.transform.translation.x, tr.transform.translation.y,
                      tr.transform.translation.z])
        self.tf_cache[frame] = (R, t)
        self.get_logger().info(f'TF 就绪 base_link <- {used}  (bag frame: {frame})')
        return (R, t)

    def on_cloud(self, m, topic):
        if not self.calibrated(m.header.stamp):
            return
        X = self.xform(m.header.frame_id)
        if X is None:
            return
        R, t = X
        names = {f.name for f in m.fields}
        want = ('x', 'y', 'z') + (('intensity',) if 'intensity' in names else ()) \
                               + (('ring',) if 'ring' in names else ())
        a = pc2.read_points(m, field_names=want, skip_nans=True)
        P = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
        Pb = P @ R.T + t
        out = np.empty(len(Pb), dtype=DT)
        out['x'], out['y'], out['z'] = Pb[:, 0], Pb[:, 1], Pb[:, 2]
        out['intensity'] = np.clip(a['intensity'], 0, 255).astype(np.uint8) \
            if 'intensity' in want else 0
        out['return_type'] = 1
        out['channel'] = a['ring'].astype(np.uint16) if 'ring' in want else 0
        self.cache[topic] = out
        if not topic.endswith('front/cloud'):
            return
        parts = list(self.cache.values())
        merged = np.concatenate(parts) if len(parts) > 1 else parts[0]
        msg = PointCloud2()
        msg.header.stamp = self.shift(m.header.stamp)
        msg.header.frame_id = 'base_link'
        msg.height, msg.width = 1, len(merged)
        msg.fields = FIELDS
        msg.is_bigendian = False
        msg.point_step, msg.row_step = 16, 16 * len(merged)
        msg.is_dense = True
        msg.data = merged.tobytes()
        self.pub_cloud.publish(msg)
        self.n_cloud += 1

    def on_imu(self, m):
        if not self.calibrated(m.header.stamp):
            return
        m.header.stamp = self.shift(m.header.stamp)
        self.pub_imu.publish(m)
        self.n_imu += 1

    def on_fix(self, m):
        if not self.calibrated(m.header.stamp):
            return
        m.header.stamp = self.shift(m.header.stamp)
        self.pub_fix.publish(m)
        self.n_fix += 1

    def on_twist(self, m):
        if not self.calibrated(m.header.stamp):
            return
        v = VelocityReport()
        v.header.stamp = self.shift(m.header.stamp)
        v.header.frame_id = 'base_link'
        v.longitudinal_velocity = float(m.twist.linear.x)
        v.lateral_velocity = float(m.twist.linear.y)
        v.heading_rate = float(m.twist.angular.z)
        self.pub_vel.publish(v)
        self.n_vel += 1

    def report(self):
        if self.offset_ns is None:
            self.get_logger().info('等待 /clock 以标定时间偏移…')
            return
        self.get_logger().info(
            f'点云={self.n_cloud} IMU={self.n_imu} GNSS={self.n_fix} 车速={self.n_vel}')


rclpy.init()
try:
    rclpy.spin(Adapter())
except KeyboardInterrupt:
    pass
