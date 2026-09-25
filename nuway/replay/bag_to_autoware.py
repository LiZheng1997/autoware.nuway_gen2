#!/usr/bin/env python3
"""campus-bag -> Autoware 1.9.0 replay adapter.

Only used for offline replay; not needed on the real vehicle. It patches up three things:

1. Time base. Every topic in the bag (GPS/IMU/front+rear lidar) has a header stamp that runs
   about 18.2 s ahead of the bag's recording time -- the capture machine's clock and the
   sensors' clocks were never synchronized as a whole, though the sensors are consistent with
   each other. Meanwhile `ros2 bag play --clock` publishes /clock based on the recording time,
   so system-wide sim time lags the data by 18.2 s, and NDT's 1 s tolerance necessarily fails
   (observed: Validation error, with the gap a constant 18.16 s). This node self-calibrates that
   offset and re-stamps every forwarded message onto the /clock time base.
2. Point cloud type. The bag was recorded with the old velodyne driver, giving PointXYZIRT
   (point_step 22); the 1.9.0 preprocessing pipeline wants PointXYZIRC (16). On the real vehicle
   the nebula driver natively produces PointXYZIRCAEDT, so this isn't an issue there.
3. Vehicle speed. The bag carries speed on /can_twist_fb, but Autoware wants
   /vehicle/status/velocity_status (VelocityReport); without it gyro_odometer has no twist and
   the EKF just spins with no input.

Extrinsics always come from nuway_sensor_kit_description's real-vehicle calibration -- this
script only aligns frame names.
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

# autoware::point_types::PointXYZIRC (verified against types.hpp)
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

FRAME_ALIAS = {                       # old naming in the bag -> sensor kit naming
    'lidar_velodyne_front': 'velodyne_front_link',
    'lidar_velodyne_rear':  'velodyne_rear_link',
}
# Time offset calibration: can't just take the first N samples. A replay started with
# --start-offset delivers a burst of backlogged messages right at the start; at that point
# /clock has already jumped to its target value while the messages are still old, so
# (header - now) reads a transient value that's off by nearly 30 s
# (observed 2026-09-24: transient -11.58 s vs. the true +18.19 s, which made NDT report 2768
# Validation errors). Instead: wait at least WARMUP_S seconds, and only lock the offset once the
# spread across the last CALIB_SAMPLES samples is < CALIB_SPREAD_S.
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
        # Downstream gnss_poser / imu_corrector subscribe with rclcpp::QoS{1} (RELIABLE);
        # publishing BEST_EFFORT here makes the QoS incompatible and messages never get
        # delivered (observed: gnss_poser produces no output, pose_initializer reports 'The GNSS
        # pose has not arrived.'). The point cloud pipeline uses SensorDataQoS, so only these two
        # topics need RELIABLE.
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

    # ---- Time-base self-calibration --------------------------------------
    def calibrated(self, stamp):
        """Returns False (and the message is dropped) until calibration is complete. Requires a
        long enough warmup and a stable sample spread before locking the offset."""
        if self.offset_ns is not None:
            return True
        now = self.get_clock().now().nanoseconds
        if now < 1_000_000_000:           # /clock hasn't come up yet
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
                self.get_logger().info(f'time offset not stable yet (spread {spread:.3f} s), still observing...')
            return False
        self.offset_ns = int(statistics.median(self.samples))
        self.get_logger().info(
            f'Time offset calibration complete: header leads /clock by {self.offset_ns/1e9:.3f} s '
            f'(warmup {wall-self.first_msg_wall:.1f} s, {len(self.samples)} samples, spread {spread*1000:.0f} ms)')
        return False

    def shift(self, stamp):
        ns = to_ns(stamp) - self.offset_ns
        out = TimeMsg(); out.sec = ns // 1_000_000_000; out.nanosec = ns % 1_000_000_000
        return out

    # ---- Callbacks ---------------------------------------------------------
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
                    f'waiting for TF base_link <- {frame} (alias {FRAME_ALIAS.get(frame)})')
            return None
        R = quat_to_R(tr.transform.rotation)
        t = np.array([tr.transform.translation.x, tr.transform.translation.y,
                      tr.transform.translation.z])
        self.tf_cache[frame] = (R, t)
        self.get_logger().info(f'TF ready base_link <- {used}  (bag frame: {frame})')
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
            self.get_logger().info('waiting for /clock to calibrate the time offset...')
            return
        self.get_logger().info(
            f'cloud={self.n_cloud} IMU={self.n_imu} GNSS={self.n_fix} speed={self.n_vel}')


rclpy.init()
try:
    rclpy.spin(Adapter())
except KeyboardInterrupt:
    pass
