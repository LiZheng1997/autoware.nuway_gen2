#!/usr/bin/env python3
"""同时记录 FAST-LIO 里程计与 GNSS，用于拟合地图坐标系 → ENU 的变换"""
import rclpy, csv, sys
OUTDIR = sys.argv[1] if len(sys.argv) > 1 else '/home/lz/campus_map'
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
odo=[]; gps=[]
class R(Node):
    def __init__(self):
        super().__init__('traj_rec')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.create_subscription(Odometry,'/Odometry',
            lambda m: odo.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                                  m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z)),50)
        self.create_subscription(NavSatFix,'/gps/fix',
            lambda m: gps.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                                  m.latitude,m.longitude,m.altitude,m.status.status,
                                  m.position_covariance[0])),50)
rclpy.init(); n=R()
try: rclpy.spin(n)
except BaseException: pass
finally:
    with open(f"{OUTDIR}/odo.csv","w") as f:
        w=csv.writer(f); w.writerow(["t","x","y","z"]); w.writerows(odo)
    with open(f"{OUTDIR}/gps.csv","w") as f:
        w=csv.writer(f); w.writerow(["t","lat","lon","alt","status","cov"]); w.writerows(gps)
    print(f"\n已写入 odo={len(odo)} gps={len(gps)}", flush=True)
