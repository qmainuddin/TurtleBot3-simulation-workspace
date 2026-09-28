#!/usr/bin/env python3
"""Sensor check + LiDAR accuracy measurement against the known world geometry.

    ros2 launch tb3_nav_task trial.launch.py gui:=false navigator:=false monitor:=false &
    python3 probe_sensors.py [world] [n_scans]

For each received /scan the expected range of every beam is ray-cast in the world model
(tb3_nav_task/worlds.py) from the ground-truth LiDAR pose (base_footprint pose from Gazebo +
0.032 m rear offset). Reported: bias, mean absolute error, RMSE and max error over beams where
both are finite, and the number of hit/miss disagreements (one finite, the other inf/out of range).
The robot must be stationary (navigator:=false) so the scan and the pose refer to the same instant.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from tb3_nav_task.geometry import LIDAR_OFFSET_X  # noqa: E402
from tb3_nav_task.navigator import yaw_from_quat  # noqa: E402
from tb3_nav_task.worlds import WORLDS  # noqa: E402


class Probe(Node):
    def __init__(self):
        super().__init__('probe_sensors')
        self.scans, self.odom, self.gt = [], None, None
        self.create_subscription(LaserScan, '/scan', self.scans.append, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(PoseStamped, '/ground_truth/pose', lambda m: setattr(self, 'gt', m), 10)


def main():
    world = WORLDS[sys.argv[1] if len(sys.argv) > 1 else 'open_field']
    want = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    rclpy.init()
    n = Probe()
    end = n.get_clock().now().nanoseconds + 40e9
    while rclpy.ok() and (len(n.scans) < want or n.gt is None) and n.get_clock().now().nanoseconds < end:
        rclpy.spin_once(n, timeout_sec=0.5)
    if not n.scans:
        print('SCAN: none received')
        return
    s = n.scans[-1]
    r = list(s.ranges)
    fin = [x for x in r if math.isfinite(x)]
    print(f'SCAN frame={s.header.frame_id} n={len(r)} angle_min={s.angle_min:.3f} '
          f'angle_max={s.angle_max:.3f} inc={s.angle_increment:.5f} '
          f'range=[{s.range_min:.2f},{s.range_max:.2f}] finite={len(fin)} '
          f'min={min(fin) if fin else None} max={max(fin) if fin else None}')
    for deg in range(0, 360, 45):
        i = int(round((math.radians(deg) - s.angle_min) / s.angle_increment)) % len(r)
        print(f'   {deg:3d} deg: {r[i]:.3f}')
    o = n.odom
    print('ODOM:', (round(o.pose.pose.position.x, 3), round(o.pose.pose.position.y, 3)) if o else 'none')
    if n.gt is None:
        print('GT  : none (cannot compute accuracy)')
        return
    gx, gy = n.gt.pose.position.x, n.gt.pose.position.y
    gyaw = yaw_from_quat(n.gt.pose.orientation)
    print(f'GT  : ({gx:.3f}, {gy:.3f}, {math.degrees(gyaw):.1f} deg) world={world.name}')
    lx, ly = gx + LIDAR_OFFSET_X * math.cos(gyaw), gy + LIDAR_OFFSET_X * math.sin(gyaw)

    errs, disagree, total = [], 0, 0
    used = n.scans[-want:]
    for scan in used:
        for i, meas in enumerate(scan.ranges):
            a = gyaw + scan.angle_min + i * scan.angle_increment
            exp = world.raycast(lx, ly, a, scan.range_max)
            m_ok = math.isfinite(meas) and scan.range_min <= meas <= scan.range_max
            e_ok = math.isfinite(exp) and exp >= scan.range_min
            total += 1
            if m_ok and e_ok:
                errs.append(meas - exp)
            elif m_ok != e_ok:
                disagree += 1
    if errs:
        mae = sum(abs(e) for e in errs) / len(errs)
        rmse = math.sqrt(sum(e * e for e in errs) / len(errs))
        bias = sum(errs) / len(errs)
        big = sum(1 for e in errs if abs(e) > 0.05)
        print(f'LIDAR ACCURACY over {len(used)} scans, {total} beams: compared={len(errs)} '
              f'bias={bias * 1000:+.1f} mm  MAE={mae * 1000:.1f} mm  RMSE={rmse * 1000:.1f} mm  '
              f'max|err|={max(abs(e) for e in errs) * 1000:.1f} mm  |err|>5cm: {big}  '
              f'hit/miss disagreements={disagree}')
    else:
        print(f'LIDAR ACCURACY: no comparable beams (disagreements={disagree}/{total})')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
