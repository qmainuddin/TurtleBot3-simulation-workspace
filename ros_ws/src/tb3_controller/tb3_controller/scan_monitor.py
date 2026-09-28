"""scan_monitor — print FRONT/LEFT/RIGHT LiDAR distances twice a second.

Use it to check the sensor BEFORE letting the controller drive:
    ros2 run tb3_controller scan_monitor --ros-args -p use_sim_time:=true
Drive with teleop toward a wall; FRONT should shrink as you approach.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from tb3_controller.decision import compute_sectors


class ScanMonitor(Node):
    def __init__(self):
        super().__init__('scan_monitor')
        self.last = None
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.5, self.report)

    def on_scan(self, msg):
        self.last = msg

    def report(self):
        s = self.last
        if s is None:
            self.get_logger().info('waiting for /scan ...')
            return
        sec = compute_sectors(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
        f = lambda x: '  n/a' if x is None else f'{x:5.2f}'  # noqa: E731
        self.get_logger().info(
            f'beams={len(s.ranges)} range=[{s.range_min:.2f},{s.range_max:.2f}] m | '
            f'FRONT {f(sec.front)}  LEFT {f(sec.left)}  RIGHT {f(sec.right)}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
