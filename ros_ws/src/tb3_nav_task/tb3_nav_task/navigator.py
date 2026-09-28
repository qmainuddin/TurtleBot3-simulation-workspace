"""navigator — ROS 2 node that drives the TurtleBot3 to a goal using LiDAR + wheel odometry.

Subscriptions
  /odom  nav_msgs/Odometry     wheel odometry from the Gazebo DiffDrive plugin (30 Hz)
  /imu   sensor_msgs/Imu       gyro yaw rate (used when localization:=odom_imu)
  /scan  sensor_msgs/LaserScan 360-beam LiDAR, best-effort QoS, 5 Hz
Publications
  /cmd_vel     geometry_msgs/TwistStamped (TurtleBot3 Jazzy bridge) or Twist (auto-detected)
  /nav/status  std_msgs/String (JSON: state, goal distance, per-cycle compute time)

Pose estimate (parameter `localization`, see localization.py):
    odom      p_world = p_start (+) p_odom   (wheel odometry only; suffers from wheel slip)
    odom_imu  heading from the integrated IMU gyro, distance from wheel speed (default)
Ground truth is deliberately NOT used here; it is only used by trial_monitor for evaluation.

    ros2 run tb3_nav_task navigator --ros-args -p use_sim_time:=true \
        -p strategy:=bug2 -p goal_x:=2.8 -p goal_y:=1.8 -p start_x:=-2.8 -p start_y:=-1.8
"""
import json
import math
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from tb3_nav_task.localization import compose, make_estimator  # noqa: F401 (compose re-exported)
from tb3_nav_task.planners import CommandShaper, ScanPoints, make_planner

TUNABLE = ['v_max', 'w_max', 'safety', 'lookahead', 'k_w', 'slow_dist', 'stop_dist',
           'hit_dist', 'd_follow', 'k_head', 'k_wall', 'mline_tol', 'leave_progress', 'goal_tol']


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.declare_parameter('strategy', 'vfh')
        for name in ('goal_x', 'goal_y', 'start_x', 'start_y', 'start_yaw'):
            self.declare_parameter(name, 0.0)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('scan_timeout', 1.0)      # s (LiDAR is 5 Hz)
        self.declare_parameter('cmd_vel_type', 'auto')   # auto | stamped | unstamped
        self.declare_parameter('localization', 'odom_imu')  # odom | odom_imu
        self.declare_parameter('shaping', 1.0)   # 1 = rate-limit (v, w) (CommandShaper), 0 = raw
        overrides = {}
        for name in TUNABLE:
            self.declare_parameter(name, -1.0)           # -1 = keep the planner default
            val = self.get_parameter(name).value
            if val is not None and val >= 0.0:
                overrides[name] = float(val)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.strategy = gp('strategy')
        self.planner = make_planner(self.strategy, **overrides)
        self.goal = (float(gp('goal_x')), float(gp('goal_y')))
        self.start = (float(gp('start_x')), float(gp('start_y')), float(gp('start_yaw')))
        self.scan_timeout = float(gp('scan_timeout'))
        self.cmd_type = gp('cmd_vel_type')
        self.localization = gp('localization')
        self.estimator = make_estimator(self.localization, self.start)
        self.shaper = CommandShaper(1.0 / float(gp('rate_hz')), enabled=float(gp('shaping')) > 0.5)

        self.pose = None
        self.scan_msg = None
        self.scan_stamp = None
        self.pub = None
        self.stamped = None
        self.state = 'WAITING'
        self.t_created = time.monotonic()

        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Imu, '/imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/nav/status', 10)
        self.create_timer(1.0 / float(gp('rate_hz')), self.on_timer)
        self.get_logger().info(
            f'navigator: strategy={self.strategy} localization={self.localization} '
            f'shaping={self.shaper.enabled} params={vars(self.planner.p)} '
            f'start={self.start} goal={self.goal}')

    # ------------------------------------------------------------------ I/O
    def _ensure_publisher(self) -> bool:
        if self.pub is not None:
            return True
        if self.cmd_type in ('stamped', 'unstamped'):
            self.stamped = self.cmd_type == 'stamped'
        else:
            types = dict(self.get_topic_names_and_types()).get('/cmd_vel', [])
            if 'geometry_msgs/msg/TwistStamped' in types:
                self.stamped = True
            elif 'geometry_msgs/msg/Twist' in types:
                self.stamped = False
            elif time.monotonic() - self.t_created > 5.0:
                self.stamped = True
                self.get_logger().warn('/cmd_vel type unknown; defaulting to TwistStamped')
            else:
                return False
        self.pub = self.create_publisher(TwistStamped if self.stamped else Twist, '/cmd_vel', 10)
        self.get_logger().info(f'publishing {"TwistStamped" if self.stamped else "Twist"} on /cmd_vel')
        return True

    def send(self, v: float, w: float) -> None:
        if self.pub is None:
            return
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_footprint'
            m.twist.linear.x, m.twist.angular.z = float(v), float(w)
        else:
            m = Twist()
            m.linear.x, m.angular.z = float(v), float(w)
        self.pub.publish(m)

    @staticmethod
    def _stamp(msg) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.estimator.on_odom(self._stamp(msg),
                               (p.position.x, p.position.y, yaw_from_quat(p.orientation)),
                               msg.twist.twist.linear.x)
        self.pose = self.estimator.pose

    def on_imu(self, msg: Imu) -> None:
        self.estimator.on_gyro(self._stamp(msg), msg.angular_velocity.z)

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg
        self.scan_stamp = self.get_clock().now()

    # ------------------------------------------------------------------ control loop
    def on_timer(self) -> None:
        if not self._ensure_publisher():
            return
        now = self.get_clock().now()
        fresh = (self.scan_stamp is not None and
                 (now - self.scan_stamp).nanoseconds * 1e-9 <= self.scan_timeout)
        if self.pose is None or not fresh:
            self.send(0.0, 0.0)
            self.publish_status('WAITING' if self.state == 'WAITING' else 'NO_DATA', {}, 0.0)
            return
        if self.state == 'REACHED':
            self.send(0.0, 0.0)
            self.publish_status('REACHED', {}, 0.0)
            return

        t0 = time.perf_counter()
        s = self.scan_msg
        pts = ScanPoints.from_scan(s.ranges, s.angle_min, s.angle_increment, s.range_min, s.range_max)
        cmd = self.planner.step(self.pose, self.goal, pts)
        compute_ms = (time.perf_counter() - t0) * 1e3

        v, w = self.shaper(cmd.v, cmd.w)
        self.send(v, w)
        if cmd.state != self.state:
            self.get_logger().info(
                f'{self.state} -> {cmd.state}  pose=({self.pose[0]:.2f},{self.pose[1]:.2f},'
                f'{math.degrees(self.pose[2]):.0f}deg) goal_dist={cmd.info.get("goal_dist", 0):.2f}')
        self.state = cmd.state
        self.publish_status(cmd.state, cmd.info, compute_ms, v, w)

    def publish_status(self, state, info, compute_ms, v=0.0, w=0.0) -> None:
        d = {'state': state, 'compute_ms': round(compute_ms, 3), 'v': round(v, 3), 'w': round(w, 3),
             'strategy': self.strategy, 'loc': self.localization}
        if self.pose is not None:
            d['est'] = [round(self.pose[0], 4), round(self.pose[1], 4), round(self.pose[2], 4)]
        d.update({k: round(val, 4) for k, val in info.items()})
        self.status_pub.publish(String(data=json.dumps(d)))

    def stop(self) -> None:
        for _ in range(3):
            self.send(0.0, 0.0)


def main(args=None):
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
