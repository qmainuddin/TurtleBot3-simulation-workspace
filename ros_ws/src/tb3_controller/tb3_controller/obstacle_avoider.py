"""obstacle_avoider — reactive LiDAR controller node for a simulated TurtleBot3.

Data flow
---------
Gazebo LiDAR --(ros_gz_bridge)--> /scan  (sensor_msgs/LaserScan)
    -> this node: sectors -> decide() -> velocity
    -> /cmd_vel (geometry_msgs/TwistStamped on TurtleBot3 Jazzy, Twist on older setups)
    --(ros_gz_bridge)--> Gazebo diff-drive plugin -> wheels move -> new /scan

Safety behaviour
----------------
* No scan yet, or last scan older than `scan_timeout` s -> publish zero velocity.
* Ctrl+C -> publishes a zero command before exiting, so the robot does not coast.

Run (after sourcing the workspace):
    ros2 run tb3_controller obstacle_avoider --ros-args -p use_sim_time:=true
"""
import math

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from tb3_controller.decision import Params, Sectors, compute_sectors, decide

TWIST_STAMPED = 'geometry_msgs/msg/TwistStamped'
TWIST = 'geometry_msgs/msg/Twist'


class ObstacleAvoider(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')
        d = Params()
        self.declare_parameter('max_linear', d.max_linear)
        self.declare_parameter('max_angular', d.max_angular)
        self.declare_parameter('stop_distance', d.stop_distance)
        self.declare_parameter('slow_distance', d.slow_distance)
        self.declare_parameter('wall_distance', d.wall_distance)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('scan_timeout', 0.5)
        # 'auto' inspects the ROS graph for the type the bridge expects on /cmd_vel.
        self.declare_parameter('cmd_vel_type', 'auto')   # auto | stamped | unstamped
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        gp = self.get_parameter
        self.params = Params(
            max_linear=gp('max_linear').value,
            max_angular=gp('max_angular').value,
            stop_distance=gp('stop_distance').value,
            slow_distance=gp('slow_distance').value,
            wall_distance=gp('wall_distance').value,
        )
        self.scan_timeout = float(gp('scan_timeout').value)
        self.cmd_topic = gp('cmd_vel_topic').value
        self.cmd_type_param = gp('cmd_vel_type').value

        self.scan = None
        self.scan_time = None
        self.turn_dir = 0
        self.state = None
        self.pub = None
        self.stamped = None
        self.start_time = self.get_clock().now()

        # LaserScan is published "best effort"; the sensor-data QoS profile matches it.
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(1.0 / float(gp('rate_hz').value), self.on_timer)
        self.get_logger().info(
            f'obstacle_avoider started: v_max={self.params.max_linear} m/s, '
            f'w_max={self.params.max_angular} rad/s, stop<{self.params.stop_distance} m, '
            f'slow<{self.params.slow_distance} m')

    # ---------------------------------------------------------------- publisher setup
    def _ensure_publisher(self) -> bool:
        if self.pub is not None:
            return True
        choice = self.cmd_type_param
        if choice == 'stamped':
            self.stamped = True
        elif choice == 'unstamped':
            self.stamped = False
        else:  # auto: look at who already uses the topic (the Gazebo bridge subscribes to it)
            types = dict(self.get_topic_names_and_types()).get(self.cmd_topic, [])
            if TWIST_STAMPED in types:
                self.stamped = True
            elif TWIST in types:
                self.stamped = False
            elif (self.get_clock().now() - self.start_time).nanoseconds > 3e9:
                self.stamped = True
                self.get_logger().warn(
                    f'{self.cmd_topic} not seen yet; defaulting to TwistStamped (TurtleBot3 Jazzy). '
                    'Override with -p cmd_vel_type:=unstamped if the robot does not move.')
            else:
                return False  # keep waiting for the simulator to appear
        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, self.cmd_topic, 10)
        self.get_logger().info(f'Publishing {msg_type.__name__} on {self.cmd_topic}')
        return True

    def publish(self, v: float, w: float) -> None:
        if self.pub is None:
            return
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x = float(v)
            msg.twist.angular.z = float(w)
        else:
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
        self.pub.publish(msg)

    # ---------------------------------------------------------------- callbacks
    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg
        self.scan_time = self.get_clock().now()

    def on_timer(self) -> None:
        if not self._ensure_publisher():
            return

        now = self.get_clock().now()
        stale = (self.scan is None or
                 (now - self.scan_time).nanoseconds * 1e-9 > self.scan_timeout)
        if stale:
            sectors = Sectors(None, None, None)
        else:
            s = self.scan
            sectors = compute_sectors(s.ranges, s.angle_min, s.angle_increment,
                                      s.range_min, s.range_max, self.params)

        v, w, state, self.turn_dir = decide(sectors, self.params, self.turn_dir)
        self.publish(v, w)

        if state != self.state:
            self.state = state
            self.get_logger().info(
                f'{state:9s} front={_fmt(sectors.front)} left={_fmt(sectors.left)} '
                f'right={_fmt(sectors.right)} -> v={v:.2f} m/s w={w:+.2f} rad/s')

    def stop(self) -> None:
        for _ in range(3):
            self.publish(0.0, 0.0)


def _fmt(x):
    return '  n/a' if x is None or not math.isfinite(x) else f'{x:5.2f}'


def main(args=None):
    # Handle Ctrl+C ourselves so the context is still alive to send a final stop command.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
            node.get_logger().info('Stopped: zero velocity sent.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
