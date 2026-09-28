"""trial_monitor — evaluates one navigation trial from Gazebo ground truth and writes the result.

Subscriptions
  /ground_truth/pose   geometry_msgs/PoseStamped  robot model world pose (gz PosePublisher)
  /odom                nav_msgs/Odometry   (to measure dead-reckoning drift)
  /nav/status          std_msgs/String     (navigator state + per-cycle compute time)

Metrics (definitions fixed BEFORE running experiments)
  reached          navigator reported REACHED before the timeout
  success          reached AND ground-truth goal error <= success_tol AND zero collisions
  time_s           simulated seconds from the first active control cycle to REACHED/timeout
  path_m           ground-truth path length (sum of planar steps > 1 mm)
  collisions       contact events: footprint circle (r=0.11 m) intersects obstacle geometry;
                   a new event needs the clearance to recover to > 0.02 m first (debounce)
  min_clear_m      minimum footprint-to-obstacle clearance over the trial
  goal_err_m       ground-truth distance base_footprint -> goal when the trial ends
  odom_err_m       |raw wheel-odometry pose - ground truth| at the end (dead-reckoning drift)
  est_err_m        |navigator's own pose estimate - ground truth| at the end (localization error;
                   equals odom_err_m when localization:=odom)
  est_err_max_m    largest localization error during the trial
  compute_mean/p95/max_ms  navigator decision time per control cycle (wall clock)
  rtf              simulated time / wall time (how fast the VM simulated)

When finished it appends one row to <results_dir>/results.csv, writes the trajectory to
<results_dir>/traces/<trial_id>.csv and exits (the trial launch file then shuts down).
"""
import csv
import json
import math
import os
import time

import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from tb3_nav_task.geometry import body_clearance
from tb3_nav_task.localization import compose
from tb3_nav_task.navigator import yaw_from_quat
from tb3_nav_task.worlds import WORLDS

FIELDS = ['trial_id', 'plan', 'world', 'strategy', 'variant', 'params', 'start_x', 'start_y',
          'start_yaw', 'outcome', 'reached', 'success', 'time_s', 'path_m', 'straight_m',
          'path_ratio', 'collisions', 'contact_s', 'min_clear_m', 'goal_err_m', 'odom_err_m', 'est_err_m', 'est_err_max_m', 'localization',
          'compute_mean_ms', 'compute_p95_ms', 'compute_max_ms', 'cycles', 'rtf', 'states',
          'wall_s', 'timestamp']


class TrialMonitor(Node):
    def __init__(self):
        super().__init__('trial_monitor')
        dp = self.declare_parameter
        dp('world', 'open_field')
        dp('trial_id', 'manual')
        dp('plan', 'manual')
        dp('strategy', '')
        dp('variant', 0)
        dp('params', '{}')
        dp('results_dir', os.path.expanduser('~/vm-share/task02/results'))
        dp('timeout', 200.0)          # simulated seconds
        dp('success_tol', 0.30)       # m, ground-truth goal tolerance
        dp('robot_name', 'burger')
        for n in ('start_x', 'start_y', 'start_yaw'):
            dp(n, float('nan'))
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.world = WORLDS[g('world')]
        self.meta = {k: g(k) for k in ('trial_id', 'plan', 'strategy', 'variant', 'params')}
        sx, sy, syaw = g('start_x'), g('start_y'), g('start_yaw')
        self.start = self.world.start if math.isnan(sx) else (sx, sy, syaw)
        self.results_dir = os.path.expanduser(g('results_dir'))
        self.timeout = float(g('timeout'))
        self.success_tol = float(g('success_tol'))
        self.robot = g('robot_name')

        self.gt = None
        self.odom_est = None
        self.t0 = None
        self.wall0 = None
        self.trace = []
        self.path = 0.0
        self.last_xy = None
        self.collisions = 0
        self.in_contact = False
        self.contact_time = 0.0
        self.last_t = None
        self.min_clear = math.inf
        self.compute = []
        self.states = {}
        self.nav_state = 'WAITING'
        self.est = None
        self.est_err_max = 0.0
        self.localization = ''
        self.done = False

        self.create_subscription(PoseStamped, '/ground_truth/pose', self.on_gt, 50)
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(String, '/nav/status', self.on_status, 50)
        self.create_timer(0.2, self.check_end)
        self.get_logger().info(f'monitoring trial {self.meta["trial_id"]} in {self.world.name}, '
                               f'timeout {self.timeout:.0f} s sim')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ callbacks
    def on_gt(self, msg: PoseStamped) -> None:
        tr, q = msg.pose.position, msg.pose.orientation
        self.gt = (tr.x, tr.y, yaw_from_quat(q))
        if self.t0 is None or self.done:
            return
        t = self.now_s()
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        x, y, yaw = self.gt
        if self.last_xy is not None:
            step = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if step > 0.001:
                self.path += step
                self.last_xy = (x, y)
        else:
            self.last_xy = (x, y)
        c = body_clearance(self.world, x, y, yaw)
        self.min_clear = min(self.min_clear, c)
        if c <= 0.0:
            self.contact_time += dt
            if not self.in_contact:
                self.collisions += 1
                self.in_contact = True
                self.get_logger().warn(f'COLLISION #{self.collisions} at ({x:.2f},{y:.2f})')
        elif c > 0.02:
            self.in_contact = False
        if not self.trace or t - self.trace[-1][0] >= 0.1:
            od = self.odom_est or (math.nan, math.nan, math.nan)
            es = self.est or (math.nan, math.nan, math.nan)
            self.trace.append((round(t - self.t0, 3), round(x, 4), round(y, 4), round(yaw, 4),
                               round(od[0], 4), round(od[1], 4), round(es[0], 4), round(es[1], 4),
                               round(c, 4), self.nav_state))

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.odom_est = compose(self.start, (p.position.x, p.position.y, yaw_from_quat(p.orientation)))

    def on_status(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        st = d.get('state', '?')
        self.nav_state = st
        self.localization = d.get('loc', self.localization)
        if 'est' in d:
            self.est = tuple(d['est'])
            if self.gt is not None and self.t0 is not None:
                self.est_err_max = max(self.est_err_max,
                                       math.hypot(self.est[0] - self.gt[0], self.est[1] - self.gt[1]))
        if st in ('WAITING', 'NO_DATA'):
            return
        if self.t0 is None:
            self.t0 = self.now_s()
            self.wall0 = time.monotonic()
            self.get_logger().info('trial started (first active control cycle)')
        if self.t0 is None:
            return
        self.states[st] = self.states.get(st, 0) + 1
        if d.get('compute_ms', 0) > 0:
            self.compute.append(float(d['compute_ms']))

    def check_end(self) -> None:
        if self.done or self.t0 is None:
            return
        elapsed = self.now_s() - self.t0
        if self.nav_state == 'REACHED':
            self.finish('REACHED', elapsed)
        elif elapsed > self.timeout:
            self.finish('TIMEOUT', elapsed)

    # ------------------------------------------------------------------ result
    def finish(self, outcome: str, elapsed: float) -> None:
        self.done = True
        gx, gy = self.world.goal
        x, y, _ = self.gt if self.gt else (math.nan, math.nan, 0)
        goal_err = math.hypot(gx - x, gy - y)
        odom_err = (math.hypot(self.odom_est[0] - x, self.odom_est[1] - y)
                    if self.odom_est else math.nan)
        est_err = (math.hypot(self.est[0] - x, self.est[1] - y) if self.est else math.nan)
        straight = math.hypot(gx - self.start[0], gy - self.start[1])
        reached = outcome == 'REACHED'
        success = reached and goal_err <= self.success_tol and self.collisions == 0
        comp = sorted(self.compute) or [math.nan]
        wall = time.monotonic() - self.wall0
        row = dict(self.meta)
        row.update({
            'world': self.world.name, 'start_x': self.start[0], 'start_y': self.start[1],
            'start_yaw': self.start[2], 'outcome': outcome, 'reached': int(reached),
            'success': int(success), 'time_s': round(elapsed, 2), 'path_m': round(self.path, 3),
            'straight_m': round(straight, 3), 'path_ratio': round(self.path / straight, 3),
            'collisions': self.collisions, 'contact_s': round(self.contact_time, 2),
            'min_clear_m': round(self.min_clear, 4), 'goal_err_m': round(goal_err, 4),
            'odom_err_m': round(odom_err, 4), 'est_err_m': round(est_err, 4),
            'est_err_max_m': round(self.est_err_max, 4), 'localization': self.localization,
            'compute_mean_ms': round(sum(comp) / len(comp), 4),
            'compute_p95_ms': round(comp[min(len(comp) - 1, int(0.95 * len(comp)))], 4),
            'compute_max_ms': round(comp[-1], 4), 'cycles': len(self.compute),
            'rtf': round(elapsed / wall, 3) if wall > 0 else math.nan,
            'states': json.dumps(self.states, sort_keys=True), 'wall_s': round(wall, 1),
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        })
        os.makedirs(os.path.join(self.results_dir, 'traces'), exist_ok=True)
        csv_path = os.path.join(self.results_dir, 'results.csv')
        new = not os.path.exists(csv_path)
        with open(csv_path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction='ignore')
            if new:
                w.writeheader()
            w.writerow(row)
        with open(os.path.join(self.results_dir, 'traces', f'{self.meta["trial_id"]}.csv'), 'w',
                  newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw', 'odom_x', 'odom_y', 'est_x', 'est_y', 'clearance', 'state'])
            w.writerows(self.trace)
        self.get_logger().info(
            f'RESULT {outcome}: success={success} time={elapsed:.1f}s path={self.path:.2f}m '
            f'collisions={self.collisions} min_clear={self.min_clear:.3f}m goal_err={goal_err:.3f}m '
            f'est_err={est_err:.3f}m odom_err={odom_err:.3f}m rtf={row["rtf"]}')
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = TrialMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
