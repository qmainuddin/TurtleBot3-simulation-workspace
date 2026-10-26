"""Goal-reaching navigation strategies for a differential-drive robot with a 2D LiDAR.

Pure Python (no ROS) so they can be unit-tested and run in the offline 2D simulator.

Common interface
----------------
    planner.step(pose, goal, scan) -> Command(v, w, state, info)
      pose : (x, y, yaw) of base_footprint in the WORLD frame, estimated from wheel odometry
      goal : (gx, gy) in the world frame
      scan : ScanPoints (LiDAR returns transformed into the base_footprint frame)
      v    : forward speed [m/s]   w : yaw rate [rad/s] (positive = counter-clockwise / left)

Strategies
----------
GapFollower ("VFH-lite", reactive): builds a polar blocked/free histogram of headings, enlarging
    every LiDAR return by the robot radius + a safety margin, then steers toward the free heading
    closest to the goal bearing. No memory, so it can be trapped by concave obstacles (local minima).
Bug2 (Lumelsky & Stepanov 1987): drive along the start-goal line (m-line); when blocked, follow the
    obstacle boundary (wall on the right) until the m-line is met again closer to the goal.
    Complete for static 2D worlds given perfect sensing and localisation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from tb3_nav_task.geometry import FOOTPRINT_RADIUS, LIDAR_OFFSET_X, wrap

Pose = Tuple[float, float, float]


def clamp(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


# --------------------------------------------------------------------------------- scan model
@dataclass
class ScanPoints:
    """LiDAR returns in the base_footprint frame: bearing b, range d, cartesian (px, py)."""
    b: List[float] = field(default_factory=list)
    d: List[float] = field(default_factory=list)
    px: List[float] = field(default_factory=list)
    py: List[float] = field(default_factory=list)

    @staticmethod
    def from_scan(ranges: Sequence[float], angle_min: float, angle_increment: float,
                  range_min: float, range_max: float,
                  lidar_offset_x: float = LIDAR_OFFSET_X) -> 'ScanPoints':
        """Keep only real returns. +inf (no hit) is free space and is not stored; NaN and
        readings below range_min are invalid and dropped."""
        s = ScanPoints()
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < range_min or r > range_max:
                continue
            a = angle_min + i * angle_increment
            x = lidar_offset_x + r * math.cos(a)
            y = r * math.sin(a)
            s.px.append(x)
            s.py.append(y)
            s.d.append(math.hypot(x, y))
            s.b.append(math.atan2(y, x))
        return s

    def corridor_free(self, heading: float, half_width: float, max_dist: float = 3.5) -> float:
        """Free travel distance of a straight corridor of given half-width along `heading`
        (robot frame). Measured from the robot origin to the nearest return inside the corridor."""
        c, s = math.cos(heading), math.sin(heading)
        best = max_dist
        for x, y in zip(self.px, self.py):
            along = c * x + s * y
            across = -s * x + c * y
            if along > 0.0 and abs(across) < half_width and along < best:
                best = along
        return best

    def wall_angle(self, lo: float, hi: float, max_dist: float, min_points: int = 8) -> Optional[float]:
        """Direction of the wall seen in the bearing sector [lo, hi] (robot frame, radians,
        wrapped to [-pi/2, pi/2]): principal axis of the returns closer than max_dist.
        0 = wall parallel to the robot's heading; > 0 = wall rotated counter-clockwise, i.e. the
        robot is yawed toward a wall on its right. None if too few returns."""
        pts = [(x, y) for b, d, x, y in zip(self.b, self.d, self.px, self.py)
               if lo <= b <= hi and d < max_dist]
        if len(pts) < min_points:
            return None
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        sxx = sum((p[0] - mx) ** 2 for p in pts)
        syy = sum((p[1] - my) ** 2 for p in pts)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
        a = 0.5 * math.atan2(2.0 * sxy, sxx - syy)          # principal-axis angle
        if a > math.pi / 2:
            a -= math.pi
        elif a < -math.pi / 2:
            a += math.pi
        return a

    def sector_min(self, lo: float, hi: float, default: float = math.inf) -> float:
        best = default
        for b, d in zip(self.b, self.d):
            if lo <= b <= hi and d < best:
                best = d
        return best


@dataclass
class Command:
    v: float
    w: float
    state: str
    info: Dict[str, float] = field(default_factory=dict)


def goal_polar(pose: Pose, goal: Tuple[float, float]) -> Tuple[float, float]:
    """Distance and bearing (robot frame) to the goal."""
    x, y, yaw = pose
    dx, dy = goal[0] - x, goal[1] - y
    return math.hypot(dx, dy), wrap(math.atan2(dy, dx) - yaw)


FRONT_EDGE = 0.04  # base collision box front face is ~0.038 m ahead of base_footprint


# ------------------------------------------------------------------------ strategy 1: VFH-lite
@dataclass
class GapParams:
    v_max: float = 0.18          # m/s   (Burger limit 0.22)
    w_max: float = 1.5           # rad/s (Burger limit 2.84)
    safety: float = 0.12         # m     margin added to the robot radius when enlarging returns
    lookahead: float = 1.0       # m     only returns closer than this block headings
    bin_deg: float = 5.0         # deg   histogram resolution
    k_w: float = 2.0             # 1/s   heading-error gain
    w_goal: float = 1.0          # cost weight: deviation from goal bearing
    w_turn: float = 0.25         # cost weight: deviation from current heading
    w_prev: float = 0.25         # cost weight: deviation from previously chosen heading
    slow_dist: float = 0.60      # m     start slowing when free distance ahead < this
    stop_dist: float = 0.10      # m     free distance at which forward motion stops
    goal_tol: float = 0.20       # m


class GapFollower:
    name = 'vfh'

    def __init__(self, p: GapParams = GapParams()):
        self.p = p
        n = int(round(360.0 / p.bin_deg))
        self.bins = [wrap(math.radians(-180.0 + i * p.bin_deg)) for i in range(n)]
        self.prev: Optional[float] = None
        self.escape_dir = 0

    def blocked_bins(self, scan: ScanPoints, horizon: float) -> List[bool]:
        p = self.p
        blocked = [False] * len(self.bins)
        inflate = FOOTPRINT_RADIUS + p.safety
        for b, d in zip(scan.b, scan.d):
            if d > horizon:
                continue
            half = math.pi / 2 if d <= inflate else math.asin(inflate / d)
            for i, a in enumerate(self.bins):
                if not blocked[i] and abs(wrap(a - b)) <= half:
                    blocked[i] = True
        return blocked

    def step(self, pose: Pose, goal: Tuple[float, float], scan: ScanPoints) -> Command:
        p = self.p
        dist, bearing = goal_polar(pose, goal)
        if dist < p.goal_tol:
            return Command(0.0, 0.0, 'REACHED', {'goal_dist': dist})

        horizon = min(p.lookahead, dist + FOOTPRINT_RADIUS)
        blocked = self.blocked_bins(scan, horizon)
        best, best_cost = None, math.inf
        for a, blk in zip(self.bins, blocked):
            if blk:
                continue
            cost = p.w_goal * abs(wrap(a - bearing)) + p.w_turn * abs(a)
            if self.prev is not None:
                cost += p.w_prev * abs(wrap(a - self.prev))
            if cost < best_cost:
                best, best_cost = a, cost

        if best is None:  # boxed in: rotate toward the side with more room, keep direction
            if self.escape_dir == 0:
                left = scan.sector_min(0.3, 2.0, 3.5)
                right = scan.sector_min(-2.0, -0.3, 3.5)
                self.escape_dir = 1 if left >= right else -1
            self.prev = None
            return Command(0.0, self.escape_dir * p.w_max, 'ESCAPE', {'goal_dist': dist})

        self.escape_dir = 0
        self.prev = best
        err = best
        w = clamp(p.k_w * err, p.w_max)
        ahead = scan.corridor_free(0.0, FOOTPRINT_RADIUS + 0.02) - FRONT_EDGE
        speed_scale = max(0.0, min(1.0, (ahead - p.stop_dist) / (p.slow_dist - p.stop_dist)))
        v = p.v_max * max(0.0, math.cos(err)) ** 2 * speed_scale
        if abs(err) > 1.2:
            v = 0.0
        v = min(v, 0.6 * dist + 0.04)  # arrive gently
        state = 'GO_GOAL' if abs(wrap(best - bearing)) < math.radians(p.bin_deg) else 'AVOID'
        return Command(v, w, state, {'goal_dist': dist, 'heading_cmd': best, 'ahead': ahead})


# ---------------------------------------------------------------------------- strategy 2: Bug2
@dataclass
class Bug2Params:
    v_max: float = 0.18          # m/s
    w_max: float = 1.5           # rad/s
    hit_dist: float = 0.30       # m   free distance ahead that triggers boundary following
    d_follow: float = 0.30       # m   desired distance from the followed boundary (from base origin)
    k_head: float = 2.0          # 1/s heading gain in GO_GOAL
    k_wall: float = 3.0          # rad/s per m, wall-distance gain
    mline_tol: float = 0.10      # m   |distance to m-line| accepted as "on the m-line"
    leave_progress: float = 0.15 # m   must be this much closer to goal than at the hit point
    min_follow: float = 0.50     # m   travel along the boundary before a leave is allowed
    goal_tol: float = 0.20       # m
    k_align: float = 0.0         # rad/s per rad: wall-alignment (heading) term of the wall
                                 #   follower; 0 = original P-only follower used in plans A-F
    turn_first: bool = False     # after LEAVE, rotate toward the goal before a new hit can be
                                 #   declared (False = original rule used in plans A-H)


class Bug2:
    name = 'bug2'

    def __init__(self, p: Bug2Params = Bug2Params()):
        self.p = p
        self.state = 'GO_GOAL'
        self.start: Optional[Tuple[float, float]] = None
        self.hit_goal_dist = math.inf
        self.follow_travel = 0.0
        self.last_xy: Optional[Tuple[float, float]] = None
        self.last_yaw: Optional[float] = None
        self.arc_turn = 0.0
        self.hits = 0
        self.abandons = 0

    def mline_distance(self, x: float, y: float, goal: Tuple[float, float]) -> float:
        sx, sy = self.start
        gx, gy = goal
        L = math.hypot(gx - sx, gy - sy)
        if L < 1e-6:
            return math.hypot(x - sx, y - sy)
        return abs((gx - sx) * (sy - y) - (sx - x) * (gy - sy)) / L

    def step(self, pose: Pose, goal: Tuple[float, float], scan: ScanPoints) -> Command:
        p = self.p
        x, y, yaw = pose
        if self.start is None:
            self.start = (x, y)
        if self.last_xy is not None and self.state == 'FOLLOW':
            self.follow_travel += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        dyaw = 0.0 if self.last_yaw is None else abs(wrap(yaw - self.last_yaw))
        self.last_xy, self.last_yaw = (x, y), yaw

        dist, bearing = goal_polar(pose, goal)
        info = {'goal_dist': dist, 'mline_dist': self.mline_distance(x, y, goal)}
        if dist < p.goal_tol:
            self.state = 'REACHED'
            return Command(0.0, 0.0, 'REACHED', info)

        half = FOOTPRINT_RADIUS + 0.03
        ahead = scan.corridor_free(0.0, half) - FRONT_EDGE
        info['ahead'] = ahead

        if self.state == 'GO_GOAL':
            # Original rule: a hit is declared from the free distance along the CURRENT heading.
            # Right after LEAVE the robot still faces along the wall it just left, so a wall ahead
            # can trigger an immediate re-hit with a new (closer) hit point; near the goal this
            # makes it circle the goal room. turn_first: no hit while still turning in place.
            turning = abs(bearing) > 0.8
            if ahead < p.hit_dist and ahead < dist and not (p.turn_first and turning):
                self.state = 'FOLLOW'
                self.hit_goal_dist = dist
                self.follow_travel = 0.0
                self.hits += 1
            else:
                w = clamp(p.k_head * bearing, p.w_max)
                v = 0.0 if abs(bearing) > 0.8 else p.v_max * math.cos(bearing) ** 2
                v = min(v, 0.6 * dist + 0.04, p.v_max * max(0.0, (ahead - 0.05) / 0.4))
                return Command(max(v, 0.0), w, 'GO_GOAL', info)

        # FOLLOW: keep the boundary on the robot's RIGHT (counter-clockwise circumnavigation).
        on_mline = info['mline_dist'] < p.mline_tol
        closer = dist < self.hit_goal_dist - p.leave_progress
        if self.follow_travel > p.min_follow and on_mline and closer:
            goal_free = scan.corridor_free(bearing, half) - FRONT_EDGE
            if goal_free > min(dist, p.hit_dist + 0.15):
                self.state = 'GO_GOAL'
                w = clamp(p.k_head * bearing, p.w_max)
                return Command(0.0, w, 'LEAVE', info)

        right = scan.sector_min(math.radians(-120), math.radians(-60), 3.5)
        diag = scan.sector_min(math.radians(-60), math.radians(-20), 3.5)
        info['right'] = right
        if ahead < p.d_follow - 0.02:                         # blocked ahead: turn left in place
            return Command(0.0, 0.8 * p.w_max, 'FOLLOW_TURN', info)
        if right > 2.0 * p.d_follow and diag > 1.5 * p.d_follow:  # lost the wall: arc right
            self.arc_turn += dyaw
            if self.arc_turn > 2.0 * math.pi:
                # A full circle without re-acquiring a boundary: the obstacle is gone (e.g. a
                # small pillar left behind). Stop following and head for the goal again.
                self.state = 'GO_GOAL'
                self.arc_turn = 0.0
                self.abandons += 1
                return Command(0.0, 0.0, 'ABANDON', info)
            v = 0.5 * p.v_max
            return Command(v, -v / max(p.d_follow, 0.2), 'FOLLOW_ARC', info)
        self.arc_turn = 0.0
        side = min(right, diag / math.cos(math.radians(40)))  # anticipate the wall ahead-right
        err = p.d_follow - side                                # > 0: too close -> turn left
        w = p.k_wall * err
        if p.k_align > 0.0:
            # PD-like wall following: also steer to stay PARALLEL to the wall. The P-only rule
            # reacts to distance alone and weaves (+/-0.1 m); in a 0.65 m corridor a weave toward
            # the opposite wall can trip FOLLOW_TURN and make the robot U-turn.
            phi = scan.wall_angle(math.radians(-115), math.radians(-65), 2.0 * p.d_follow + 0.2)
            if phi is not None:
                w += p.k_align * phi
        w = clamp(w, p.w_max)
        v = p.v_max * max(0.3, 1.0 - abs(err) / p.d_follow)
        v = min(v, p.v_max * max(0.2, (ahead - 0.05) / 0.4))
        return Command(v, w, 'FOLLOW', info)


# ------------------------------------------------------------------------ command shaping
@dataclass
class ShaperParams:
    a_lin: float = 0.5           # m/s^2   max forward acceleration
    d_lin: float = 1.0           # m/s^2   max deceleration (= DiffDrive plugin limit; braking not delayed)
    a_ang: float = 2.0           # rad/s^2 max change of yaw rate
    w_spin: float = 0.6          # rad/s   max yaw rate while (almost) not translating


class CommandShaper:
    """Rate-limits (v, w) before they reach the robot.

    Pilot trials showed that step changes of yaw rate up to 1.5 rad/s make the Burger's tyres skid
    sideways in Gazebo: the robot translated 0.5 m while commanded to turn on the spot, and neither
    the wheel encoders nor the gyro see that motion, so localization error grew by ~0.2 m per wall
    corner (see explanation.md). Limiting angular acceleration and the in-place turn rate keeps the
    tyre forces below the friction limit.
    """

    def __init__(self, dt: float, p: ShaperParams = ShaperParams(), enabled: bool = True):
        self.dt, self.p, self.enabled = dt, p, enabled
        self.v = 0.0
        self.w = 0.0

    def __call__(self, v: float, w: float) -> Tuple[float, float]:
        if not self.enabled:
            return v, w
        p, dt = self.p, self.dt
        if abs(v) < 0.03:
            w = clamp(w, p.w_spin)
        dv = v - self.v
        dv = min(dv, p.a_lin * dt) if dv > 0 else max(dv, -p.d_lin * dt)
        self.v += dv
        self.w += clamp(w - self.w, p.a_ang * dt)
        return self.v, self.w

# ---------------------------------------------------------------------------- strategy 3: A* Global Planner
@dataclass
class AStarParams:
    v_max: float = 0.18            # m/s
    w_max: float = 1.5             # rad/s
    resolution: float = 0.05       # m per grid cell
    robot_radius: float = 0.15     # m inflation radius
    goal_tol: float = 0.20         # m
    waypoint_tol: float = 0.30     # m distance to switch to next waypoint
    k_w: float = 2.0               # heading gain


class OccupancyGridMap:
    """Discrete 2D occupancy grid map built from LiDAR scans and robot poses."""
    def __init__(self, x_min: float = -4.0, x_max: float = 4.0,
                 y_min: float = -3.0, y_max: float = 3.0,
                 resolution: float = 0.05, inflation_radius: float = 0.15):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.res = resolution
        self.width = int(round((x_max - x_min) / resolution))
        self.height = int(round((y_max - y_min) / resolution))
        
        # 0: free, 1: occupied, -1: unknown
        self.grid = [[0 for _ in range(self.height)] for _ in range(self.width)]
        self.inflation_cells = int(math.ceil(inflation_radius / resolution))

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(round((x - self.x_min) / self.res))
        gy = int(round((y - self.y_min) / self.res))
        return max(0, min(self.width - 1, gx)), max(0, min(self.height - 1, gy))

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = self.x_min + gx * self.res
        y = self.y_min + gy * self.res
        return x, y

    def update_from_scan(self, pose: Pose, scan: ScanPoints):
        rx, ry, ryaw = pose
        rgx, rgy = self.world_to_grid(rx, ry)
        
        # Transform LiDAR points to world frame and mark occupied
        c, s = math.cos(ryaw), math.sin(ryaw)
        for px, py in zip(scan.px, scan.py):
            wx = rx + c * px - s * py
            wy = ry + s * px + c * py
            gx, gy = self.world_to_grid(wx, wy)
            
            # Inflate obstacle cells for safety margin
            for dx in range(-self.inflation_cells, self.inflation_cells + 1):
                for dy in range(-self.inflation_cells, self.inflation_cells + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        self.grid[nx][ny] = 1

    def is_free(self, gx: int, gy: int) -> bool:
        if not (0 <= gx < self.width and 0 <= gy < self.height):
            return False
        return self.grid[gx][gy] == 0


class AStarPlanner:
    name = 'astar'

    def __init__(self, p: AStarParams = AStarParams()):
        self.p = p
        self.map = OccupancyGridMap(resolution=p.resolution, inflation_radius=p.robot_radius)
        self.path: List[Tuple[float, float]] = []
        self.last_goal: Optional[Tuple[float, float]] = None

    def plan(self, start_pose: Pose, goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        sx, sy, _ = start_pose
        sgx, sgy = self.map.world_to_grid(sx, sy)
        ggx, ggy = self.map.world_to_grid(goal[0], goal[1])

        if not self.map.is_free(ggx, ggy):
            # If goal is blocked/unknown, find nearest free cell
            ggx, ggy = self._find_nearest_free(ggx, ggy)

        import heapq
        open_set = []
        heapq.heappush(open_set, (0.0, 0, sgx, sgy))
        
        came_from = {}
        g_score = {(sgx, sgy): 0.0}
        f_score = {(sgx, sgy): math.hypot(ggx - sgx, ggy - sgy)}
        
        closed_set = set()
        neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                     (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

        while open_set:
            _, _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (ggx, ggy):
                # Reconstruct path
                path_grid = []
                curr = (cx, cy)
                while curr in came_from:
                    path_grid.append(curr)
                    curr = came_from[curr]
                path_grid.reverse()
                return [self.map.grid_to_world(gx, gy) for gx, gy in path_grid]

            closed_set.add((cx, cy))

            for dx, dy, cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if not self.map.is_free(nx, ny) or (nx, ny) in closed_set:
                    continue

                tentative_g = g_score[(cx, cy)] + cost
                if tentative_g < g_score.get((nx, ny), math.inf):
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + math.hypot(ggx - nx, ggy - ny)
                    f_score[(nx, ny)] = f
                    heapq.heappush(open_set, (f, id((nx, ny)), nx, ny))

        return [] # No path found

    def _find_nearest_free(self, gx: int, gy: int) -> Tuple[int, int]:
        for r in range(1, 20):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = gx + dx, gy + dy
                    if self.map.is_free(nx, ny):
                        return nx, ny
        return gx, gy

    def step(self, pose: Pose, goal: Tuple[float, float], scan: ScanPoints) -> Command:
        p = self.p
        dist, bearing = goal_polar(pose, goal)
        if dist < p.goal_tol:
            return Command(0.0, 0.0, 'REACHED', {'goal_dist': dist})

        # Update occupancy grid map with current scan
        self.map.update_from_scan(pose, scan)

        # Re-plan if path is empty or goal changed significantly
        if not self.path or self.last_goal != goal or len(self.path) == 0:
            self.path = self.plan(pose, goal)
            self.last_goal = goal

        # Drop waypoints that are behind or too close
        while self.path and math.hypot(self.path[0][0] - pose[0], self.path[0][1] - pose[1]) < p.waypoint_tol:
            self.path.pop(0)

        if not self.path:
            # Fallback if A* fails: rotate in place to scan/clear
            return Command(0.0, p.w_max, 'ESCAPE', {'goal_dist': dist})

        # Target the next waypoint in the A* path
        target_wp = self.path[0]
        wp_dist, wp_bearing = goal_polar(pose, target_wp)

        w = clamp(p.k_w * wp_bearing, p.w_max)
        v = p.v_max * max(0.0, math.cos(wp_bearing)) ** 2
        v = min(v, 0.6 * dist + 0.04)

        return Command(v, w, 'GO_GOAL', {'goal_dist': dist, 'path_len': len(self.path)})


def make_planner(name: str, **overrides):
    """Factory used by the ROS node: make_planner('vfh', v_max=0.12)."""
    if name == 'vfh':
        params = GapParams(**{k: v for k, v in overrides.items() if hasattr(GapParams, k)})
        return GapFollower(params)
    if name == 'bug2':
        params = Bug2Params(**{k: v for k, v in overrides.items() if hasattr(Bug2Params, k)})
        return Bug2(params)
    if name == 'astar':
        params = AStarParams(**{k: v for k, v in overrides.items() if hasattr(AStarParams, k)})
        return AStarPlanner(params)
    raise ValueError(f'unknown strategy {name!r} (use vfh, bug2, or astar)')
