"""Minimal 2D kinematic simulator used to unit-test and pre-tune the planners without Gazebo.

It mimics the Gazebo TurtleBot3 Burger set-up where it matters for control:
  * LiDAR: 360 beams over [0, 6.28] rad, 5 Hz, range 0.12-3.5 m, Gaussian noise sigma 0.01 m,
    mounted 0.032 m behind base_footprint
  * DiffDrive: unicycle kinematics, max linear acceleration 1.0 m/s^2 (as in model.sdf)
  * Controller at 10 Hz
It is NOT a physics simulator (no wheel slip, inertia or contact dynamics): contact simply blocks
motion and is counted. Gazebo remains the evaluation platform; this is a fast pre-check.

    python3 -m tb3_nav_task.sim2d vfh open_field
"""
from __future__ import annotations

import math
import random
import sys
from dataclasses import dataclass, field
from typing import List, Tuple

from tb3_nav_task.geometry import LIDAR_OFFSET_X, WorldSpec, body_clearance, path_length
from tb3_nav_task.planners import CommandShaper, ScanPoints, make_planner


@dataclass
class SimResult:
    success: bool
    time: float
    path: float
    collisions: int
    min_clearance: float
    final_goal_dist: float
    states: dict = field(default_factory=dict)
    trace: List[Tuple[float, float]] = field(default_factory=list)


def simulate(world: WorldSpec, strategy: str, start=None, seed: int = 0, timeout: float = 180.0,
             shaping: bool = True, **params) -> SimResult:
    rng = random.Random(seed)
    x, y, th = start if start is not None else world.start
    planner = make_planner(strategy, **params)
    dt, ctrl_dt, scan_dt = 0.01, 0.1, 0.2
    shaper = CommandShaper(ctrl_dt, enabled=shaping)
    v_cmd = w_cmd = v = 0.0
    t = next_ctrl = next_scan = 0.0
    scan = ScanPoints()
    trace = [(x, y)]
    collisions, in_contact, min_clear = 0, False, math.inf
    states = {}
    n, inc = 360, 6.28 / 359
    while t < timeout:
        if t >= next_scan:
            lx, ly = x + LIDAR_OFFSET_X * math.cos(th), y + LIDAR_OFFSET_X * math.sin(th)
            ranges = []
            for i in range(n):
                r = world.raycast(lx, ly, th + i * inc, 3.5)
                ranges.append(r + rng.gauss(0, 0.01) if math.isfinite(r) else r)
            scan = ScanPoints.from_scan(ranges, 0.0, inc, 0.12, 3.5)
            next_scan += scan_dt
        if t >= next_ctrl:
            cmd = planner.step((x, y, th), world.goal, scan)
            v_cmd, w_cmd = shaper(cmd.v, cmd.w)
            states[cmd.state] = states.get(cmd.state, 0) + 1
            next_ctrl += ctrl_dt
            if cmd.state == 'REACHED':
                break
        dv = max(-1.0 * dt, min(1.0 * dt, v_cmd - v))
        v += dv
        nx, ny, nth = x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w_cmd * dt
        c = body_clearance(world, nx, ny, nth)
        if c <= 0.0:
            if not in_contact:
                collisions += 1
            in_contact, v = True, 0.0
            nx, ny = x, y                       # contact blocks translation
            if body_clearance(world, nx, ny, nth) <= 0.0:
                nth = th
        elif c > 0.02:
            in_contact = False
        x, y, th = nx, ny, nth
        min_clear = min(min_clear, body_clearance(world, x, y, th))
        if not trace or math.hypot(x - trace[-1][0], y - trace[-1][1]) > 0.02:
            trace.append((x, y))
        t += dt
    gd = math.hypot(world.goal[0] - x, world.goal[1] - y)
    return SimResult(gd < 0.2 + 0.05, t, path_length(trace), collisions, min_clear, gd, states, trace)


def main():
    from tb3_nav_task.worlds import WORLDS
    strat = sys.argv[1] if len(sys.argv) > 1 else 'vfh'
    names = sys.argv[2:] or list(WORLDS)
    for name in names:
        r = simulate(WORLDS[name], strat)
        print(f'{strat:5s} {name:11s} success={r.success!s:5s} t={r.time:6.1f}s path={r.path:5.2f}m '
              f'collisions={r.collisions} min_clear={r.min_clearance:.3f}m goal_err={r.final_goal_dist:.2f}m '
              f'states={r.states}')


if __name__ == '__main__':
    main()
