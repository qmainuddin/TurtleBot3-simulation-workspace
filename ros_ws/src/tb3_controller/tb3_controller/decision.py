"""Pure decision logic for a reactive obstacle-avoiding differential-drive robot.

No ROS imports here on purpose: everything can be unit-tested with plain
Python lists (see test/test_decision.py).

Pipeline per control tick
-------------------------
LaserScan ranges --> clean_range() --> sector_min() for FRONT / LEFT / RIGHT
                 --> decide() --> (linear v [m/s], angular w [rad/s], state)

Geometry (REP-103 robot frame, TurtleBot3 LDS):
    angle 0 rad       = straight ahead (+x)
    positive angle    = counter-clockwise = robot's LEFT (+y)
    positive w        = turn LEFT (counter-clockwise, seen from above)

Differential-drive meaning of the output (v, w):
    v_left  = v - w * L / 2
    v_right = v + w * L / 2        (L = wheel separation, 0.160 m on Burger)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

# TurtleBot3 Burger hardware limits (ROBOTIS e-manual specifications).
BURGER_MAX_LINEAR = 0.22   # m/s
BURGER_MAX_ANGULAR = 2.84  # rad/s


@dataclass(frozen=True)
class Params:
    max_linear: float = 0.18            # m/s   cruise speed (below 0.22 limit)
    max_angular: float = 1.2            # rad/s turning speed (below 2.84 limit)
    stop_distance: float = 0.35         # m     front obstacle closer -> rotate in place
    slow_distance: float = 0.80         # m     front obstacle closer -> slow + steer away
    wall_distance: float = 0.45         # m     side obstacle closer -> nudge away from it
    front_half_angle: float = math.radians(30)   # FRONT sector = [-30 deg, +30 deg]
    side_min_angle: float = math.radians(30)     # LEFT  sector = [+30, +100] deg
    side_max_angle: float = math.radians(100)    # RIGHT sector = [-100, -30] deg
    wall_gain: float = 1.5              # rad/s per metre of side imbalance


@dataclass(frozen=True)
class Sectors:
    front: Optional[float]
    left: Optional[float]
    right: Optional[float]


def normalize_angle(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def clean_range(r: float, range_min: float, range_max: float) -> Optional[float]:
    """Turn one raw LaserScan reading into a usable distance, or None if invalid.

    +inf  -> range_max : the beam hit nothing inside the sensor's range (free space)
    NaN   -> None      : no valid measurement
    < range_min -> None: too close to measure reliably / hardware 'no return' zero
    > range_max -> range_max
    """
    if math.isnan(r):
        return None
    if math.isinf(r):
        return range_max if r > 0 else None
    if r < range_min:
        return None
    return min(r, range_max)


def sector_min(ranges: Sequence[float], angle_min: float, angle_increment: float,
               lo: float, hi: float, range_min: float, range_max: float) -> Optional[float]:
    """Smallest valid distance among beams whose angle lies in [lo, hi] (radians)."""
    best: Optional[float] = None
    for i, raw in enumerate(ranges):
        ang = normalize_angle(angle_min + i * angle_increment)
        if lo <= ang <= hi:
            d = clean_range(raw, range_min, range_max)
            if d is not None and (best is None or d < best):
                best = d
    return best


def compute_sectors(ranges: Sequence[float], angle_min: float, angle_increment: float,
                    range_min: float, range_max: float, p: Params = Params()) -> Sectors:
    f = p.front_half_angle
    return Sectors(
        front=sector_min(ranges, angle_min, angle_increment, -f, f, range_min, range_max),
        left=sector_min(ranges, angle_min, angle_increment,
                        p.side_min_angle, p.side_max_angle, range_min, range_max),
        right=sector_min(ranges, angle_min, angle_increment,
                         -p.side_max_angle, -p.side_min_angle, range_min, range_max),
    )


def _clamp(x: float, lim: float) -> float:
    return max(-lim, min(lim, x))


def decide(s: Sectors, p: Params = Params(),
           last_turn_dir: int = 0) -> Tuple[float, float, str, int]:
    """Map sector distances to a velocity command.

    Returns (v, w, state, turn_dir). `turn_dir` (+1 left / -1 right / 0) is fed back
    on the next call so that a rotation in place keeps its direction (hysteresis),
    preventing left/right dithering in front of a symmetric obstacle.
    """
    if s.front is None:
        return 0.0, 0.0, 'NO_DATA', 0

    left = s.left if s.left is not None else 0.0     # unknown side = treat as blocked
    right = s.right if s.right is not None else 0.0
    open_dir = 1 if left >= right else -1

    # 1) Obstacle dead ahead: stop translating, rotate toward the more open side.
    if s.front < p.stop_distance:
        d = last_turn_dir if last_turn_dir != 0 else open_dir
        return 0.0, d * p.max_angular, 'TURNING', d

    # 2) Obstacle approaching: slow down linearly and steer toward the open side.
    if s.front < p.slow_distance:
        k = (s.front - p.stop_distance) / (p.slow_distance - p.stop_distance)  # 0..1
        v = p.max_linear * k
        w = open_dir * p.max_angular * (1.0 - k)
        return v, w, 'AVOIDING', 0

    # 3) Clear ahead: cruise; if a wall is close on either side, nudge away from it.
    w = 0.0
    if min(left, right) < p.wall_distance:
        w = _clamp(p.wall_gain * (left - right), 0.5 * p.max_angular)
    return p.max_linear, w, 'CRUISING', 0
