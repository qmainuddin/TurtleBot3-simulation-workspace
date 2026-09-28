"""Unit tests for the pure decision logic (no ROS or simulator required).

Run:  cd ~/vm-share/ros_ws/src/tb3_controller && python3 -m pytest -q test
"""
import math

import pytest

from tb3_controller.decision import (Params, Sectors, clean_range, compute_sectors, decide,
                                     normalize_angle)

P = Params()
N = 360                    # TurtleBot3 LDS: 360 beams, 1 degree apart, starting straight ahead
INC = 2 * math.pi / N
RMIN, RMAX = 0.12, 3.5


def scan(default=math.inf, **overrides):
    """Build a 360-beam scan; overrides like deg_0=0.3 set the beam at that degree."""
    r = [default] * N
    for k, v in overrides.items():
        r[int(k.split('_')[1]) % N] = v
    return r


def test_normalize_angle():
    assert normalize_angle(0.0) == pytest.approx(0.0)
    assert normalize_angle(math.radians(350)) == pytest.approx(math.radians(-10))
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi)


def test_clean_range():
    assert clean_range(math.inf, RMIN, RMAX) == RMAX       # nothing hit -> free space
    assert clean_range(math.nan, RMIN, RMAX) is None       # invalid
    assert clean_range(0.0, RMIN, RMAX) is None            # below range_min
    assert clean_range(9.0, RMIN, RMAX) == RMAX
    assert clean_range(1.0, RMIN, RMAX) == 1.0


def test_sector_assignment_front_left_right():
    # beam at 350 deg == -10 deg is still FRONT; 60 deg is LEFT; 300 deg == -60 deg is RIGHT
    s = compute_sectors(scan(deg_350=0.9, deg_60=0.5, deg_300=0.7), 0.0, INC, RMIN, RMAX)
    assert s.front == pytest.approx(0.9)
    assert s.left == pytest.approx(0.5)
    assert s.right == pytest.approx(0.7)


def test_empty_world_cruises_straight():
    s = compute_sectors(scan(), 0.0, INC, RMIN, RMAX)
    v, w, state, _ = decide(s, P)
    assert state == 'CRUISING' and v == P.max_linear and w == 0.0


def test_close_front_obstacle_rotates_toward_open_side():
    # wall ahead and on the right -> rotate LEFT (positive w), no forward motion
    v, w, state, d = decide(Sectors(front=0.2, left=2.0, right=0.4), P)
    assert state == 'TURNING' and v == 0.0 and w > 0 and d == 1
    v, w, state, d = decide(Sectors(front=0.2, left=0.4, right=2.0), P)
    assert w < 0 and d == -1


def test_turn_direction_hysteresis():
    # once turning left, keep turning left even if the right side now looks slightly better
    _, w, _, d = decide(Sectors(front=0.2, left=1.0, right=1.1), P, last_turn_dir=1)
    assert w > 0 and d == 1


def test_slowing_is_monotonic():
    fronts = [0.40, 0.55, 0.70]
    speeds = [decide(Sectors(f, 3.0, 3.0), P)[0] for f in fronts]
    assert speeds == sorted(speeds) and 0 < speeds[0] < P.max_linear


def test_wall_following_nudge_away_from_close_side():
    _, w, state, _ = decide(Sectors(front=3.0, left=3.0, right=0.3), P)
    assert state == 'CRUISING' and 0 < w <= 0.5 * P.max_angular


def test_no_data_stops():
    assert decide(Sectors(None, None, None), P)[:3] == (0.0, 0.0, 'NO_DATA')


def test_limits_respected():
    for f in [0.1, 0.3, 0.5, 0.8, 2.0]:
        for l, r in [(0.2, 3.0), (3.0, 0.2), (1.0, 1.0)]:
            v, w, _, _ = decide(Sectors(f, l, r), P)
            assert 0.0 <= v <= P.max_linear and abs(w) <= P.max_angular
