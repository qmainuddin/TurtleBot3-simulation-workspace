"""Tests for Task 02 pure-Python components (no ROS/Gazebo needed).

    cd ~/vm-share/ros_ws/src/tb3_nav_task && python3 -m pytest -q test
"""
import math
import xml.dom.minidom

import pytest

from tb3_nav_task.geometry import Box, Cylinder, body_clearance, wrap
from tb3_nav_task.planners import Bug2, GapFollower, ScanPoints, goal_polar
from tb3_nav_task.sim2d import simulate
from tb3_nav_task.worlds import WORLDS, to_sdf


def test_wrap():
    assert wrap(3 * math.pi) == pytest.approx(-math.pi)
    assert wrap(math.radians(370)) == pytest.approx(math.radians(10))


def test_box_distance_and_ray():
    b = Box(0, 0, 2, 1)
    assert b.distance(2, 0) == pytest.approx(1.0)
    assert b.distance(0, 0) == pytest.approx(-0.5)
    assert b.ray(-3, 0, 1, 0) == pytest.approx(2.0)
    assert b.ray(-3, 2, 1, 0) is None
    rb = Box(0, 0, 2, 1, math.pi / 2)            # rotated 90 deg: now 1 wide in x
    assert rb.ray(-3, 0, 1, 0) == pytest.approx(2.5)


def test_cylinder_ray():
    c = Cylinder(2, 0, 0.5)
    assert c.ray(0, 0, 1, 0) == pytest.approx(1.5)
    assert c.ray(0, 1, 1, 0) is None


def test_scan_points_skip_inf_nan_and_offset():
    s = ScanPoints.from_scan([1.0, math.inf, math.nan, 0.05], 0.0, math.pi / 2, 0.12, 3.5)
    assert len(s.d) == 1                         # only the valid 1.0 m return is kept
    assert s.px[0] == pytest.approx(1.0 - 0.032)  # LiDAR mounted 0.032 m behind base


def test_goal_polar():
    d, b = goal_polar((0, 0, math.pi / 2), (1, 0))
    assert d == pytest.approx(1.0) and b == pytest.approx(-math.pi / 2)


def test_vfh_blocks_heading_toward_close_obstacle():
    g = GapFollower()
    scan = ScanPoints.from_scan([0.5] * 11, math.radians(-10), math.radians(2), 0.12, 3.5)
    cmd = g.step((0, 0, 0), (5, 0), scan)
    assert cmd.state == 'AVOID' and abs(cmd.info['heading_cmd']) > math.radians(20)


def test_bug2_switches_to_follow_when_blocked():
    b = Bug2()
    scan = ScanPoints.from_scan([0.25] * 21, math.radians(-20), math.radians(2), 0.12, 3.5)
    cmd = b.step((0, 0, 0), (5, 0), scan)
    assert b.state == 'FOLLOW' and cmd.v == 0.0 and cmd.w > 0  # blocked -> turn left in place


def test_reached_stops():
    for p in (GapFollower(), Bug2()):
        assert p.step((1.0, 1.0, 0.0), (1.1, 1.0), ScanPoints()).state == 'REACHED'


@pytest.mark.parametrize('name', list(WORLDS))
def test_world_sdf_well_formed_and_start_free(name):
    w = WORLDS[name]
    xml.dom.minidom.parseString(to_sdf(w))
    assert body_clearance(w, *w.start) > 0.2


@pytest.mark.parametrize('world', ['open_field', 'cluttered'])
@pytest.mark.parametrize('strategy', ['vfh', 'bug2'])
def test_offline_reaches_goal_without_collision(world, strategy):
    r = simulate(WORLDS[world], strategy, timeout=200)
    assert r.success and r.collisions == 0


def test_offline_bug2_escapes_u_trap_where_vfh_is_trapped():
    assert simulate(WORLDS['u_trap'], 'bug2', timeout=200).success
    assert not simulate(WORLDS['u_trap'], 'vfh', timeout=120).success


def test_gyro_odometry_ignores_wheel_heading_and_integrates_speed():
    from tb3_nav_task.localization import GyroOdometry, WheelOdometry, compose
    g = GyroOdometry((1.0, 2.0, 0.0))
    for k in range(101):                      # 1 s of 0.5 rad/s gyro at 100 Hz
        g.on_gyro(k * 0.01, 0.5)
    assert g.pose is None                     # no odometry yet
    g.on_odom(1.0, (9.0, 9.0, 3.0), 0.2)      # wheel pose is ignored, only speed is used
    g.on_odom(1.1, (9.0, 9.0, 3.0), 0.2)      # 0.1 s at 0.2 m/s -> 0.02 m along the gyro heading
    x, y, th = g.pose
    assert th == pytest.approx(0.5, abs=1e-6)
    assert x == pytest.approx(1.0 + 0.02 * math.cos(0.5)) and y == pytest.approx(2.0 + 0.02 * math.sin(0.5))
    g.on_odom(3.0, (9.0, 9.0, 3.0), 0.2)      # a >0.5 s gap (message dropout) is not integrated
    assert g.pose[0] == pytest.approx(x)
    w = WheelOdometry((1.0, 0.0, math.pi / 2))
    w.on_odom(0.0, (1.0, 0.0, 0.0), 0.0)
    assert w.pose[0] == pytest.approx(1.0) and w.pose[1] == pytest.approx(1.0)
    assert compose((0, 0, 0), (1, 2, 0.3)) == pytest.approx((1, 2, 0.3))


def test_command_shaper_limits_rates_and_spin():
    from tb3_nav_task.planners import CommandShaper, ShaperParams
    sh = CommandShaper(0.1, ShaperParams(a_lin=0.5, d_lin=1.0, a_ang=2.0, w_spin=0.6))
    v, w = sh(0.18, 1.5)                    # from rest: v +0.05, w +0.2 per 0.1 s
    assert v == pytest.approx(0.05) and w == pytest.approx(0.2)
    for _ in range(20):
        v, w = sh(0.0, 1.5)                 # spin in place: capped at w_spin
    assert v == pytest.approx(0.0) and w == pytest.approx(0.6)
    assert CommandShaper(0.1, enabled=False)(0.18, 1.5) == (0.18, 1.5)


def test_bug2_abandons_after_full_circle_without_wall():
    import math as m
    from tb3_nav_task.planners import Bug2
    b = Bug2()
    b.state, b.start, b.follow_travel, b.hit_goal_dist = 'FOLLOW', (0.0, 0.0), 0.0, 10.0
    yaw, states = 0.0, []
    for _ in range(200):                    # empty scan: wall lost -> arc; yaw advances each step
        yaw -= 0.1
        cmd = b.step((0.0, 5.0, m.remainder(yaw, 2 * m.pi)), (10.0, 0.0), ScanPoints())
        states.append(cmd.state)
        if cmd.state == 'ABANDON':
            break
    assert 'ABANDON' in states and b.state == 'GO_GOAL' and 60 <= len(states) <= 70
