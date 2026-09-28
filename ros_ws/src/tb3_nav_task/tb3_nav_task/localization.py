"""Pose estimators for the navigator (pure Python, no ROS).

Why this exists
---------------
The Gazebo DiffDrive plugin integrates *wheel* rotation. When the Burger turns, its wheels slip on
the floor, so the odometry believes it rotated more (or less) than it really did. A heading error
of a few degrees then grows into a large position error because every later metre is travelled
in the wrong direction (dead-reckoning drift). Measured in the smoke trial: 0.89 m error after 9 m.

Estimators
----------
WheelOdometry   pose = start (+) odom pose from the DiffDrive plugin (baseline, wheel-only)
GyroOdometry    heading from integrating the IMU gyro yaw rate (does not see wheel slip),
                distance from the wheel odometry forward speed:
                    theta_k+1 = theta_k + wz * dt                         (IMU, ~200 Hz)
                    x_k+1 = x_k + v * dt * cos(theta),  y likewise         (odom twist, 30 Hz)
                A gyro still drifts slowly (bias, noise) but is immune to rotational wheel slip.
Both use only on-board sensors; ground truth is never used for control.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

from tb3_nav_task.geometry import wrap

Pose = Tuple[float, float, float]


def compose(a: Pose, b: Pose) -> Pose:
    """SE(2) composition a (+) b: express pose b (given in frame a) in a's parent frame."""
    ax, ay, at = a
    bx, by, bt = b
    return (ax + math.cos(at) * bx - math.sin(at) * by,
            ay + math.sin(at) * bx + math.cos(at) * by,
            wrap(at + bt))


class WheelOdometry:
    name = 'odom'

    def __init__(self, start: Pose):
        self.start = start
        self.pose: Optional[Pose] = None

    def on_odom(self, t: float, odom_pose: Pose, v: float) -> None:
        self.pose = compose(self.start, odom_pose)

    def on_gyro(self, t: float, wz: float) -> None:
        pass


class GyroOdometry:
    name = 'odom_imu'

    def __init__(self, start: Pose):
        self.x, self.y, self.theta = start
        self.t_odom: Optional[float] = None
        self.t_gyro: Optional[float] = None
        self.have_odom = False

    @property
    def pose(self) -> Optional[Pose]:
        return (self.x, self.y, self.theta) if self.have_odom else None

    def on_gyro(self, t: float, wz: float) -> None:
        if self.t_gyro is not None:
            dt = t - self.t_gyro
            if 0.0 < dt < 0.5:
                self.theta = wrap(self.theta + wz * dt)
        self.t_gyro = t

    def on_odom(self, t: float, odom_pose: Pose, v: float) -> None:
        if self.t_odom is not None:
            dt = t - self.t_odom
            if 0.0 < dt < 0.5:
                self.x += v * dt * math.cos(self.theta)
                self.y += v * dt * math.sin(self.theta)
        self.t_odom = t
        self.have_odom = True


def make_estimator(name: str, start: Pose):
    if name == 'odom':
        return WheelOdometry(start)
    if name == 'odom_imu':
        return GyroOdometry(start)
    raise ValueError(f'unknown localization {name!r} (odom | odom_imu)')
