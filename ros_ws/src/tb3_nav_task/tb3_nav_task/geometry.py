"""Planar geometry shared by the world generator, the evaluation monitor and the offline tests.

Pure Python (no ROS). All units SI: metres, radians. World frame = Gazebo world frame,
x forward/east, y left/north, yaw counter-clockwise from +x (REP-103).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def wrap(a: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Box:
    """Static rectangular obstacle / wall. (cx, cy) centre, (sx, sy) side lengths, yaw rotation."""
    cx: float
    cy: float
    sx: float
    sy: float
    yaw: float = 0.0
    name: str = 'box'

    def to_local(self, x: float, y: float) -> Point:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x - self.cx, y - self.cy
        return c * dx + s * dy, -s * dx + c * dy

    def distance(self, x: float, y: float) -> float:
        """Signed distance from a point to the rectangle boundary (negative = inside)."""
        lx, ly = self.to_local(x, y)
        qx, qy = abs(lx) - self.sx / 2.0, abs(ly) - self.sy / 2.0
        outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
        inside = min(max(qx, qy), 0.0)
        return outside + inside

    def ray(self, ox: float, oy: float, dx: float, dy: float) -> Optional[float]:
        """Distance along a unit ray to the first intersection (slab method), or None."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lox, loy = self.to_local(ox, oy)
        ldx, ldy = c * dx + s * dy, -s * dx + c * dy
        tmin, tmax = -math.inf, math.inf
        for o, d, h in ((lox, ldx, self.sx / 2.0), (loy, ldy, self.sy / 2.0)):
            if abs(d) < 1e-12:
                if abs(o) > h:
                    return None
            else:
                t1, t2 = (-h - o) / d, (h - o) / d
                tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
        if tmax < max(tmin, 0.0):
            return None
        return tmin if tmin > 0 else (tmax if tmax > 0 else None)


@dataclass(frozen=True)
class Cylinder:
    cx: float
    cy: float
    r: float
    name: str = 'cyl'

    def distance(self, x: float, y: float) -> float:
        return math.hypot(x - self.cx, y - self.cy) - self.r

    def ray(self, ox: float, oy: float, dx: float, dy: float) -> Optional[float]:
        fx, fy = ox - self.cx, oy - self.cy
        b = fx * dx + fy * dy
        c = fx * fx + fy * fy - self.r * self.r
        disc = b * b - c
        if disc < 0:
            return None
        sq = math.sqrt(disc)
        for t in (-b - sq, -b + sq):
            if t > 0:
                return t
        return None


@dataclass
class WorldSpec:
    """Everything that defines one evaluation environment."""
    name: str
    description: str
    size_x: float                      # interior arena length (m)
    size_y: float                      # interior arena width (m)
    start: Tuple[float, float, float]  # x, y, yaw (world frame)
    goal: Tuple[float, float]
    obstacles: List[object] = field(default_factory=list)
    wall_thickness: float = 0.10
    wall_height: float = 0.50

    def walls(self) -> List[Box]:
        hx, hy, t = self.size_x / 2.0, self.size_y / 2.0, self.wall_thickness
        return [
            Box(0.0, hy + t / 2, self.size_x + 2 * t, t, name='wall_north'),
            Box(0.0, -hy - t / 2, self.size_x + 2 * t, t, name='wall_south'),
            Box(hx + t / 2, 0.0, t, self.size_y, name='wall_east'),
            Box(-hx - t / 2, 0.0, t, self.size_y, name='wall_west'),
        ]

    def solids(self) -> List[object]:
        return self.walls() + list(self.obstacles)

    def clearance(self, x: float, y: float) -> float:
        """Distance from a point to the nearest solid surface (negative if inside)."""
        return min(o.distance(x, y) for o in self.solids())

    def raycast(self, ox: float, oy: float, angle: float, max_range: float) -> float:
        dx, dy = math.cos(angle), math.sin(angle)
        best = math.inf
        for o in self.solids():
            t = o.ray(ox, oy, dx, dy)
            if t is not None and t < best:
                best = t
        return best if best <= max_range else math.inf


# ---------------------------------------------------------------- robot footprint model
# TurtleBot3 Burger (turtlebot3_gazebo/models/turtlebot3_burger/model.sdf, Jazzy):
#   base collision box 0.14 x 0.14 m centred 0.032 m BEHIND base_footprint; wheels at x=0,
#   y=+/-0.08 (radius 0.033). A circle of radius 0.11 m centred on the box centre encloses
#   the box (circumradius 0.099 m) and the wheel treads (<= 0.110 m).
FOOTPRINT_OFFSET_X = -0.032
FOOTPRINT_RADIUS = 0.11
LIDAR_OFFSET_X = -0.032          # base_scan is 0.032 m behind base_footprint


def footprint_centre(x: float, y: float, yaw: float) -> Point:
    return x + FOOTPRINT_OFFSET_X * math.cos(yaw), y + FOOTPRINT_OFFSET_X * math.sin(yaw)


def body_clearance(world: WorldSpec, x: float, y: float, yaw: float) -> float:
    """Gap between the robot footprint circle and the nearest obstacle (<= 0 means contact)."""
    cx, cy = footprint_centre(x, y, yaw)
    return world.clearance(cx, cy) - FOOTPRINT_RADIUS


def path_length(points: Sequence[Point]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))
