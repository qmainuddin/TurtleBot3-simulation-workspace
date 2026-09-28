"""Evaluation environments for COSC471 Task 02 and their Gazebo (SDF 1.8) generator.

Each WorldSpec is the single source of truth used to
  * write the Gazebo world file   (python3 -m tb3_nav_task.worlds <out_dir>)
  * measure ground-truth collisions/clearance in trial_monitor
  * run the fast offline 2D tests  (test/)

Arena: 7.0 m x 5.0 m walled room, x in [-3.5, 3.5], y in [-2.5, 2.5].
Robot: TurtleBot3 Burger, ~0.18 m wide, so every gap is >= 0.5 m.
"""
from __future__ import annotations

import math
import os
import sys
from typing import Dict

from tb3_nav_task.geometry import Box, Cylinder, WorldSpec

WORLDS: Dict[str, WorldSpec] = {}


def _register(w: WorldSpec) -> WorldSpec:
    WORLDS[w.name] = w
    return w


# 1) Sparse obstacles; the straight start->goal line is blocked three times.
_register(WorldSpec(
    name='open_field',
    description='Sparse field: 3 pillars + 3 blocks, straight line to goal blocked 3 times',
    size_x=7.0, size_y=5.0,
    start=(-2.8, -1.8, 0.0), goal=(2.8, 1.8),
    obstacles=[
        Cylinder(-1.4, -1.0, 0.25, 'pillar_1'),
        Cylinder(0.0, 0.2, 0.30, 'pillar_2'),
        Cylinder(1.3, 1.0, 0.25, 'pillar_3'),
        Box(-0.3, -1.6, 0.8, 0.4, 0.0, 'block_1'),
        Box(1.8, -0.6, 0.4, 1.0, 0.0, 'block_2'),
        Box(-1.6, 1.2, 1.0, 0.4, 0.3, 'block_3'),
    ]))

# 2) Dense clutter; many narrow (0.5-0.8 m) gaps.
_register(WorldSpec(
    name='cluttered',
    description='Dense clutter: 12 pillars + 2 blocks, gaps 0.5-0.8 m',
    size_x=7.0, size_y=5.0,
    start=(-2.9, -2.0, 0.0), goal=(2.9, 2.0),
    obstacles=[
        Cylinder(-2.0, -1.0, 0.2, 'c01'), Cylinder(-1.2, -1.8, 0.2, 'c02'),
        Cylinder(-1.0, -0.2, 0.2, 'c03'), Cylinder(-2.2, 0.8, 0.2, 'c04'),
        Cylinder(0.0, -1.0, 0.2, 'c05'), Cylinder(0.2, 0.6, 0.2, 'c06'),
        Cylinder(-0.3, 1.7, 0.2, 'c07'), Cylinder(1.0, -0.2, 0.2, 'c08'),
        Cylinder(1.2, 1.4, 0.2, 'c09'), Cylinder(2.2, 0.4, 0.2, 'c10'),
        Cylinder(2.0, -1.6, 0.2, 'c11'), Cylinder(1.1, -1.9, 0.2, 'c12'),
        Box(-1.9, 2.0, 0.8, 0.3, 0.0, 'block_n'),
        Box(2.9, -0.5, 0.3, 0.8, 0.0, 'block_e'),
    ]))

# 3) U-shaped trap facing the start: a classic local minimum for purely reactive control.
_register(WorldSpec(
    name='u_trap',
    description='U-shaped cul-de-sac on the start-goal line (local-minimum test)',
    size_x=7.0, size_y=5.0,
    start=(-2.8, 0.0, 0.0), goal=(2.8, 0.0),
    obstacles=[
        Box(0.60, 0.0, 0.10, 2.10, 0.0, 'u_back'),
        Box(0.05, 1.0, 1.20, 0.10, 0.0, 'u_arm_left'),
        Box(0.05, -1.0, 1.20, 0.10, 0.0, 'u_arm_right'),
    ]))


# ------------------------------------------------------------------------ SDF generation
def _material(r, g, b):
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


def _box_model(b: Box, h: float, color) -> str:
    geom = f'<geometry><box><size>{b.sx} {b.sy} {h}</size></box></geometry>'
    return (f'<model name="{b.name}"><static>true</static>'
            f'<pose>{b.cx} {b.cy} {h / 2} 0 0 {b.yaw}</pose><link name="link">'
            f'<collision name="c">{geom}</collision>'
            f'<visual name="v">{geom}{_material(*color)}</visual></link></model>')


def _cyl_model(c: Cylinder, h: float, color) -> str:
    geom = f'<geometry><cylinder><radius>{c.r}</radius><length>{h}</length></cylinder></geometry>'
    return (f'<model name="{c.name}"><static>true</static>'
            f'<pose>{c.cx} {c.cy} {h / 2} 0 0 0</pose><link name="link">'
            f'<collision name="c">{geom}</collision>'
            f'<visual name="v">{geom}{_material(*color)}</visual></link></model>')


def _marker(name: str, x: float, y: float, r: float, color) -> str:
    """Visual-only disc on the floor (no collision, invisible to the LiDAR)."""
    geom = f'<geometry><cylinder><radius>{r}</radius><length>0.004</length></cylinder></geometry>'
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} 0.002 0 0 0</pose>'
            f'<link name="link"><visual name="v">{geom}{_material(*color)}'
            f'<cast_shadows>false</cast_shadows></visual></link></model>')


# Rendering engine for sensors. ogre2 (Gazebo default) is required: in this VM the ogre1 GPU LiDAR
# returns range_min on every beam. ogre2 needs OpenGL >= 3.3, which the UTM virgl driver only
# provides with a Mesa version override (see launch/trial.launch.py gl_mode and scripts/gltest2.sh).
RENDER_ENGINE = 'ogre2'


def to_sdf(w: WorldSpec, step: float = 0.002, render_engine: str = RENDER_ENGINE) -> str:
    parts = []
    for b in w.walls():
        parts.append(_box_model(b, w.wall_height, (0.6, 0.6, 0.6)))
    for o in w.obstacles:
        if isinstance(o, Box):
            parts.append(_box_model(o, w.wall_height, (0.8, 0.45, 0.2)))
        else:
            parts.append(_cyl_model(o, w.wall_height, (0.2, 0.4, 0.8)))
    sx, sy, _ = w.start
    parts.append(_marker('start_marker', sx, sy, 0.20, (0.1, 0.3, 0.9)))
    parts.append(_marker('goal_marker', w.goal[0], w.goal[1], 0.20, (0.1, 0.8, 0.2)))
    body = '\n    '.join(parts)
    return f'''<?xml version="1.0"?>
<!-- AUTO-GENERATED by tb3_nav_task/worlds.py ({w.name}): {w.description}. Edit the Python spec, not this file. -->
<sdf version="1.8">
  <world name="{w.name}">
    <physics name="default_physics" type="dart">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>{int(round(1.0 / step))}</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>{render_engine}</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.8 0.9 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry>{_material(0.85, 0.85, 0.82)}</visual>
      </link>
    </model>
    {body}
  </world>
</sdf>
'''


def write_all(out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    for w in WORLDS.values():
        path = os.path.join(out_dir, f'{w.name}.sdf')
        with open(path, 'w') as f:
            f.write(to_sdf(w))
        print('wrote', path)


def check_worlds() -> None:
    """Sanity checks: start/goal free, obstacles leave >= 0.45 m passages near them."""
    from tb3_nav_task.geometry import FOOTPRINT_RADIUS
    for w in WORLDS.values():
        for label, (x, y) in (('start', w.start[:2]), ('goal', w.goal)):
            c = w.clearance(x, y)
            assert c > FOOTPRINT_RADIUS + 0.2, f'{w.name}: {label} too close to obstacle ({c:.2f})'
        print(f'{w.name}: ok, start->goal straight-line distance '
              f'{math.hypot(w.goal[0] - w.start[0], w.goal[1] - w.start[1]):.2f} m')


if __name__ == '__main__':
    check_worlds()
    write_all(sys.argv[1] if len(sys.argv) > 1 else 'worlds')
