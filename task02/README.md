# COSC471 Task 02 — TurtleBot3 goal navigation in Gazebo (ROS 2 Jazzy)

A simulated TurtleBot3 Burger drives from a start pad to a goal pad through custom obstacle worlds, using its 2D LiDAR, wheel odometry and IMU. Two controllers (VFH-style gap follower, Bug2) are compared in 75 scripted trials, scored from Gazebo ground truth.

| Item | Value |
|---|---|
| OS | Ubuntu 24.04.5 LTS, ARM64 (UTM VM on Apple Silicon) |
| ROS | ROS 2 **Jazzy** (`ros-jazzy-desktop`) |
| Simulator | Gazebo **Harmonic** via `ros-jazzy-ros-gz` |
| Robot | TurtleBot3 **Burger** (`ros-jazzy-turtlebot3-gazebo`) |
| Package | `tb3_nav_task` (ament_python) |
| Editor | VS Code (run everything from its integrated terminal) |

## Where things are

```text
vm-share/                                   (Mac folder shared into the VM as ~/vm-share)
├── ros_ws/src/tb3_nav_task/                ROS 2 package (code)
│   ├── tb3_nav_task/
│   │   ├── worlds.py        world definitions + SDF generator (single source of geometry)
│   │   ├── geometry.py      shapes, ray casting, footprint clearance
│   │   ├── planners.py      VFH-lite gap follower and Bug2 (pure Python)
│   │   ├── localization.py  wheel odometry vs gyro-fused dead reckoning
│   │   ├── navigator.py     ROS 2 node: /scan /odom /imu -> /cmd_vel
│   │   ├── trial_monitor.py ROS 2 node: ground-truth evaluation -> results.csv
│   │   └── sim2d.py         fast offline 2D simulator for tests
│   ├── launch/trial.launch.py   whole system (Gazebo + robot + bridges + nodes)
│   ├── worlds/*.sdf             generated Gazebo worlds
│   ├── scripts/run_experiments.py   batch experiments (plans A-D)
│   ├── scripts/analyze_results.py   tables + figures
│   ├── scripts/probe_sensors.py     LiDAR accuracy vs ground truth
│   └── test/test_task02.py          19 unit/integration tests (pytest)
├── task02/results/                results.csv, traces/, logs/, analysis/ (figures, summary.md)
├── task02/results_pilot_v1/       first 12 trials with the v1 controller (see experiments.md §5)
└── scripts/                       VM set-up scripts (setup-all.sh, gltest*.sh, probe.sh)
```

The documents (`README.md`, `requirements.md`, `explanation.md`, `experiments.md`) and a copy of the results are in `discuss/assignments/task02/`.

## Build (once, or after changing setup.py / package.xml / adding files)

```bash
cd ~/ros_ws
colcon build --symlink-install --packages-select tb3_nav_task
source ~/ros_ws/install/local_setup.bash     # already in ~/.bashrc
```

Python edits in `~/vm-share/ros_ws/src/tb3_nav_task/` take effect without rebuilding (symlink install).

## Demo (final demonstration, Gazebo GUI)

```bash
# The U-trap: Bug2 escapes the cul-de-sac and reaches the goal
ros2 launch tb3_nav_task trial.launch.py world:=u_trap strategy:=bug2 gui:=true

# Same world with the reactive controller: watch it get trapped
ros2 launch tb3_nav_task trial.launch.py world:=u_trap strategy:=vfh gui:=true

# Other worlds / settings
ros2 launch tb3_nav_task trial.launch.py world:=cluttered strategy:=vfh gui:=true nav_params:='{"v_max": 0.22}'
ros2 launch tb3_nav_task trial.launch.py world:=open_field strategy:=bug2 gui:=true nav_params:='{"localization": "odom"}'
```

The GUI opens after about 10 s (software rendering). The navigator logs every state change (`GO_GOAL`, `AVOID`, `FOLLOW`, `REACHED`), and `trial_monitor` prints a `RESULT` line with the metrics when the goal is reached. Stop with **Ctrl+C**.

Useful while the demo runs (second terminal):

```bash
ros2 node list
ros2 topic list
ros2 topic hz /scan                      # ~5 Hz
ros2 topic echo /nav/status              # state, goal distance, decision time
ros2 topic info /cmd_vel --verbose       # TwistStamped, 1 publisher (navigator)
```

## Experiments

```bash
cd ~/vm-share
python3 ros_ws/src/tb3_nav_task/scripts/run_experiments.py --plan A B C D E   # ~2.5 h, resumable
python3 ros_ws/src/tb3_nav_task/scripts/analyze_results.py                 # -> task02/results/analysis/
```

Plans: **A** strategy × world × 4 starts; **B** speed sweep; **C** safety-margin sweep; **D** wheel-only odometry; **E** command shaping off. Each trial is headless and writes one row to `results.csv` plus a trajectory in `traces/`. An interrupted run resumes where it stopped.

## Tests

```bash
cd ~/vm-share/ros_ws/src/tb3_nav_task && python3 -m pytest -q test      # 19 passed
python3 -m tb3_nav_task.sim2d bug2                                        # offline 2D preview
bash ~/vm-share/scripts/probe.sh open_field                               # LiDAR accuracy check
```

## VM-specific settings (important)

- **Rendering:** the UTM virgl driver provides only OpenGL 2.1. Gazebo's LiDAR needs the ogre2 renderer (OpenGL ≥ 3.3), so the launch file defaults to `gl_mode:=software` (Mesa software rasteriser). On a normal Linux PC use `gl_mode:=native`.
- **VS Code:** hardware acceleration is disabled (`~/.vscode/argv.json`) because the Electron GPU path froze on virgl.
- **UTM network:** *Emulated VLAN* (Shared Network NAT was blocked by VPN software on the Mac).

## Known limitations

These are covered in `explanation.md` §13: local controllers only (no map), VFH local minima, a circular footprint approximation for collisions, gyro drift over long runs, and 3–4 trials per condition.
