# Assignment Task 02 — requirements and where each one is met

**Task:** Mobile Robot Simulation Using ROS 2 and Gazebo (COSC471, 2026 S2)
**Chosen robotics task:** goal-directed sensor-based navigation with obstacle avoidance. A TurtleBot3 Burger must drive from a start pad to a goal pad in unknown indoor layouts, using only its 2D LiDAR, wheel odometry and IMU (no map, no GPS).

> **Before submitting:** the course brief and marking rubric were not readable in the course folder: `COSC471-26S2-Details.pdf` is 0 bytes, probably an iCloud placeholder. Check the brief's **GenAI / AI-use rules**, the required report format and the deadline yourself. Code and documents here were produced with AI assistance (Claude). If AI use must be acknowledged or limited, follow the brief; a draft acknowledgement is in `explanation.md` §15.

| # | Requirement (from the task text) | How it is met | Where |
|---|---|---|---|
| 1 | Study the simulation platform: robot models, sensors, actuators, environments, features | Survey of Gazebo Harmonic sensors/systems, ROS–Gazebo bridging, TurtleBot3 model family; VM-specific rendering investigation (ogre1/ogre2, virgl vs software GL) with measured results | `explanation.md` §2, §11 |
| 2 | Select a robot model and justify it | TurtleBot3 **Burger** vs Waffle / Waffle Pi and other options, justified by task needs (360° LiDAR, IMU, differential drive, small footprint, official Jazzy support) | `explanation.md` §3 |
| 3 | Define a robotics task | Start→goal navigation with obstacle avoidance; success, collision and timeout rules fixed before experiments | `explanation.md` §4 |
| 4 | Develop a suitable Gazebo environment | Three generated SDF worlds (walls, pillars, blocks, U-shaped trap, start/goal markers): `open_field`, `cluttered`, `u_trap` | `tb3_nav_task/worlds.py`, `worlds/*.sdf`, `explanation.md` §5 |
| 5 | ROS 2 integration: nodes, publishers, subscribers, control algorithms | `navigator` node (subscribes `/scan` `/odom` `/imu`, publishes `/cmd_vel` `/nav/status`), two control algorithms (VFH-style gap follower, Bug2), gyro-fused dead reckoning; `trial_monitor` evaluation node; one launch file for the whole system | `tb3_nav_task/*.py`, `launch/trial.launch.py`, `explanation.md` §6–8 |
| 6 | Systematic simulations; investigate parameters; multiple experiments | 75 scripted trials (+12-trial pilot). **A:** strategy × world × 4 start poses. **B:** speed sweep. **C:** safety-margin sweep. **D:** localization method (wheel-only vs gyro-fused). **E:** command shaping on/off | `scripts/run_experiments.py`, `experiments.md` |
| 7 | Define performance measures | Success rate, completion time, path length and path/straight ratio, collision count (ground-truth geometry rule), minimum body clearance, final goal error, localization error, per-cycle compute time, real-time factor, plus LiDAR accuracy vs ray-cast ground truth | `trial_monitor.py`, `probe_sensors.py`, `explanation.md` §9 |
| 8 | Final demonstration: ROS 2 + robot model + sensors + Gazebo environment + control algorithm + task execution | One command starts Gazebo (GUI), spawns the Burger, bridges sensors, runs the controller and the evaluator | `README.md` → *Demo* |
