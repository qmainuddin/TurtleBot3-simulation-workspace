# Task 02 — explanation: goal-directed LiDAR navigation of a simulated TurtleBot3

This file explains what was built, why, and how it works, in simple language first and then with the maths. Measured results are in `experiments.md`.

---

## 1. Objective

Build and evaluate a complete simulated mobile-robot system, and answer three questions with repeatable experiments:

1. **Which control strategy works better for reaching a goal among obstacles:** a purely reactive gap-follower (VFH-style) or the boundary-following Bug2 algorithm, and in which environments?
2. **How do speed and safety margin trade off** time, path length and safety?
3. **How much does localization quality matter:** wheel odometry alone vs wheel odometry fused with the IMU gyro? And how much does *how* the robot is commanded (rate-limited vs step commands) affect it?

**Pipeline:** Ubuntu 24.04 (ARM64 VM in UTM) → ROS 2 Jazzy → Gazebo Harmonic → TurtleBot3 Burger → Python ROS 2 nodes.

---

## 2. Study of the simulation platform

### 2.1 Gazebo Harmonic (gz-sim 8)

- **What it does:** Gazebo simulates rigid-body physics (DART by default), sensors and actuators in a world described by an SDF file. Behaviour is added by *systems* (plugins).
- **Systems used here:**
  - `Physics`
  - `UserCommands` (spawning)
  - `SceneBroadcaster` (poses for the GUI)
  - `Sensors` (renders camera/LiDAR data with the `ogre2` engine)
  - `Imu`
  - `DiffDrive` (turns a velocity command into wheel speeds and publishes odometry)
  - `JointStatePublisher`
  - `PosePublisher` (ground truth)
- **Sensors Gazebo offers** ([gz-sensors docs](https://gazebosim.org/docs/harmonic/sensors/)): camera, depth camera, RGBD, GPU LiDAR, IMU, magnetometer, altimeter, air pressure, NavSat (GPS), contact, force-torque, thermal, segmentation and bounding-box cameras.
- **How the LiDAR works:** it is **GPU-rendered**. The scene is rendered from the sensor and depth is read back, so a working OpenGL ≥ 3.3 renderer is required even without a GUI. This mattered a lot in the VM (§11).
- **Environments:** worlds are SDF files. Models can come from Gazebo Fuel (online) or be written inline. Here every world is generated from Python (§5), so the world file and the evaluation geometry cannot disagree.

### 2.2 ROS 2 ↔ Gazebo

- Gazebo has its own transport (gz-transport). ROS 2 nodes talk to it through **`ros_gz_bridge`**, which converts message types, e.g. `gz.msgs.LaserScan` → `sensor_msgs/LaserScan`, and `geometry_msgs/TwistStamped` → `gz.msgs.Twist` ([ros_gz](https://gazebosim.org/docs/harmonic/ros2_integration/)).
- **Jazzy's official pairing is Gazebo Harmonic**, installed through `ros-jazzy-ros-gz`.

### 2.3 TurtleBot3 simulation packages (ROBOTIS, Jazzy branch)

- **What they provide:** robot models, worlds, a bridge configuration and a keyboard teleop.
- **Burger bridge topics:**
  - `/clock`, `/odom`, `/tf`, `/imu`, `/scan` and `/joint_states` from Gazebo to ROS.
  - `/cmd_vel` from ROS to Gazebo (as `TwistStamped` in Jazzy).

---

## 3. Robot model selection

| Candidate | Sensors | Max speed (ROBOTIS spec) | Fit for this task |
|---|---|---|---|
| **TurtleBot3 Burger** (chosen) | 360° LDS LiDAR (1°, 0.12–3.5 m, 5 Hz), IMU, wheel encoders | 0.22 m/s, 2.84 rad/s | Everything the task needs, nothing extra |
| TurtleBot3 Waffle / Waffle Pi | Same LiDAR + camera | 0.26 m/s, 1.82 rad/s | Camera unused; wider body (~0.3 m) needs wider gaps; image rendering costs CPU |
| Custom SDF robot | Anything | — | More modelling work; less credible than a documented platform |
| Ackermann car / legged robot | — | — | Non-holonomic turning radius (car) or gait control (legged) are not the focus of this task |

**Why Burger:**

1. **Sensors:** the full 360° LiDAR sees obstacles beside and behind the robot, which Bug2's wall following and the escape rotation both need.
2. **Kinematics:** differential drive can turn on the spot (v = 0, ω ≠ 0). This keeps the controller simple and matches the course's differential-drive kinematics.
3. **Size:** the small footprint (≈0.14 × 0.18 m body) allows interesting cluttered worlds with 0.5 m gaps.
4. **Localization:** it has an IMU, which made the localization experiment (plan D) possible.
5. **Support:** it is officially supported on ROS 2 Jazzy + Gazebo Harmonic, so results are reproducible.

---

## 4. Task definition

| Item | Definition |
|---|---|
| Input sensors | `/scan` (LiDAR), `/odom` (wheel odometry), `/imu` (gyro) |
| Output | `/cmd_vel` (forward speed v [m/s], yaw rate ω [rad/s]) |
| Goal | Reach the goal position; the navigator stops when its **own estimate** is within 0.20 m |
| Success (evaluation) | Navigator reports `REACHED` **and** the ground-truth distance to the goal is ≤ 0.30 m **and** there were zero collisions |
| Collision | Robot footprint circle (radius 0.11 m, centred 0.032 m behind `base_footprint`) intersects obstacle geometry. A new event needs clearance > 0.02 m first (debounce) |
| Timeout | 200 s of simulated time |
| Prior knowledge | Start pose and goal coordinates only; no map |

Ground truth (Gazebo's true robot pose) is used **only** by the evaluator, never by the controller.

---

## 5. Simulation environments

All worlds are 7.0 × 5.0 m walled rooms, generated by `tb3_nav_task/worlds.py`. The same Python description also produces the collision geometry the evaluator uses.

| World | Layout | Purpose |
|---|---|---|
| `open_field` | 3 pillars + 3 blocks; the straight start→goal line is blocked three times | Basic obstacle avoidance |
| `cluttered` | 12 pillars + 2 blocks, gaps 0.5–0.8 m | Narrow-gap navigation, safety-margin trade-offs |
| `u_trap` | A U-shaped cul-de-sac on the start→goal line, opening toward the robot | Local-minimum trap for reactive control |

**Markers and graphics:**

- Start (blue) and goal (green) markers are visual-only discs, so the LiDAR cannot see them.
- No models are downloaded from the internet, so worlds load offline.
- Shadows are disabled to save rendering time.

---

## 6. ROS 2 architecture

```text
 Gazebo Harmonic (gz sim server, physics 2 ms step)
   ├─ Burger model: DiffDrive, GPU LiDAR (5 Hz), IMU, PosePublisher (ground truth)
   │
 ros_gz_bridge ── /scan /odom /imu /clock /tf ──────────────────┐
 (tb3 params)  ←─ /cmd_vel (TwistStamped) ────────────┐         │
                                                      │         ▼
 ros_gz_bridge ── /ground_truth/pose ──┐        ┌─────────────────────────────┐
                                       │        │ navigator (10 Hz timer)     │
                                       │        │  pose estimator (odom/imu)  │
                                       │        │  planner: vfh | bug2        │
                                       │        └─────────┬───────────────────┘
                                       ▼                  │ /nav/status (JSON)
                               ┌──────────────────┐       │
                               │ trial_monitor    │◄──────┘
                               │ metrics → CSV    │
                               └──────────────────┘
 robot_state_publisher: URDF → /tf_static
```

| ROS concept | Where and why |
|---|---|
| **Node** | `navigator` (control), `trial_monitor` (evaluation), bridges, `robot_state_publisher`: one job per process |
| **Topic** | Streaming data: sensors in, commands out, status to the evaluator |
| **Message types** | `LaserScan`, `Odometry`, `Imu`, `TwistStamped`, `PoseStamped`, `String` (JSON status) |
| **QoS** | `/scan` and `/imu` are subscribed with the *sensor-data* profile (best effort, small queue): old sensor data is useless, and a reliable subscriber could stall. `/cmd_vel` is reliable |
| **Parameters** | Strategy, localization and every gain/threshold are ROS parameters, so experiments change behaviour without code edits |
| **Simulated time** | `use_sim_time:=true`: timers and timestamps follow Gazebo's `/clock`, so results are in simulated seconds even if the VM runs slower than real time |
| **Launch file** | `trial.launch.py` starts the whole system. In batch mode the monitor's exit shuts the launch down (event handler) |
| **Package / workspace** | `tb3_nav_task` (ament_python) in `~/ros_ws`, built with `colcon --symlink-install` so Python edits apply without rebuilding |

---

## 7. Control algorithms

Both run at 10 Hz. Each cycle:

1. convert the scan into points in the robot frame (the LiDAR is mounted 0.032 m behind the base);
2. estimate the pose;
3. compute (v, ω).

The goal bearing β and distance d come from the estimated pose (x, y, θ) and the goal G:

```text
d = ‖G − p‖,      β = atan2(Gy − y, Gx − x) − θ   (wrapped to [−π, π))
```

### 7.1 Strategy A — VFH-style gap follower (`vfh`, reactive)

- **Idea:** list every heading the robot could drive in, mark headings blocked by nearby obstacles, then pick the free heading closest to the goal.

1. **Polar histogram:** 72 bins of 5°.
2. **Enlarge obstacles by the robot size:** a LiDAR return at distance r blocks every heading within angle γ of its bearing:

   ```text
   γ = asin( (R + s) / r ),   R = 0.11 m robot radius,  s = safety margin (default 0.12 m)
   ```

   Example: a pillar edge at r = 0.5 m blocks ±asin(0.23/0.5) = ±27.4°. Only returns closer than 1.0 m (lookahead) count.
3. **Choose the direction** minimising the cost

   ```text
   J(a) = 1.0·|a − β| + 0.25·|a| + 0.25·|a − a_prev|
   ```

   The first term means "go to the goal", the second "don't turn much", the third "don't change your mind" (reduces oscillation).
4. **Command:**

   ```text
   ω = clip(2.0·a*, ±ω_max)
   v = v_max · cos²(a*) · clip((f − 0.10)/(0.60 − 0.10), 0, 1)
   ```

   Here f is the free distance straight ahead in a corridor one robot wide. The speed falls to 0 as the heading error grows (no driving sideways into things) and as an obstacle approaches (a braking ramp).
5. **Escape:** if every heading is blocked, rotate on the spot toward the more open side.

- **Why it may fail:** it has no memory. In a U-shaped trap the goal lies straight through the back wall, the free headings all lead back into the U, and the robot can circle for ever. This is the classic *local minimum* of reactive navigation, and the `u_trap` world tests it.

### 7.2 Strategy B — Bug2 (`bug2`, Lumelsky & Stepanov, 1987)

- **Idea:** walk along the straight line from start to goal (the **m-line**). When blocked, walk around the obstacle keeping it on your right, until you are back on the m-line *closer to the goal than where you hit it*. Then leave and head for the goal again.

```text
state GO_GOAL : ω = clip(2·β), v = v_max·cos²β (0 if |β| > 0.8 rad)
                if free distance ahead < 0.30 m (and the goal is farther): HIT → FOLLOW, remember d_hit
state FOLLOW  : keep the wall on the right at d_follow = 0.30 m
                  blocked ahead      → turn left in place
                  wall lost (corner) → arc right  (ω = −v / d_follow)
                  otherwise          → ω = 3.0·(d_follow − d_side)
                LEAVE when: travelled ≥ 0.5 m along the wall AND distance to m-line < 0.10 m
                            AND d < d_hit − 0.15 m AND the corridor toward the goal is free
```

The distance from the robot p to the m-line through S and G is `|(G − S) × (S − p)| / ‖G − S‖`.

- **Why it can escape the U-trap:** following the boundary eventually leads around the U, and the "closer than d_hit" rule prevents leaving back into it. For static 2D worlds with perfect sensing, Bug2 is *complete*: it reaches the goal if a path exists.
- **Cost:** boundary following is slow and paths are longer than necessary.

### 7.3 Command shaping (both strategies)

Planner outputs pass through `CommandShaper` before `/cmd_vel`:

- forward acceleration ≤ 0.5 m/s², deceleration ≤ 1.0 m/s² (braking is never delayed beyond the plugin's own limit);
- yaw-rate change ≤ 2 rad/s² (i.e. ≤ 0.2 rad/s per 0.1 s cycle);
- |ω| ≤ 0.6 rad/s while turning on the spot.

**Why:** tyre friction can only supply a limited sideways force. A step from ω = 0 to 1.5 rad/s demands a large angular acceleration from the wheels, the tyres slip, and the pilot run measured the robot **sliding 0.53 m while commanded to turn in place**. Neither the wheel encoders nor the gyro measure that sliding, so it goes straight into localization error. Plan E measures the effect.

**Bug2 lost-wall recovery:** if the "arc right to re-find the wall" behaviour turns the robot through a full circle without finding a boundary, Bug2 abandons following and returns to GO_GOAL. This fixes an infinite orbit around empty floor seen in the pilot.

### 7.4 Safety design (physics of stopping)

Stopping distance, using the course formula:

```text
d_stop = v·t_delay + v²/(2a) + margin
```

- **Delay:** worst-case t_delay ≈ 0.2 s (5 Hz LiDAR) + 0.1 s (10 Hz control) = 0.3 s.
- **Braking:** the DiffDrive plugin limits acceleration to a = 1.0 m/s².
- **At v = 0.18 m/s:** d_stop = 0.054 + 0.016 + margin ≈ 0.07 m + margin.
- **At v = 0.22 m/s:** d_stop ≈ 0.09 m + margin.

Both are well inside the 0.10–0.60 m braking ramp. This is a design estimate (flat floor, constant deceleration); plan B tests it empirically.

**Differential drive:** a command (v, ω) becomes wheel speeds v_L = v − ω·L/2 and v_R = v + ω·L/2, with L = 0.160 m (wheel angular speed = v_wheel / 0.033 m). Turning on the spot (v = 0, ω = 1.5 rad/s) spins the wheels at ∓0.12 m/s.

---

## 8. Localization: why wheel odometry alone failed

- **Observation (smoke trial):** with wheel odometry only, the robot stopped where it *believed* the goal was, but ground truth put it **0.84 m** away. In another run the wheel-odometry error reached **3.2 m** after a 6.7 m path.
- **Why:** the DiffDrive plugin computes the pose by integrating *wheel rotation*. When the Burger turns, the tyres slip on the floor, so the wheels rotate by an amount that does not match the body's real rotation.
  - The encoder-based odometry therefore gets the **heading** wrong by a few degrees at every turn.
  - A heading error δθ makes every later metre travelled go in the wrong direction, so position error grows about as distance × sin(δθ) and never recovers. This is dead-reckoning drift.
  - The trace showed it directly: on the first straight segment, ground truth moved about 12° north of east while odometry believed about 29°.
- **Fix (sensor fusion, `localization:=odom_imu`, default):** take the **heading** from the IMU gyro, which measures real body rotation and is immune to wheel slip, and only the **forward speed** from the wheels:

  ```text
  θ ← θ + ω_gyro·Δt          (IMU messages)
  x ← x + v_wheel·Δt·cos θ   (odometry messages, 30 Hz)
  y ← y + v_wheel·Δt·sin θ
  ```

  Gaps over 0.5 s (dropped messages) are not integrated.
- **Remaining limits:** a real gyro drifts slowly (bias), and wheel speed still over-reads during slip. A full solution is an EKF (`robot_localization`) or LiDAR scan matching / SLAM.
- Plan D measures the difference between the two methods.

---

## 9. Performance measures

| Metric | Definition | Why |
|---|---|---|
| Success rate | Successful trials / attempted | Task completion |
| Completion time | Simulated seconds from the first active control cycle to REACHED (successful trials) | Efficiency |
| Path length, path ratio | Ground-truth distance travelled; ÷ straight-line distance | Path quality |
| Collisions | Footprint–geometry contact events (§4) | Safety |
| Minimum clearance | Smallest footprint-to-obstacle gap in the trial | Safety margin actually kept |
| Goal error | Ground-truth distance to the goal at the end | Accuracy |
| Localization error | ‖navigator estimate − ground truth‖ at the end and its maximum | Effect of odometry drift |
| Compute time | Wall-clock time of one decision (mean, p95, max) | Computational response |
| Real-time factor | Simulated s / wall s | How fast the VM simulated |
| LiDAR accuracy | Measured range − ray-cast range from the true pose (bias, MAE, RMSE) | Sensor accuracy |

---

## 10. Experiment design (fixed before collecting data)

| Plan | Varies | Fixed | Trials |
|---|---|---|---|
| A | Strategy {vfh, bug2} × world {open_field, cluttered, u_trap} × 4 start poses | v_max 0.18, default margins, odom_imu | 24 |
| B | v_max {0.10, 0.15, 0.22} × strategy, cluttered, 3 starts (0.18 from A) | — | 18 |
| C | vfh safety {0.05, 0.20, 0.30}; bug2 d_follow {0.22, 0.40}; cluttered, 3 starts (defaults from A) | — | 15 |
| D | Localization = wheel odometry only; open_field + cluttered × strategy × 3 starts (compare with A) | — | 12 |
| E | Command shaping off (Bug2; open_field + cluttered × 3 starts; compare with A) | — | 6 |

- **Start variants:** start 0 is the nominal pose. Starts 1–3 add a seeded offset (±0.15 m, ±0.6 rad), so repeats are different but reproducible.
- **Determinism:** each trial is a fresh Gazebo process.
- **Tuning:** the controllers were tuned only on the offline 2D simulator (`sim2d.py`) and a single smoke trial, never on the evaluation trials themselves.

---

## 11. Debugging log (the engineering problems and how they were diagnosed)

| # | Symptom | Diagnosis | Fix |
|---|---|---|---|
| 1 | Ubuntu installer failed: `Unable to locate package linux-generic-hwe-24.04` | VM had DHCP/DNS but no outbound traffic; UTM *Shared Network* NAT blocked by VPN software on the Mac | UTM network → *Emulated VLAN* |
| 2 | VS Code window frozen | Electron GPU path on the virgl driver | `disable-hardware-acceleration` in `~/.vscode/argv.json` |
| 3 | Gazebo segfault in `driCreateNewScreen3` | `glxinfo`: virgl exposes **OpenGL 2.1** only; ogre2 needs ≥ 3.3 | Tested 12 configurations (`scripts/gltest*.sh` in `vm-share/scripts/`) |
| 4 | ogre1 renderer runs, but every beam = 0.12 m (`range_min`) | Not self-occlusion (removing the LiDAR housing visual changed nothing); ogre1 GPU rays unusable here | Use ogre2 |
| 5 | ogre2 with virgl + `MESA_GL_VERSION_OVERRIDE=4.3` runs, but ranges ≈ 0.13 m | Measured against ray-cast ground truth: bias −1.14 m | Rejected |
| 6 | ogre2 with Mesa software rasteriser (`LIBGL_ALWAYS_SOFTWARE=1`, `kms_swrast`) | LiDAR vs ground truth: bias −0.4 mm, MAE 11 mm, RMSE 17 mm, 0 hit/miss mismatches (matches σ = 10 mm noise + 15 mm resolution) | **Used**; RTF ≈ 1.0 |
| 7 | Evaluator could not find the robot in `dynamic_pose/info` | The Pose_V → TFMessage bridge drops entity names | Spawn a copy of the Burger with Gazebo's `PosePublisher` → `/ground_truth/pose` |
| 8 | Robot stopped 0.84 m from the goal | Wheel-slip heading error in odometry (§8) | Gyro-fused dead reckoning |
| 9 | Pilot: Bug2 still 0.4–0.5 m off; one trial orbited empty floor for 120 s | Per-state error analysis of traces: in-place turns at up to 1.5 rad/s made the robot skid 0.53 m; lost-wall arc had no exit | `CommandShaper` + lost-wall abandon (§7.3); all plans re-run from scratch, pilot kept in `results_pilot_v1/` |

The LiDAR-accuracy check in rows 5–6 is also the answer to "sensor accuracy" as a performance measure. Run `scripts/probe_sensors.py` to repeat it.

---

## 12. Alternatives and why they were not chosen

| Method | Strength | Weakness | Why not (here) |
|---|---|---|---|
| **Nav2** (costmaps + global planner + DWB/MPPI local controller) | Industry standard, map-based, handles traps | Needs a map (SLAM first) and many parameters; heavy in a VM | Goal was to *implement* and *compare* controllers, not configure a stack; good next step |
| Artificial potential fields | Smooth, simple | Local minima, oscillation in corridors | Same failure class as VFH with less explicit safety |
| Full VFH / VFH+ | Certainty grid, robust to noise | More state and parameters | VFH-lite keeps the key idea (enlarged polar histogram) and is easier to explain |
| Dynamic Window Approach | Respects acceleration limits explicitly | More computation; still local | Would be a strong third strategy |
| Tangent Bug | Uses range data to shortcut Bug2 | More complex | Bug2 is enough to show the memory-vs-reactive trade-off |
| EKF (`robot_localization`) | Principled fusion with covariances | Configuration effort | The two-line gyro fusion captures the main effect; EKF is the next step |

---

## 13. Limitations

- Everything is simulated; Gazebo's tyre-slip and contact models only approximate reality.
- The `cluttered` world's start–goal diagonal turned out to pass through a free corridor, so the nominal start needs no avoidance; only offset starts exercise the clutter. A better design would block the diagonal.
- Software rendering limits scene complexity. The real-time factor stayed ≈ 1.0 for these worlds but would fall with more sensors or robots.
- Worlds are static; there are no moving people or objects.
- Controllers are local: no map, no global planning. VFH can be trapped; Bug2 paths are long.
- The collision rule uses a circular footprint approximation (conservative near corners).
- The number of trials per condition (3–4) shows trends but is too small for strong statistical claims.
- Localization still drifts slowly (gyro integration); long missions would need absolute correction (landmarks, scan matching, SLAM).

---

## 14. Key terms

| Term | Meaning |
|---|---|
| **Odometry / dead reckoning** | Estimating pose by adding up small measured motions; errors accumulate |
| **Wheel slip** | Tyre moves relative to the floor, so encoder rotation ≠ body motion |
| **IMU gyro** | Measures angular velocity of the body directly |
| **Local minimum** | A place where a reactive rule has no locally better move toward the goal |
| **m-line** | Bug2's straight start→goal line |
| **QoS** | ROS 2 delivery policy (reliable vs best-effort, history depth) |
| **RTF** | Real-time factor = simulated time / wall time |
| **SDF** | Simulation Description Format (Gazebo world/model files) |

---

## 15. AI-use acknowledgement (draft — check the brief's GenAI rules first)

> The simulation code (world generator, ROS 2 nodes, experiment scripts), the VM set-up scripts and the first draft of the documentation were produced with the assistance of an AI system (Claude, Anthropic), under my direction. I ran the experiments in my own VM and checked the results. All numbers reported come from the logged trials in `results/`.

Adapt or remove this to match the course rules.

---

## 16. What remains / study next

- If the brief requires a written report, write it in your own words from `experiments.md` and the figures.
- Optional improvements: block the `cluttered` diagonal and re-run; more starts per condition; an EKF (`robot_localization`) as a third localization option.
- Record a screen video of the demo (§ README) if the demonstration is not live.
- **Study next:** EKF localization (`robot_localization`), SLAM Toolbox mapping → Nav2 goal navigation, DWA/MPPI local planners, and comparing them with these results.

## References

- Lumelsky, V. J., & Stepanov, A. A. (1987). Path-planning strategies for a point mobile automaton moving amidst unknown obstacles of arbitrary shape. *Algorithmica*, 2, 403–430.
- Borenstein, J., & Koren, Y. (1991). The vector field histogram — fast obstacle avoidance for mobile robots. *IEEE Transactions on Robotics and Automation*, 7(3), 278–288.
- Siegwart, R., Nourbakhsh, I., & Scaramuzza, D. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press (odometry error, differential-drive kinematics).
- Gazebo Harmonic documentation: https://gazebosim.org/docs/harmonic/ (sensors, ROS 2 integration, troubleshooting).
- ROS 2 Jazzy documentation: https://docs.ros.org/en/jazzy/
- ROBOTIS TurtleBot3 e-Manual (simulation, specifications): https://emanual.robotis.com/docs/en/platform/turtlebot3/
- ROBOTIS `turtlebot3_simulations`, branch `jazzy` (Burger model, bridge configuration).
