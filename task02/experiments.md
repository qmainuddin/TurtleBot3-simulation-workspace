# Task 02 — experiments and results

**Where the data comes from:**

- **Main study:** 75 Gazebo trials (plans A–E), run 28 Sep 2026 from the VS Code terminal in the Ubuntu VM. The runner is `scripts/run_experiments.py`.
- **Raw data:** `results/results.csv` (one row per trial) and `results/traces/*.csv` (ground-truth trajectory, odometry and estimate at 10 Hz).
- **Tables and figures:** `results/analysis/summary.md` and the `results/analysis/fig_*.png` files, produced by `scripts/analyze_results.py`.
- **Pilot:** a first run (v1 controller, 12 trials) is kept in `results_pilot_v1/`. It revealed the skid problem described in §5.
- **Simulation speed:** real-time factor was 0.98–1.00 in every trial, so simulated and wall-clock times match.
- **Collisions:** zero in all 75 trials.

Definitions of every metric: `explanation.md` §9. Success = goal reported **and** ground-truth goal error ≤ 0.30 m **and** no collision.

---

## 0. Sensor accuracy (LiDAR vs ray-cast ground truth)

Robot stationary at the open_field start; 10 scans × 360 beams (`scripts/probe_sensors.py`).

| Rendering configuration | Compared beams | Bias | MAE | RMSE | Max error | Hit/miss mismatches |
|---|---|---|---|---|---|---|
| ogre2 + Mesa software (used) | 2,950 | −0.4 mm | 11.1 mm | 16.8 mm | 103 mm | 0 |
| ogre2 + virgl + GL-version override | 1,660 | −1,136 mm | 1,136 mm | 1,410 mm | 3,261 mm | 1,760 |
| ogre1 (virgl or software) | – | every beam = 0.12 m (range_min) | | | | |

- **Reading:** with the chosen configuration the LiDAR is unbiased. Its error matches the model's specified noise (σ = 10 mm) plus 15 mm range resolution.
- **Why it matters:** the other two configurations *run without errors* but produce wrong data. Without this check, every navigation result would have been meaningless.

---

## 1. Plan A — strategy comparison (default parameters, 4 start poses each)

| World | Strategy | Success | Time [s] | Path [m] | Path / straight | Min clearance [m] | Goal error [m] | Localization error [m] |
|---|---|---|---|---|---|---|---|---|
| open_field | vfh | **4/4** | 38.3 ± 0.6 | 6.76 ± 0.08 | 1.01 | 0.131 | 0.201 | 0.023 |
| open_field | bug2 | **4/4** | 97.2 ± 1.5 | 9.68 ± 0.13 | 1.44 | 0.087 | 0.222 | 0.147 |
| cluttered | vfh | **4/4** | 38.4 ± 0.5 | 6.84 ± 0.04 | 0.97 | 0.142 | 0.211 | 0.023 |
| cluttered | bug2 | **4/4** | 103.2 ± 75.6 | 11.28 ± 5.27 | 1.60 | 0.065 | 0.209 | 0.096 |
| u_trap | vfh | **0/4** (all timeouts) | – | – | – | 0.396 | 2.97 | 0.487 |
| u_trap | bug2 | **4/4** | 85.7 ± 3.0 | 10.79 ± 0.08 | 1.93 | 0.097 | 0.197 | 0.047 |

Figures: `fig_A_trajectories.png`, `fig_A_metrics.png`.

**What the results show:**

- **Where VFH wins:** in open and cluttered rooms it is 2.5× faster than Bug2, with paths almost equal to the straight-line distance (ratio ≈ 1.0) and about twice the clearance (0.13–0.14 m vs 0.07–0.09 m). Its enlarged polar histogram keeps it centred in gaps, while Bug2 deliberately hugs obstacles at d_follow.
  - Path ratio below 1.0 is possible because paths are measured to where the robot stops, 0.2 m short of the goal centre.
- **Where VFH fails:** in the U-trap it failed every time. The trajectories show it driving into the cul-de-sac and oscillating at the back wall for the remaining ~190 s. Every free heading points back out of the U, while the goal-attraction term keeps pulling it in. This is the reactive local minimum predicted in `explanation.md` §7.1.
- **Bug2 in the U-trap:** it escaped 4/4 by following the inside of the U, around the outside and back to the m-line. Its path is 1.9× the straight line: slower, but complete. **Memory** (hit point, m-line) is what reactive control lacks.
- **Bug2's variance in `cluttered`:** 38–186 s. With the nominal start, the start–goal line happens to pass through a free diagonal corridor, and Bug2 never has to follow a boundary. Offset starts hit pillars and trigger long boundary-following detours. *Design limitation:* this world should have blocked the diagonal. Its results are less discriminating than intended.

---

## 2. Plan B — speed sweep (cluttered world, 3 starts; v_max 0.18 m/s taken from plan A)

| Strategy | v_max [m/s] | Success | Time [s] | Min clearance [m] | Goal error [m] |
|---|---|---|---|---|---|
| vfh | 0.10 | 3/3 | 69.4 ± 0.2 | 0.138 | 0.223 |
| vfh | 0.15 | 3/3 | 46.4 ± 0.1 | 0.142 | 0.216 |
| vfh | 0.18 (A) | 4/4 | 38.4 ± 0.5 | 0.142 | 0.211 |
| vfh | 0.22 | 3/3 | 32.0 ± 0.2 | 0.139 | 0.218 |
| bug2 | 0.10 | 2/3 | 69.6 (2 runs) | 0.054 | 1.24 (one timeout) |
| bug2 | 0.15 | 3/3 | 86.8 ± 69.6 | 0.061 | 0.229 |
| bug2 | 0.22 | 3/3 | 31.8 ± 0.3 | 0.051 | 0.215 |

Figure: `fig_B_speed.png`.

- **VFH:** completion time is inversely proportional to speed (69.4 × 0.10 ≈ 32.0 × 0.22 ≈ 7 m of path).
- **Clearance is independent of speed** in this range. This is consistent with the stopping-distance estimate (`explanation.md` §7.3): at 0.22 m/s the robot needs about 0.09 m + margin to stop, well inside the 0.10–0.60 m braking ramp. The Burger's 0.22 m/s limit is not a safety problem here.
- **Bug2's one failure at 0.10 m/s** was a timeout: slow boundary following did not finish within 200 s. It was not a safety failure.

---

## 3. Plan C — safety-margin sweep (cluttered world, 3 starts; defaults from plan A)

| Strategy | Parameter | Success | Min clearance [m] | Goal error [m] | Localization error [m] |
|---|---|---|---|---|---|
| vfh | safety 0.05 | 3/3 | 0.067 | 0.212 | 0.027 |
| vfh | safety 0.12 (A) | 4/4 | 0.142 | 0.211 | 0.023 |
| vfh | safety 0.20 | **1/3** | 0.164 | 0.508 | 0.324 |
| vfh | safety 0.30 | **0/3** | 0.265 | 0.566 | 0.559 |
| bug2 | d_follow 0.22 | 2/3 | 0.054 | 1.44 (one timeout) | 0.028 |
| bug2 | d_follow 0.30 (A) | 4/4 | 0.065 | 0.209 | 0.096 |
| bug2 | d_follow 0.40 | 2/3 | 0.050 | 2.19 (one timeout) | 0.055 |

Figure: `fig_C_safety.png`.

- **The expected effect:** a larger VFH margin buys clearance (0.07 → 0.27 m).
- **A surprising side-effect:** success collapses at larger margins. Every failure *reached* the goal by the robot's own estimate but was 0.40–0.74 m away in ground truth. With a large margin, the enlarged obstacles close most gaps, so VFH spends much more time in `AVOID`, weaving and turning (1,000+ AVOID cycles vs ~50 at the default). More turning means more wheel skid, and localization error rose from 0.02 m to 0.56 m.
  - *Interpretation (inference):* the safety parameter interacts with localization through rotation-induced slip. That is a coupling between control and estimation that a pure-geometry analysis would miss.
- **Bug2's timeouts at both non-default d_follow values** came from repeated boundary circling around pillar clusters (`ABANDON` / `FOLLOW_ARC` loops). The wall-follow gains were tuned for d_follow = 0.30 m.

---

## 4. Plan D — localization: wheel odometry only vs gyro-fused (3 starts; compare with plan A)

| World | Strategy | Localization | Success | Goal error [m] | Localization error, end [m] |
|---|---|---|---|---|---|
| open_field | vfh | odom_imu (A) | 4/4 | 0.201 | 0.023 |
| open_field | vfh | odom | 1/3 | 0.331 | 0.298 |
| open_field | bug2 | odom_imu (A) | 4/4 | 0.222 | 0.147 |
| open_field | bug2 | odom | 0/3 | 3.45 | 3.51 |
| cluttered | vfh | odom_imu (A) | 4/4 | 0.211 | 0.023 |
| cluttered | vfh | odom | 0/3 | 0.968 | 0.990 |
| cluttered | bug2 | odom_imu (A) | 4/4 | 0.209 | 0.096 |
| cluttered | bug2 | odom | 0/3 | 3.16 | 2.05 |

Figure: `fig_D_localization.png` (error vs time; solid = fused, dashed = wheel only).

- **Headline:** with wheel odometry alone, **success fell from 16/16 to 1/12**. The controllers still avoided every obstacle (0 collisions), because avoidance uses the LiDAR directly. But they went to the wrong place.
- **Error growth:** wheel-only error grows roughly linearly with time and turning, reaching 1–5 m. The fused estimate stays below 0.16 m in every successful trial.
- **Bug2 suffers most,** because boundary following involves the most rotation. Its hit/leave logic also depends on knowing where the m-line is.
- **Mechanism:** wheel slip during turns corrupts the odometry heading. The gyro measures true body rotation, which removes the dominant error source.

---

## 5. Plan E and the pilot — command shaping (Bug2, 3 starts)

| Configuration | open_field success | cluttered success | Localization error, open_field [m] |
|---|---|---|---|
| Bug2, shaping on (plan A) | 4/4 | 4/4 | 0.147 |
| Bug2, shaping off (plan E) | 0/3 | 1/3 | 0.461 |
| Pilot v1 (no shaping, no lost-wall recovery) | 0/4 | – | 0.40–0.52 |

- **What the pilot showed:** while Bug2 was commanded to turn *on the spot* (v = 0, ω up to 1.5 rad/s), the ground-truth robot **translated 0.53 m**. Neither the wheels nor the gyro measure that sideways skid, so it adds straight to the localization error.
- **The fix:** `CommandShaper` limits yaw acceleration to 2 rad/s², caps the in-place yaw rate at 0.6 rad/s and ramps forward speed. It cut Bug2's localization error roughly 3× and restored success.
- **Evidence:** plan E switches shaping off with everything else unchanged, and reproduces the failure. Commanding physically gentle motion is part of good localization, not just comfort.
- **Second pilot fix:** the pilot also exposed a Bug2 flaw: after passing a small pillar it orbited empty floor for over 120 s. The lost-wall rule now abandons following after a full circle without a wall. That event appears as `ABANDON` in the state counts.

---

## 6. Computational response

| Strategy | Decision time per cycle, mean | p95 (worst trial) | Budget at 10 Hz |
|---|---|---|---|
| vfh | ≈ 0.6 ms | ≈ 1.1 ms | 100 ms |
| bug2 | ≈ 0.14 ms | ≈ 0.3 ms | 100 ms |

Figure: `fig_compute.png`.

- Both run in about 1 % of the control period in pure Python, measured wall-clock in the VM.
- VFH is about 4× more expensive: it tests every LiDAR return against 72 heading bins, O(returns × bins). Bug2 only takes a few sector minima.
- Computation is not a limiting factor; the 5 Hz LiDAR is (up to 0.2 s of sensing delay).

---

## 7. Overall conclusions

1. **Neither strategy dominates.** The reactive VFH-style controller is faster (≈ 2.5×), smoother and keeps more clearance in rooms without concave traps, but fails completely in a U-shaped cul-de-sac. Bug2 is complete there (4/4) at the cost of 1.4–1.9× longer paths and closer passes.
2. **Localization, not obstacle avoidance, decided most failures.** All 75 trials were collision-free. Nearly every "reached but unsuccessful" trial was a pose-estimate problem, caused by wheel slip during rotation.
3. **Two engineering changes mattered most:** fusing the gyro for heading (plan D: 1/12 → 16/16 success) and limiting angular acceleration (plan E: 1/6 → 8/8).
4. **Speed up to the Burger's 0.22 m/s limit** reduced time proportionally with no loss of clearance in these worlds.
5. **Larger safety margins** increase clearance but cause excessive weaving, which inflates localization error. The default 0.12 m was the best compromise.

**Statistical caution:** only 3–4 trials per condition, with deterministic physics and seeded start offsets. Differences like 16/16 vs 1/12 or 4/4 vs 0/4 are clear, but small numeric differences (e.g. goal error 0.20 vs 0.22 m) are within run-to-run variation. These are simulation results; real-robot behaviour (tyre slip, sensor noise) will differ.
