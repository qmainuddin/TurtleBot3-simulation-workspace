# Task 02 results summary

75 trials. Time and path statistics use successful trials only; values are mean ± sample standard deviation. Success = navigator reported the goal, ground-truth goal error ≤ 0.30 m and zero collisions. Collisions use the footprint-circle rule in trial_monitor.py.

## Plan A — strategy comparison (default parameters)

| world | strategy | n | success | reached | time s (succ.) | path m (succ.) | path/straight | collisions | min clear m | goal err m | loc. err m (end) | wheel-odom drift m | decide ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cluttered | bug2 | 4 | 4/4 (100%) | 4/4 | 103.2 ± 75.6 | 11.28 ± 5.27 | 1.60 ± 0.75 | 0 | 0.065 | 0.209 ± 0.020 | 0.096 ± 0.078 | 3.020 ± 2.618 | 0.14 / 0.24 |
| cluttered | vfh | 4 | 4/4 (100%) | 4/4 | 38.4 ± 0.5 | 6.84 ± 0.04 | 0.97 ± 0.00 | 0 | 0.142 | 0.211 ± 0.005 | 0.023 ± 0.011 | 1.178 ± 1.123 | 0.58 / 1.09 |
| open_field | bug2 | 4 | 4/4 (100%) | 4/4 | 97.2 ± 1.5 | 9.68 ± 0.13 | 1.44 ± 0.03 | 0 | 0.087 | 0.222 ± 0.025 | 0.147 ± 0.034 | 4.889 ± 1.381 | 0.16 / 0.30 |
| open_field | vfh | 4 | 4/4 (100%) | 4/4 | 38.3 ± 0.6 | 6.76 ± 0.08 | 1.01 ± 0.01 | 0 | 0.131 | 0.201 ± 0.025 | 0.023 ± 0.014 | 1.164 ± 1.016 | 0.57 / 1.10 |
| u_trap | bug2 | 4 | 4/4 (100%) | 4/4 | 85.7 ± 3.0 | 10.79 ± 0.08 | 1.93 ± 0.01 | 0 | 0.097 | 0.197 ± 0.021 | 0.047 ± 0.018 | 0.693 ± 0.497 | 0.13 / 0.23 |
| u_trap | vfh | 4 | 0/4 (0%) | 0/4 | – | – | – | 0 | 0.396 | 2.972 ± 0.157 | 0.487 ± 0.098 | 0.425 ± 0.180 | 0.90 / 1.07 |

Unsuccessful trials:

- `A_u_trap_vfh_default_s0`: TIMEOUT, goal error 3.21 m, collisions 0, states {"AVOID": 1867, "GO_GOAL": 133}
- `A_u_trap_vfh_default_s1`: TIMEOUT, goal error 2.87 m, collisions 0, states {"AVOID": 1865, "GO_GOAL": 136}
- `A_u_trap_vfh_default_s2`: TIMEOUT, goal error 2.91 m, collisions 0, states {"AVOID": 1865, "GO_GOAL": 136}
- `A_u_trap_vfh_default_s3`: TIMEOUT, goal error 2.90 m, collisions 0, states {"AVOID": 1865, "GO_GOAL": 136}

## Plan B — speed sweep (cluttered world)

| strategy | params | n | success | reached | time s (succ.) | path m (succ.) | path/straight | collisions | min clear m | goal err m | loc. err m (end) | wheel-odom drift m | decide ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bug2 | v_max: 0.1 | 3 | 2/3 (67%) | 2/3 | 69.6 ± 0.4 | 6.88 ± 0.01 | 0.97 ± 0.00 | 0 | 0.054 | 1.238 ± 1.775 | 0.063 ± 0.069 | 1.343 ± 1.018 | 0.14 / 0.24 |
| bug2 | v_max: 0.15 | 3 | 3/3 (100%) | 3/3 | 86.8 ± 69.6 | 9.28 ± 4.16 | 1.31 ± 0.59 | 0 | 0.061 | 0.229 ± 0.029 | 0.084 ± 0.103 | 3.031 ± 3.394 | 0.12 / 0.16 |
| bug2 | v_max: 0.22 | 3 | 3/3 (100%) | 3/3 | 31.8 ± 0.3 | 6.86 ± 0.03 | 0.97 ± 0.00 | 0 | 0.051 | 0.215 ± 0.005 | 0.029 ± 0.007 | 1.166 ± 0.221 | 0.12 / 0.23 |
| vfh | v_max: 0.1 | 3 | 3/3 (100%) | 3/3 | 69.4 ± 0.2 | 6.86 ± 0.02 | 0.97 ± 0.00 | 0 | 0.138 | 0.223 ± 0.008 | 0.037 ± 0.015 | 1.619 ± 1.008 | 0.60 / 1.05 |
| vfh | v_max: 0.15 | 3 | 3/3 (100%) | 3/3 | 46.4 ± 0.1 | 6.86 ± 0.02 | 0.97 ± 0.00 | 0 | 0.142 | 0.216 ± 0.012 | 0.035 ± 0.005 | 1.608 ± 0.519 | 0.57 / 1.03 |
| vfh | v_max: 0.22 | 3 | 3/3 (100%) | 3/3 | 32.0 ± 0.2 | 6.87 ± 0.03 | 0.97 ± 0.00 | 0 | 0.139 | 0.218 ± 0.009 | 0.040 ± 0.014 | 1.835 ± 1.022 | 0.60 / 1.08 |

Unsuccessful trials:

- `B_cluttered_bug2_v_max0.1_s0`: TIMEOUT, goal error 3.29 m, collisions 0, states {"ABANDON": 2, "FOLLOW": 852, "FOLLOW_ARC": 767, "FOLLOW_TURN": 174, "GO_GOAL": 203, "LEAVE": 2}

## Plan C — safety-margin sweep (cluttered world)

| strategy | params | n | success | reached | time s (succ.) | path m (succ.) | path/straight | collisions | min clear m | goal err m | loc. err m (end) | wheel-odom drift m | decide ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| bug2 | d_follow: 0.22 | 3 | 2/3 (67%) | 2/3 | 39.0 ± 0.4 | 6.90 ± 0.02 | 0.97 ± 0.00 | 0 | 0.054 | 1.436 ± 2.152 | 0.028 ± 0.014 | 1.133 ± 0.342 | 0.12 / 0.24 |
| bug2 | d_follow: 0.4 | 3 | 2/3 (67%) | 2/3 | 39.0 ± 0.4 | 6.89 ± 0.01 | 0.97 ± 0.00 | 0 | 0.050 | 2.194 ± 3.448 | 0.055 ± 0.056 | 1.514 ± 1.625 | 0.13 / 0.23 |
| vfh | safety: 0.05 | 3 | 3/3 (100%) | 3/3 | 38.6 ± 0.3 | 6.86 ± 0.01 | 0.97 ± 0.00 | 0 | 0.067 | 0.212 ± 0.006 | 0.027 ± 0.000 | 1.177 ± 0.253 | 0.61 / 1.08 |
| vfh | safety: 0.2 | 3 | 1/3 (33%) | 3/3 | 40.1 | 6.87 | 0.97 | 0 | 0.164 | 0.508 ± 0.258 | 0.324 ± 0.252 | 1.885 ± 0.956 | 0.59 / 1.01 |
| vfh | safety: 0.3 | 3 | 0/3 (0%) | 3/3 | – | – | – | 0 | 0.265 | 0.566 ± 0.151 | 0.559 ± 0.199 | 5.544 ± 3.249 | 0.47 / 0.82 |

Unsuccessful trials:

- `C_cluttered_vfh_safety0.2_s0`: REACHED, goal error 0.56 m, collisions 0, states {"AVOID": 484, "GO_GOAL": 129, "REACHED": 2}
- `C_cluttered_vfh_safety0.2_s1`: REACHED, goal error 0.74 m, collisions 0, states {"AVOID": 621, "GO_GOAL": 203, "REACHED": 1}
- `C_cluttered_vfh_safety0.3_s0`: REACHED, goal error 0.69 m, collisions 0, states {"AVOID": 1000, "GO_GOAL": 191, "REACHED": 2}
- `C_cluttered_vfh_safety0.3_s1`: REACHED, goal error 0.61 m, collisions 0, states {"AVOID": 1165, "GO_GOAL": 176, "REACHED": 3}
- `C_cluttered_vfh_safety0.3_s2`: REACHED, goal error 0.40 m, collisions 0, states {"AVOID": 514, "GO_GOAL": 184, "REACHED": 1}
- `C_cluttered_bug2_d_follow0.22_s0`: TIMEOUT, goal error 3.92 m, collisions 0, states {"ABANDON": 7, "FOLLOW": 548, "FOLLOW_ARC": 1085, "FOLLOW_TURN": 206, "GO_GOAL": 154, "LEAVE": 1}
- `C_cluttered_bug2_d_follow0.4_s0`: TIMEOUT, goal error 6.18 m, collisions 0, states {"FOLLOW": 1376, "FOLLOW_ARC": 107, "FOLLOW_TURN": 330, "GO_GOAL": 188, "LEAVE": 1}

## Plan D — localization: wheel-only odometry (compare with plan A, gyro-fused)

| world | strategy | params | n | success | reached | time s (succ.) | path m (succ.) | path/straight | collisions | min clear m | goal err m | loc. err m (end) | wheel-odom drift m | decide ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cluttered | bug2 | localization: odom | 3 | 0/3 (0%) | 0/3 | – | – | – | 0 | 0.081 | 3.157 ± 1.670 | 2.048 ± 1.354 | 2.046 ± 1.352 | 0.14 / 0.23 |
| cluttered | vfh | localization: odom | 3 | 0/3 (0%) | 3/3 | – | – | – | 0 | 0.134 | 0.968 ± 0.281 | 0.990 ± 0.313 | 0.988 ± 0.310 | 0.65 / 1.12 |
| open_field | bug2 | localization: odom | 3 | 0/3 (0%) | 2/3 | – | – | – | 0 | 0.090 | 3.445 ± 0.997 | 3.505 ± 1.222 | 3.509 ± 1.228 | 0.13 / 0.24 |
| open_field | vfh | localization: odom | 3 | 1/3 (33%) | 3/3 | 49.5 | 7.34 | 1.09 | 0 | 0.099 | 0.331 ± 0.072 | 0.298 ± 0.039 | 0.299 ± 0.039 | 0.58 / 0.96 |

Unsuccessful trials:

- `D_open_field_vfh_localizationodom_s0`: REACHED, goal error 0.37 m, collisions 0, states {"AVOID": 185, "GO_GOAL": 298, "REACHED": 2}
- `D_open_field_vfh_localizationodom_s1`: REACHED, goal error 0.37 m, collisions 0, states {"AVOID": 192, "GO_GOAL": 180, "REACHED": 2}
- `D_open_field_bug2_localizationodom_s0`: REACHED, goal error 2.78 m, collisions 0, states {"FOLLOW": 513, "FOLLOW_ARC": 28, "FOLLOW_TURN": 138, "GO_GOAL": 249, "LEAVE": 2, "REACHED": 2}
- `D_open_field_bug2_localizationodom_s1`: REACHED, goal error 2.97 m, collisions 0, states {"FOLLOW": 502, "FOLLOW_ARC": 20, "FOLLOW_TURN": 148, "GO_GOAL": 263, "LEAVE": 2, "REACHED": 1}
- `D_open_field_bug2_localizationodom_s2`: TIMEOUT, goal error 4.59 m, collisions 0, states {"ABANDON": 2, "FOLLOW": 1114, "FOLLOW_ARC": 443, "FOLLOW_TURN": 294, "GO_GOAL": 147, "LEAVE": 2}
- `D_cluttered_vfh_localizationodom_s0`: REACHED, goal error 1.12 m, collisions 0, states {"AVOID": 274, "GO_GOAL": 176, "REACHED": 1}
- `D_cluttered_vfh_localizationodom_s1`: REACHED, goal error 0.64 m, collisions 0, states {"AVOID": 58, "GO_GOAL": 330, "REACHED": 1}
- `D_cluttered_vfh_localizationodom_s2`: REACHED, goal error 1.15 m, collisions 0, states {"AVOID": 395, "GO_GOAL": 145, "REACHED": 1}
- `D_cluttered_bug2_localizationodom_s0`: TIMEOUT, goal error 3.38 m, collisions 0, states {"FOLLOW": 1474, "FOLLOW_ARC": 26, "FOLLOW_TURN": 318, "GO_GOAL": 182, "LEAVE": 1}
- `D_cluttered_bug2_localizationodom_s1`: TIMEOUT, goal error 4.70 m, collisions 0, states {"FOLLOW": 1413, "FOLLOW_ARC": 34, "FOLLOW_TURN": 346, "GO_GOAL": 206, "LEAVE": 2}
- `D_cluttered_bug2_localizationodom_s2`: TIMEOUT, goal error 1.38 m, collisions 0, states {"FOLLOW": 1414, "FOLLOW_ARC": 44, "FOLLOW_TURN": 376, "GO_GOAL": 165, "LEAVE": 2}

## Plan E — Bug2 without command shaping (compare with plan A)

| world | strategy | params | n | success | reached | time s (succ.) | path m (succ.) | path/straight | collisions | min clear m | goal err m | loc. err m (end) | wheel-odom drift m | decide ms mean/p95 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| cluttered | bug2 | shaping: 0 | 3 | 1/3 (33%) | 1/3 | 38.7 | 6.86 | 0.97 | 0 | 0.048 | 1.871 ± 1.421 | 0.244 ± 0.188 | 4.093 ± 2.160 | 0.13 / 0.24 |
| open_field | bug2 | shaping: 0 | 3 | 0/3 (0%) | 3/3 | – | – | – | 0 | 0.065 | 0.476 ± 0.085 | 0.461 ± 0.038 | 5.994 ± 0.599 | 0.14 / 0.25 |

Unsuccessful trials:

- `E_open_field_bug2_shaping0_s0`: REACHED, goal error 0.46 m, collisions 0, states {"FOLLOW": 462, "FOLLOW_ARC": 10, "FOLLOW_TURN": 96, "GO_GOAL": 228, "LEAVE": 3, "REACHED": 2}
- `E_open_field_bug2_shaping0_s1`: REACHED, goal error 0.40 m, collisions 0, states {"FOLLOW": 428, "FOLLOW_ARC": 20, "FOLLOW_TURN": 90, "GO_GOAL": 221, "LEAVE": 3, "REACHED": 1}
- `E_open_field_bug2_shaping0_s2`: REACHED, goal error 0.57 m, collisions 0, states {"FOLLOW": 332, "FOLLOW_ARC": 18, "FOLLOW_TURN": 104, "GO_GOAL": 264, "LEAVE": 2, "REACHED": 1}
- `E_cluttered_bug2_shaping0_s0`: TIMEOUT, goal error 2.77 m, collisions 0, states {"FOLLOW": 1482, "FOLLOW_ARC": 56, "FOLLOW_TURN": 324, "GO_GOAL": 136, "LEAVE": 3}
- `E_cluttered_bug2_shaping0_s2`: TIMEOUT, goal error 2.61 m, collisions 0, states {"ABANDON": 1, "FOLLOW": 1157, "FOLLOW_ARC": 291, "FOLLOW_TURN": 280, "GO_GOAL": 268, "LEAVE": 4}

## Simulation speed

Real-time factor: mean 1.00, min 0.98, max 1.00 (simulated seconds per wall-clock second in the UTM VM).
