#!/usr/bin/env python3
"""Run the Task 02 experiment plans in Gazebo, one headless trial at a time.

    python3 run_experiments.py --plan smoke          # 1 quick trial to check the pipeline
    python3 run_experiments.py --plan A B C          # full study (resumable: finished trials are skipped)
    python3 run_experiments.py --plan A --dry-run    # just list the trials

Experiment design (fixed before collecting data)
  A  strategy comparison : 3 worlds x {vfh, bug2} x 4 start variants, default parameters
  B  speed sweep         : cluttered world, {vfh, bug2} x v_max {0.10, 0.15, 0.22} x 3 variants
                           (v_max 0.18 comes from plan A)
  C  safety-margin sweep : cluttered world, vfh safety {0.05, 0.20, 0.30} and
                           bug2 d_follow {0.22, 0.40}, x 3 variants (defaults come from plan A)
  D  localization        : open_field + cluttered, {vfh, bug2}, wheel-only odometry
                           (localization=odom) x 3 variants; compared with plan A, which uses the
                           default gyro-fused estimate (localization=odom_imu)
  E  command shaping     : bug2, open_field + cluttered, CommandShaper disabled (shaping=0)
                           x 3 variants; compared with plan A (shaping on)
Start variant 0 is the nominal start pose; variants 1..n add a deterministic offset
(+/-0.15 m, +/-0.6 rad) so repeated trials are not identical (seeded, reproducible).
Every trial: same robot, physics, goal tolerance (0.20 m nav / 0.30 m success), timeout 200 s sim.
"""
import argparse
import csv
import json
import math
import os
import random
import signal
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))  # allow running from the source tree
from tb3_nav_task.geometry import body_clearance  # noqa: E402
from tb3_nav_task.worlds import WORLDS  # noqa: E402

DEFAULT_RESULTS = os.path.expanduser('~/vm-share/task02/results')
TIMEOUT_SIM = 200.0


def start_variant(world_name: str, k: int):
    w = WORLDS[world_name]
    if k == 0:
        return w.start
    rng = random.Random(f'{world_name}-{k}')
    for _ in range(100):
        x = w.start[0] + rng.uniform(-0.15, 0.15)
        y = w.start[1] + rng.uniform(-0.15, 0.15)
        yaw = w.start[2] + rng.uniform(-0.6, 0.6)
        if body_clearance(w, x, y, yaw) > 0.25:
            return (round(x, 3), round(y, 3), round(yaw, 3))
    return w.start


def build_plan(names):
    trials = []

    def add(plan, world, strategy, variant, params):
        tag = '_'.join(f'{k}{v}' for k, v in sorted(params.items())) or 'default'
        tid = f'{plan}_{world}_{strategy}_{tag}_s{variant}'
        trials.append(dict(trial_id=tid, plan=plan, world=world, strategy=strategy,
                           variant=variant, params=params, start=start_variant(world, variant)))

    if 'smoke' in names:
        add('smoke', 'open_field', 'vfh', 0, {})
    if 'A' in names:
        for world in ('open_field', 'cluttered', 'u_trap'):
            for strategy in ('vfh', 'bug2'):
                for k in range(4):
                    add('A', world, strategy, k, {})
    if 'B' in names:
        for strategy in ('vfh', 'bug2'):
            for v in (0.10, 0.15, 0.22):
                for k in range(3):
                    add('B', 'cluttered', strategy, k, {'v_max': v})
    if 'C' in names:
        for s in (0.05, 0.20, 0.30):
            for k in range(3):
                add('C', 'cluttered', 'vfh', k, {'safety': s})
        for d in (0.22, 0.40):
            for k in range(3):
                add('C', 'cluttered', 'bug2', k, {'d_follow': d})
    if 'D' in names:
        for world in ('open_field', 'cluttered'):
            for strategy in ('vfh', 'bug2'):
                for k in range(3):
                    add('D', world, strategy, k, {'localization': 'odom'})
    if 'E' in names:
        for world in ('open_field', 'cluttered'):
            for k in range(3):
                add('E', world, 'bug2', k, {'shaping': 0})
    return trials


def done_ids(results_dir):
    path = os.path.join(results_dir, 'results.csv')
    if not os.path.exists(path):
        return set()
    with open(path) as f:
        return {r['trial_id'] for r in csv.DictReader(f)}


def cleanup():
    for pat in ('gz sim', 'parameter_bridge', 'robot_state_publisher', 'ros_gz_sim/create',
                'tb3_nav_task/navigator', 'tb3_nav_task/trial_monitor'):
        subprocess.run(['pkill', '-f', pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    for pat in ('gz sim', 'parameter_bridge'):
        subprocess.run(['pkill', '-9', '-f', pat], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)


def run_trial(t, results_dir, gui, min_rtf):
    x, y, yaw = t['start']
    cmd = ['ros2', 'launch', 'tb3_nav_task', 'trial.launch.py',
           f'world:={t["world"]}', f'strategy:={t["strategy"]}', f'gui:={"true" if gui else "false"}',
           f'trial_id:={t["trial_id"]}', f'plan:={t["plan"]}', f'variant:={t["variant"]}',
           f'start_x:={x}', f'start_y:={y}', f'start_yaw:={yaw}',
           f'nav_params:={json.dumps(t["params"])}', f'results_dir:={results_dir}',
           f'timeout:={TIMEOUT_SIM}', 'verbose:=1']
    os.makedirs(os.path.join(results_dir, 'logs'), exist_ok=True)
    log_path = os.path.join(results_dir, 'logs', f'{t["trial_id"]}.log')
    wall_limit = TIMEOUT_SIM / min_rtf + 120
    t0 = time.monotonic()
    with open(log_path, 'w') as log:
        log.write(' '.join(cmd) + '\n')
        log.flush()
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            proc.wait(timeout=wall_limit)
        except subprocess.TimeoutExpired:
            log.write(f'\n[run_experiments] wall-clock limit {wall_limit:.0f}s exceeded, stopping\n')
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
    cleanup()
    return time.monotonic() - t0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plan', nargs='+', default=['A', 'B', 'C', 'D', 'E'])
    ap.add_argument('--results-dir', default=DEFAULT_RESULTS)
    ap.add_argument('--gui', action='store_true', help='show Gazebo (slower)')
    ap.add_argument('--min-rtf', type=float, default=0.25,
                    help='lowest expected real-time factor, sets the wall-clock safety limit')
    ap.add_argument('--retries', type=int, default=1)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    trials = build_plan(args.plan)
    print(f'{len(trials)} trials in plan(s) {args.plan}; results -> {args.results_dir}')
    if args.dry_run:
        for t in trials:
            print(f'  {t["trial_id"]:45s} start={t["start"]}')
        return
    os.makedirs(args.results_dir, exist_ok=True)
    cleanup()
    for i, t in enumerate(trials, 1):
        for attempt in range(1 + args.retries):
            if t['trial_id'] in done_ids(args.results_dir):
                print(f'[{i}/{len(trials)}] {t["trial_id"]}: already done, skipping')
                break
            print(f'[{i}/{len(trials)}] {t["trial_id"]} (attempt {attempt + 1}) ...', flush=True)
            wall = run_trial(t, args.results_dir, args.gui, args.min_rtf)
            if t['trial_id'] in done_ids(args.results_dir):
                with open(os.path.join(args.results_dir, 'results.csv')) as f:
                    row = [r for r in csv.DictReader(f) if r['trial_id'] == t['trial_id']][-1]
                print(f'    -> {row["outcome"]} success={row["success"]} time={row["time_s"]}s '
                      f'path={row["path_m"]}m collisions={row["collisions"]} '
                      f'min_clear={row["min_clear_m"]} rtf={row["rtf"]} (wall {wall:.0f}s)', flush=True)
                break
            print(f'    -> no result written (see logs/{t["trial_id"]}.log)', flush=True)
    print('ALL PLANS FINISHED')


if __name__ == '__main__':
    main()
