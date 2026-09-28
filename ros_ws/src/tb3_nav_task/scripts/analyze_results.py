#!/usr/bin/env python3
"""Summarise Task 02 results: tables (Markdown) + figures (PNG) from results.csv and traces/.

    python3 analyze_results.py [--results-dir ~/vm-share/task02/results]

Outputs (in <results-dir>/analysis/):
  summary.md                    per-plan tables (mean +/- sample std; success-only time/path)
  fig_A_trajectories.png        ground-truth paths of every plan-A trial, per world
  fig_A_metrics.png             success rate, time, path ratio, min clearance by world x strategy
  fig_B_speed.png               speed sweep (cluttered)
  fig_C_safety.png              safety-margin sweep (cluttered)
  fig_compute.png               per-cycle decision time distribution by strategy
"""
import argparse
import csv
import math
import os
import statistics as st
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

from tb3_nav_task.geometry import Box, Cylinder  # noqa: E402
from tb3_nav_task.worlds import WORLDS  # noqa: E402

COLORS = {'vfh': '#1f77b4', 'bug2': '#d62728'}
NUM = ['success', 'reached', 'time_s', 'path_m', 'path_ratio', 'collisions', 'min_clear_m',
       'goal_err_m', 'odom_err_m', 'est_err_m', 'est_err_max_m', 'compute_mean_ms', 'compute_p95_ms', 'compute_max_ms', 'rtf',
       'contact_s']


def load(results_dir):
    with open(os.path.join(results_dir, 'results.csv')) as f:
        rows = list(csv.DictReader(f))
    latest = {}
    for r in rows:                      # keep the last row per trial id (re-runs)
        for k in NUM:
            try:
                r[k] = float(r[k])
            except (KeyError, ValueError):
                r[k] = math.nan
        latest[r['trial_id']] = r
    return list(latest.values())


def ms(vals, fmt='{:.2f}'):
    vals = [v for v in vals if not math.isnan(v)]
    if not vals:
        return '–'
    if len(vals) == 1:
        return fmt.format(vals[0])
    return f'{fmt.format(st.mean(vals))} ± {fmt.format(st.stdev(vals))}'


def param_label(r):
    p = r.get('params', '{}')
    return 'default' if p in ('{}', '') else p.replace('"', '').strip('{}')


def table(rows, keys):
    groups = defaultdict(list)
    for r in rows:
        groups[tuple(r[k] if k != 'params' else param_label(r) for k in keys)].append(r)
    head = keys + ['n', 'success', 'reached', 'time s (succ.)', 'path m (succ.)', 'path/straight',
                   'collisions', 'min clear m', 'goal err m', 'loc. err m (end)', 'wheel-odom drift m', 'decide ms mean/p95']
    out = ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    for g in sorted(groups):
        rs = groups[g]
        succ = [r for r in rs if r['success'] == 1]
        out.append('| ' + ' | '.join(list(g) + [
            str(len(rs)),
            f'{sum(r["success"] for r in rs):.0f}/{len(rs)} ({100 * st.mean(r["success"] for r in rs):.0f}%)',
            f'{sum(r["reached"] for r in rs):.0f}/{len(rs)}',
            ms([r['time_s'] for r in succ], '{:.1f}'),
            ms([r['path_m'] for r in succ]),
            ms([r['path_ratio'] for r in succ]),
            f'{sum(r["collisions"] for r in rs):.0f}',
            f'{min(r["min_clear_m"] for r in rs):.3f}',
            ms([r['goal_err_m'] for r in rs], '{:.3f}'),
            ms([r['est_err_m'] for r in rs], '{:.3f}'),
            ms([r['odom_err_m'] for r in rs], '{:.3f}'),
            f'{st.mean(r["compute_mean_ms"] for r in rs):.2f} / {max(r["compute_p95_ms"] for r in rs):.2f}',
        ]) + ' |')
    return '\n'.join(out)


def draw_world(ax, w):
    for o in w.solids():
        if isinstance(o, Box):
            c, s = math.cos(o.yaw), math.sin(o.yaw)
            pts = [(o.cx + c * dx - s * dy, o.cy + s * dx + c * dy)
                   for dx, dy in ((-o.sx / 2, -o.sy / 2), (o.sx / 2, -o.sy / 2),
                                  (o.sx / 2, o.sy / 2), (-o.sx / 2, o.sy / 2))]
            ax.add_patch(Polygon(pts, closed=True, color='0.45'))
        elif isinstance(o, Cylinder):
            ax.add_patch(Circle((o.cx, o.cy), o.r, color='0.45'))
    ax.plot(*w.start[:2], 's', color='#1f3fbf', ms=9, label='start')
    ax.add_patch(Circle(w.goal, 0.2, color='#2ca02c', alpha=0.5))
    ax.plot(*w.goal, '*', color='#2ca02c', ms=14, label='goal')
    ax.plot([w.start[0], w.goal[0]], [w.start[1], w.goal[1]], ':', color='0.6', lw=1)
    ax.set_xlim(-w.size_x / 2 - 0.2, w.size_x / 2 + 0.2)
    ax.set_ylim(-w.size_y / 2 - 0.2, w.size_y / 2 + 0.2)
    ax.set_aspect('equal')
    ax.set_title(w.name)


def read_trace(results_dir, tid):
    path = os.path.join(results_dir, 'traces', f'{tid}.csv')
    if not os.path.exists(path):
        return [], []
    xs, ys = [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            xs.append(float(r['x']))
            ys.append(float(r['y']))
    return xs, ys


def fig_trajectories(rows, results_dir, out):
    worlds = [w for w in WORLDS if any(r['world'] == w for r in rows)]
    if not worlds:
        return
    fig, axes = plt.subplots(1, len(worlds), figsize=(6 * len(worlds), 4.6))
    axes = [axes] if len(worlds) == 1 else axes
    for ax, wn in zip(axes, worlds):
        draw_world(ax, WORLDS[wn])
        seen = set()
        for r in rows:
            if r['world'] != wn:
                continue
            xs, ys = read_trace(results_dir, r['trial_id'])
            lab = r['strategy'] if r['strategy'] not in seen else None
            seen.add(r['strategy'])
            ax.plot(xs, ys, color=COLORS.get(r['strategy'], 'k'), lw=1.4, alpha=0.8,
                    ls='-' if r['success'] == 1 else '--', label=lab)
        ax.legend(loc='lower right', fontsize=8)
    fig.suptitle('Plan A: ground-truth trajectories (solid = success, dashed = failure)')
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def grouped_bars(ax, rows, metric, title, succ_only=False, agg=st.mean):
    worlds = [w for w in WORLDS if any(r['world'] == w for r in rows)]
    strategies = [s for s in ('vfh', 'bug2') if any(r['strategy'] == s for r in rows)]
    width = 0.8 / max(1, len(strategies))
    for j, s in enumerate(strategies):
        vals, errs = [], []
        for w in worlds:
            rs = [r[metric] for r in rows if r['world'] == w and r['strategy'] == s
                  and (not succ_only or r['success'] == 1) and not math.isnan(r[metric])]
            vals.append(agg(rs) if rs else 0.0)
            errs.append(st.stdev(rs) if len(rs) > 1 and agg is st.mean else 0.0)
        xs = [i + (j - (len(strategies) - 1) / 2) * width for i in range(len(worlds))]
        ax.bar(xs, vals, width, yerr=errs, capsize=3, color=COLORS[s], label=s)
    ax.set_xticks(range(len(worlds)))
    ax.set_xticklabels(worlds)
    ax.set_title(title)
    ax.grid(axis='y', alpha=0.3)


def fig_metrics(rows, out):
    fig, axes = plt.subplots(1, 4, figsize=(18, 4))
    grouped_bars(axes[0], rows, 'success', 'Success rate')
    axes[0].set_ylim(0, 1.05)
    grouped_bars(axes[1], rows, 'time_s', 'Completion time [s sim] (successful)', succ_only=True)
    grouped_bars(axes[2], rows, 'path_ratio', 'Path length / straight line (successful)', succ_only=True)
    grouped_bars(axes[3], rows, 'min_clear_m', 'Minimum body clearance [m] (worst trial)', agg=min)
    axes[0].legend()
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def sweep_fig(rows, key, default, xlabel, out, title):
    fig, axes = plt.subplots(1, 4, figsize=(18, 4))
    for s in ('vfh', 'bug2'):
        pts = defaultdict(list)
        for r in rows:
            if r['strategy'] != s:
                continue
            p = r.get('params', '{}')
            val = default.get(s)
            for part in p.strip('{}').split(','):
                if ':' in part:
                    k, v = part.split(':')
                    if k.strip().strip('"') == key.get(s):
                        val = float(v)
            if val is not None:
                pts[val].append(r)
        if not pts:
            continue
        xs = sorted(pts)
        def series(m, succ=False, agg=st.mean):
            ys = []
            for x in xs:
                v = [r[m] for r in pts[x] if (not succ or r['success'] == 1) and not math.isnan(r[m])]
                ys.append(agg(v) if v else math.nan)
            return ys
        lbl = f'{s} ({key[s]})'
        axes[0].plot(xs, series('success'), 'o-', color=COLORS[s], label=lbl)
        axes[1].plot(xs, series('time_s', True), 'o-', color=COLORS[s], label=lbl)
        axes[2].plot(xs, series('path_m', True), 'o-', color=COLORS[s], label=lbl)
        axes[3].plot(xs, series('min_clear_m', agg=min), 'o-', color=COLORS[s], label=lbl)
    for ax, t in zip(axes, ('Success rate', 'Time [s sim] (successful)', 'Path [m] (successful)',
                            'Min body clearance [m] (worst)')):
        ax.set_title(t)
        ax.set_xlabel(xlabel)
        ax.grid(alpha=0.3)
    axes[0].set_ylim(-0.05, 1.05)
    axes[0].legend()
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def fig_localization(a_rows, d_rows, results_dir, out):
    """Localization error over time: gyro-fused (plan A) vs wheel-only (plan D), same starts."""
    fig, axes = plt.subplots(1, 2, figsize=(13, 4))
    for ax, world in zip(axes, ('open_field', 'cluttered')):
        for rows, style, lab in ((a_rows, '-', 'odom_imu (gyro-fused)'), (d_rows, '--', 'odom (wheel only)')):
            first = True
            for r in rows:
                if r['world'] != world:
                    continue
                path = os.path.join(results_dir, 'traces', f'{r["trial_id"]}.csv')
                if not os.path.exists(path):
                    continue
                ts, es = [], []
                with open(path) as f:
                    for q in csv.DictReader(f):
                        try:
                            ts.append(float(q['t']))
                            es.append(math.hypot(float(q['est_x']) - float(q['x']),
                                                 float(q['est_y']) - float(q['y'])))
                        except (KeyError, ValueError):
                            pass
                ax.plot(ts, es, style, color=COLORS.get(r['strategy'], 'k'), alpha=0.7, lw=1.2,
                        label=f'{lab}' if first else None)
                first = False
        ax.set_title(f'{world}: |estimated - true position|')
        ax.set_xlabel('simulated time [s]')
        ax.set_ylabel('localization error [m]')
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def fig_compute(rows, out):
    fig, ax = plt.subplots(figsize=(6, 4))
    data = [[r['compute_mean_ms'] for r in rows if r['strategy'] == s] for s in ('vfh', 'bug2')]
    ax.boxplot(data, labels=['vfh', 'bug2'])
    ax.set_ylabel('mean decision time per cycle [ms]')
    ax.set_title('Computational response (all trials)')
    ax.grid(axis='y', alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results-dir', default=os.path.expanduser('~/vm-share/task02/results'))
    a = ap.parse_args()
    rows = load(a.results_dir)
    out = os.path.join(a.results_dir, 'analysis')
    os.makedirs(out, exist_ok=True)
    plans = defaultdict(list)
    for r in rows:
        plans[r['plan']].append(r)

    md = ['# Task 02 results summary', '',
          f'{len(rows)} trials. Time and path statistics use successful trials only; '
          'values are mean ± sample standard deviation. Success = navigator reported the goal, '
          'ground-truth goal error ≤ 0.30 m and zero collisions. Collisions use the footprint-circle '
          'rule in trial_monitor.py.', '']
    titles = {'A': 'Plan A — strategy comparison (default parameters)',
              'B': 'Plan B — speed sweep (cluttered world)',
              'C': 'Plan C — safety-margin sweep (cluttered world)',
              'D': 'Plan D — localization: wheel-only odometry (compare with plan A, gyro-fused)',
              'E': 'Plan E — Bug2 without command shaping (compare with plan A)'}
    for p in sorted(plans):
        keys = (['world', 'strategy'] if p == 'A' else
                ['world', 'strategy', 'params'] if p in ('D', 'E') else ['strategy', 'params'])
        md += [f'## {titles.get(p, "Plan " + p)}', '', table(plans[p], keys), '']
        fails = [r for r in plans[p] if r['success'] != 1]
        if fails:
            md += ['Unsuccessful trials:', '']
            md += [f'- `{r["trial_id"]}`: {r["outcome"]}, goal error {r["goal_err_m"]:.2f} m, '
                   f'collisions {r["collisions"]:.0f}, states {r["states"]}' for r in fails]
            md += ['']
    md += ['## Simulation speed', '',
           f'Real-time factor: mean {st.mean(r["rtf"] for r in rows):.2f}, '
           f'min {min(r["rtf"] for r in rows):.2f}, max {max(r["rtf"] for r in rows):.2f} '
           '(simulated seconds per wall-clock second in the UTM VM).', '']
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write('\n'.join(md))

    if plans.get('A'):
        fig_trajectories(plans['A'], a.results_dir, os.path.join(out, 'fig_A_trajectories.png'))
        fig_metrics(plans['A'], os.path.join(out, 'fig_A_metrics.png'))
    base = [r for r in plans.get('A', []) if r['world'] == 'cluttered']
    if plans.get('B'):
        sweep_fig(plans['B'] + base, {'vfh': 'v_max', 'bug2': 'v_max'}, {'vfh': 0.18, 'bug2': 0.18},
                  'v_max [m/s]', os.path.join(out, 'fig_B_speed.png'), 'Plan B: speed sweep (cluttered)')
    if plans.get('C'):
        sweep_fig(plans['C'] + base, {'vfh': 'safety', 'bug2': 'd_follow'},
                  {'vfh': 0.12, 'bug2': 0.30}, 'vfh safety margin / bug2 wall distance [m]',
                  os.path.join(out, 'fig_C_safety.png'), 'Plan C: safety-margin sweep (cluttered)')
    if plans.get('D'):
        a_match = [r for r in plans.get('A', []) if r['world'] in ('open_field', 'cluttered')
                   and int(float(r.get('variant', 0) or 0)) < 3]
        fig_localization(a_match, plans['D'], a.results_dir, os.path.join(out, 'fig_D_localization.png'))
    fig_compute(rows, os.path.join(out, 'fig_compute.png'))
    print('\n'.join(md))
    print('figures ->', out)


if __name__ == '__main__':
    main()
