"""Analyse square-test recordings: odometry vs. ground truth.

Usage:
  ros2 run bld120_sim analyze_square RUN.csv [RUN2.csv ...] --out DIR

For each run, both trajectories are expressed relative to their own starting
pose, then the script reports
  * return error: how far the *true* robot ends from where it started (the
    UMBmark quantity: the robot believes it returned to the start),
  * odometry error at the end: |odometry pose - true pose|, position and yaw,
and plots the paths. A JSON summary is written next to the plots.
"""
import argparse
import csv
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ODOM, TRUTH = 'odom', 'ground_truth/odom'


def load(path):
    rows = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.setdefault(r['topic'], []).append(
                (float(r['t']), float(r['x']), float(r['y']), float(r['yaw'])))
    return {k: np.array(v) for k, v in rows.items()}


def relative_to_start(track):
    """Express a (t, x, y, yaw) track in the frame of its first pose."""
    t, x, y, yaw = track.T
    c, s = math.cos(yaw[0]), math.sin(yaw[0])
    dx, dy = x - x[0], y - y[0]
    rel_yaw = np.arctan2(np.sin(yaw - yaw[0]), np.cos(yaw - yaw[0]))
    return np.column_stack([t, c * dx + s * dy, -s * dx + c * dy, rel_yaw])


def analyse(path):
    data = load(path)
    odom = relative_to_start(data[ODOM])
    truth = relative_to_start(data[TRUTH]) if TRUTH in data else None
    result = {'run': os.path.splitext(os.path.basename(path))[0],
              'duration_s': float(odom[-1, 0] - odom[0, 0])}
    if truth is not None:
        # Compare at the last odometry sample's time.
        t_end = odom[-1, 0]
        i = min(np.searchsorted(truth[:, 0], t_end), len(truth) - 1)
        truth = truth[:i + 1]  # ignore anything recorded after odometry stopped
        tx, ty, tyaw = truth[-1, 1:]
        ox, oy, oyaw = odom[-1, 1:]
        result.update({
            'return_error_m': float(math.hypot(tx, ty)),
            'return_error_xy_m': [float(tx), float(ty)],
            'odom_end_error_m': float(math.hypot(ox - tx, oy - ty)),
            'odom_end_yaw_error_deg': float(math.degrees(
                math.atan2(math.sin(oyaw - tyaw), math.cos(oyaw - tyaw)))),
        })
    return result, odom, truth


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('runs', nargs='+', help='CSV files written by pose_recorder')
    ap.add_argument('--out', default='.', help='output directory for plots and summary')
    ap.add_argument('--title', default='Square test: odometry vs. ground truth')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    results = []
    n = len(args.runs)
    fig, axes = plt.subplots(1, n, figsize=(5 * n, 5), squeeze=False)
    for ax, path in zip(axes[0], args.runs):
        result, odom, truth = analyse(path)
        results.append(result)
        if truth is not None:
            ax.plot(truth[:, 1], truth[:, 2], color='#1f6feb', lw=2, label='ground truth')
        ax.plot(odom[:, 1], odom[:, 2], color='#d29922', lw=1.5, ls='--', label='odometry')
        ax.plot(0, 0, 'ko', ms=5, label='start')
        if truth is not None:
            ax.plot(truth[-1, 1], truth[-1, 2], 'x', color='#1f6feb', ms=9, mew=2,
                    label=f'true end ({100 * result["return_error_m"]:.1f} cm off)')
        ax.set_title(result['run'])
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.legend(fontsize=8, loc='best')
    fig.suptitle(args.title)
    fig.tight_layout()
    plot_path = os.path.join(args.out, 'square_paths.png')
    fig.savefig(plot_path, dpi=120)

    summary_path = os.path.join(args.out, 'square_summary.json')
    with open(summary_path, 'w') as f:
        json.dump(results, f, indent=2)
    for r in results:
        line = f'{r["run"]:>28}: {r["duration_s"]:6.1f} s'
        if 'return_error_m' in r:
            line += (f' | true return error {100 * r["return_error_m"]:6.2f} cm'
                     f' | odom end error {100 * r["odom_end_error_m"]:6.2f} cm,'
                     f' {r["odom_end_yaw_error_deg"]:+6.2f} deg')
        print(line)
    print(f'wrote {plot_path} and {summary_path}')
