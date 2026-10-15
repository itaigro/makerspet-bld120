"""Analyse step-test recordings: wheel speed response to velocity commands.

Usage:
  ros2 run bld120_sim analyze_steps RUN.csv [RUN2.csv ...] --out DIR [--labels A B ...]

For each run (left wheel; the profile is symmetric), per command segment:
  * step segments: rise time (10-90 % of the change), overshoot, and the
    steady-state error over the segment's last second,
  * the creep segment (0.02 m/s): mean speed and the fraction of time the
    wheel is stopped (the PWM dead zone shows up here),
  * the stop segments: time to come to rest.
Writes ``steps_response.png``, ``steps_zoom.png`` and ``steps_summary.json``.
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

WHEEL_RADIUS = 0.0215
WHEEL_TRACK = 0.105043
STOPPED = 0.05          # rad/s: below this a wheel counts as stopped
COLORS = ('#1f6feb', '#d29922', '#2da44e', '#cf222e')


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))

    def col(name):
        return np.array([float(r[name]) if r[name] != '' else np.nan for r in rows])

    data = {k: col(k) for k in rows[0].keys()}
    data['cmd_left'] = (data['cmd_v'] - 0.5 * WHEEL_TRACK * data['cmd_w']) / WHEEL_RADIUS
    # On the real robot the left wheel's joint turns negative when driving
    # forward (mirrored motor); in simulation it is positive. Use the sign
    # that agrees with the command.
    if np.nansum(data['js_left'] * data['cmd_left']) < 0:
        data['js_left'] = -data['js_left']
    return data


def first_command_time(data):
    """Time of the first non-zero command: the common time origin of all runs."""
    moving = (data['cmd_v'] != 0) | (data['cmd_w'] != 0)
    return data['t'][np.argmax(moving)]


def segments(data):
    """(start index, end index, commanded left speed) for each constant command."""
    cmd = np.column_stack([data['cmd_v'], data['cmd_w']])
    change = np.flatnonzero(np.any(np.diff(cmd, axis=0) != 0, axis=1)) + 1
    bounds = np.concatenate([[0], change, [len(cmd)]])
    return [(a, b, data['cmd_left'][a]) for a, b in zip(bounds[:-1], bounds[1:]) if b > a]


def step_metrics(t, w, w0, w1):
    """Rise time, overshoot and final error for a change from w0 to w1."""
    delta = w1 - w0
    frac = (w - w0) / delta
    t10 = t[np.argmax(frac >= 0.1)] if np.any(frac >= 0.1) else math.nan
    t90 = t[np.argmax(frac >= 0.9)] if np.any(frac >= 0.9) else math.nan
    overshoot = max(0.0, float(np.nanmax(frac)) - 1.0) * 100.0
    last = t >= t[-1] - 1.0
    return {
        'rise_time_ms': (t90 - t10) * 1e3,
        'overshoot_pct': overshoot,
        'steady_error_pct': float(np.nanmean(w[last]) - w1) / abs(w1) * 100.0 if w1 else
        float(np.nanmean(w[last])),
    }


def analyse(data):
    t, w = data['t'], data['js_left']
    t0 = t[0]
    out = {'segments': []}
    prev_cmd = 0.0
    for a, b, cmd in segments(data):
        ts, ws = t[a:b], w[a:b]
        seg = {'t_start_s': float(ts[0] - t0), 'cmd_left_rad_s': float(cmd)}
        if cmd != prev_cmd and cmd != 0.0 and abs(cmd) > 2.0:
            seg.update(step_metrics(ts, ws, prev_cmd, cmd))
        elif cmd == 0.0 and prev_cmd != 0.0:
            moving = np.abs(ws) > STOPPED
            seg['time_to_stop_ms'] = float(
                (ts[np.argmin(moving)] - ts[0]) * 1e3) if not moving.all() else math.nan
        elif 0.0 < abs(cmd) <= 2.0:  # creep
            seg['creep_mean_rad_s'] = float(np.nanmean(ws))
            seg['creep_stopped_pct'] = float(np.mean(np.abs(ws) < STOPPED) * 100.0)
        out['segments'].append(seg)
        prev_cmd = cmd
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--labels', nargs='+')
    ap.add_argument('--out', default='.')
    args = ap.parse_args()
    labels = args.labels or [os.path.splitext(os.path.basename(r))[0] for r in args.runs]
    os.makedirs(args.out, exist_ok=True)
    runs = [load(r) for r in args.runs]

    # Full response
    fig, (ax, ax_duty) = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True,
                                      gridspec_kw={'height_ratios': [3, 1]})
    # Runs start recording at slightly different moments; align them on their
    # first command, shown at 1 s (the profile's initial pause).
    for data in runs:
        data['t_plot'] = data['t'] - first_command_time(data) + 1.0
    ax.step(runs[0]['t_plot'], runs[0]['cmd_left'], where='post', color='#57606a',
            lw=1, ls=':', label='command')
    for data, label, color in zip(runs, labels, COLORS):
        t = data['t_plot']
        ax.plot(t, data['js_left'], color=color, lw=1.4, label=label)
        if not np.all(np.isnan(data['duty_left'])):
            ax_duty.plot(t, data['duty_left'], color=color, lw=1, label=f'{label} duty')
    ax.set_ylabel('left wheel speed [rad/s]')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(alpha=0.3)
    ax_duty.set_ylabel('PWM duty')
    ax_duty.set_xlabel('time [s]')
    ax_duty.grid(alpha=0.3)
    ax.set_title('Wheel response to the step profile')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'steps_response.png'), dpi=120)

    # Zooms: acceleration step, stop, creep. The command timer quantises each
    # step's start (10 Hz on the robot), so every panel aligns each run on that
    # step's own start instead of only on the first command.
    starts = [[data['t_plot'][a] for a, _, _ in segments(data)] for data in runs]
    windows = (('0.10 -> 0.20 m/s', 2, 2, -0.1, 0.6),
               ('0.20 m/s -> stop', 3, 3, -0.1, 0.6),
               ('creep 0.02 m/s', 4, 5, -0.2, 0.3))
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    for axz, (title, first, last, before, after) in zip(axes, windows):
        ref = starts[0][first]
        axz.step(runs[0]['t_plot'], runs[0]['cmd_left'], where='post', color='#57606a',
                 lw=1, ls=':', label='command')
        for data, run_starts, label, color in zip(runs, starts, labels, COLORS):
            axz.plot(data['t_plot'] - run_starts[first] + ref, data['js_left'], color=color,
                     lw=1.4, label=label)
        axz.set_xlim(ref + before, starts[0][last] + after)
        axz.set_title(title)
        axz.set_xlabel('time [s]')
        axz.grid(alpha=0.3)
    axes[0].set_ylabel('left wheel speed [rad/s]')
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'steps_zoom.png'), dpi=120)

    summary = {label: analyse(data) for data, label in zip(runs, labels)}
    with open(os.path.join(args.out, 'steps_summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)
    for label, result in summary.items():
        print(f'== {label}')
        for seg in result['segments']:
            extras = {k: round(v, 2) for k, v in seg.items()
                      if k not in ('t_start_s', 'cmd_left_rad_s')}
            if extras:
                print(f'  t={seg["t_start_s"]:5.1f}s cmd={seg["cmd_left_rad_s"]:+6.2f} '
                      f'rad/s  {extras}')
    print(f'wrote plots and summary to {args.out}')
