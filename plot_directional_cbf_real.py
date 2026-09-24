#!/usr/bin/env python3
"""Exactly four figures for one CBF-off and one CBF-on experiment.
Input: diagnostics CSV and /cbf_info CSV exported from the SAME rosbag.
Dependencies: numpy, matplotlib. No ROS required for plotting.
Raw u_nom/u_safe exclude Coriolis, before final limiting and actuator filtering.
cbf_constraint_nom/safe are model residuals for predicted actuator torques.
Missing fields are marked unavailable, never reconstructed as zero.
"""
import argparse
import csv
from pathlib import Path
import re
import sys
import numpy as np
INTEGER_FIELDS = {'stamp_ns', 'sample_index', 'experiment_id', 'time_segment'}

def normalize_ros_rows(names, rows):
    """Map rostopic diagnostics export to the recorder schema, without resampling."""
    if 'field.cbf_h' not in names:
        return names, rows
    if not rows:
        raise ValueError('CSV contains no samples')
    def convert_name(key):
        key = key[6:]
        if key == 'header.stamp': return 'stamp_ns'
        if key == 'header.frame_id': return 'frame_id'
        match = re.fullmatch(r'(direction|ee_position|target_position)([0-2])', key)
        if match: return match[1] + '_' + 'xyz'[int(match[2])]
        match = re.fullmatch(r'(svd_jacobian|q|dq|tau_command|tau_J_d|tau_J|tau_predicted)([0-6])', key)
        if match: return match[1] + '_' + str(int(match[2]) + 1)
        return key
    mapping = {k: convert_name(k) for k in names
               if k.startswith('field.') and k != 'field.header.seq'}
    required = {'stamp_ns', 'sample_index', 'experiment_id', 'cbf_active'}
    if not required.issubset(set(mapping.values())):
        raise ValueError('Use the CSV exported from /directional_cbf/diagnostics')
    converted = []
    previous_ns = previous_index = origin = None
    segment = 0
    for row in rows:
        out = {value: row[key] for key, value in mapping.items()}
        ns, index = int(out['stamp_ns']), int(out['sample_index'])
        restarted = previous_ns is not None and (ns <= previous_ns or index <= previous_index)
        if restarted:
            segment += 1
            origin = None
        if origin is None: origin = ns
        out.update(time_s=(ns-origin)*1e-9, time_segment=segment,
                   missing_samples_before=0 if previous_index is None or restarted else
                   max(0, index-previous_index-1))
        for key in ('cbf_active', 'task_aborted', 'cbf_solution_applied', 'debug_valid'):
            if key in out and str(out[key]).lower() in ('true', 'false'):
                out[key] = int(str(out[key]).lower() == 'true')
        converted.append(out)
        previous_ns, previous_index = ns, index
    print('Input: rostopic diagnostics CSV; timestamps and sample gaps preserved.')
    return list(converted[0]), converted

def load_csv(path):
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        names = reader.fieldnames or []
        names, rows = normalize_ros_rows(names, list(reader))
        required = {'time_s', 'time_segment', 'experiment_id', 'sample_index',
                    'cbf_h', 'cbf_constraint_safe', 'kinetic_energy_dir', 'Kmax', 'cbf_active'}
        absent = required - set(names)
        if absent:
            raise ValueError('Missing columns: ' + ', '.join(sorted(absent)))
    if not rows:
        raise ValueError('CSV contains no samples')
    data = {}
    for key in names:
        if key == 'frame_id':
            continue
        dtype = np.int64 if key in INTEGER_FIELDS else float
        try:
            data[key] = np.array([dtype(row[key]) if row[key] not in ('', None) else np.nan
                                  for row in rows], dtype=dtype)
        except (ValueError, TypeError):
            if key in required or key in INTEGER_FIELDS:
                raise ValueError('Invalid values in ' + key)
            # Ignore unknown nonnumeric extension fields.
    if not np.isfinite(data['time_s']).all():
        raise ValueError('Nonfinite time_s')
    return data

def attach_controls(data, path):
    """Join only exact integer header timestamps; never interpolate control signals."""
    if 'stamp_ns' not in data:
        raise ValueError('Control comparison requires stamp_ns in diagnostics')
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        groups = [g for g in ('u_des', 'u_cbf', 'u_nom', 'u_safe', 'coriolis')
                  if all('field.{}{}'.format(g, j) in fields for j in range(7))]
        if not {'u_des', 'u_cbf'}.issubset(groups):
            raise ValueError('--cbf-info requires the /cbf_info rostopic CSV')
        rows = {}
        for row in reader:
            stamp = int(row['field.header.stamp'])
            if stamp in rows:
                raise ValueError('Duplicate cbf_info header stamp; export one controller run at a time')
            rows[stamp] = row
    stamps = data['stamp_ns']
    if len(np.unique(stamps)) != len(stamps):
        raise ValueError('Duplicate diagnostic timestamps; export one controller run at a time')
    matched = np.array([int(t) in rows for t in stamps])
    for group in groups:
        for j in range(7):
            key = 'field.{}{}'.format(group, j)
            data['control_{}_{}'.format(group, j+1)] = np.array([
                float(rows[int(t)][key]) if int(t) in rows else np.nan for t in stamps])
    print('Control CSV: {}/{} exact timestamp matches (no interpolation).'.format(matched.sum(), len(stamps)))
    if not matched.any():
        raise ValueError('No matching header timestamps between the two CSVs')
    if 'u_safe' not in groups:
        print('Legacy data: u_cbf is FINAL command, not raw QP u_safe. Both compared torques include Coriolis.')

def col(data, key):
    return data.get(key, np.full(len(data['time_s']), np.nan))

def slices(data):
    """Never connect different experiments, resets, or missing samples."""
    edges = np.r_[0, np.flatnonzero(
        (np.diff(data['sample_index']) != 1) |
        (np.diff(data['experiment_id']) != 0) |
        (np.diff(data['time_segment']) != 0) |
        (np.diff(data['time_s']) <= 0) |
        (np.diff(data['cbf_active']) != 0)) + 1, len(data['time_s'])]
    return [slice(int(a), int(b)) for a, b in zip(edges[:-1], edges[1:])]

def line(ax, data, values, label, **kwargs):
    xx, yy = [], []
    for sl in slices(data):
        xx.extend(data['time_s'][sl]); xx.append(np.nan)
        yy.extend(values[sl]); yy.append(np.nan)
    ax.plot(xx, yy, label=label, linewidth=1, **kwargs)

def detect_motion(data, args):
    """Detect sustained Cartesian motion on a regular grid, independently per trial.

    The same EE-based criterion is used with CBF on/off. Joint velocity is not
    used because the recorder omits it when CBF is off.
    """
    t = data['time_s']
    if len(t) < 5 or t[-1] - t[0] < .2:
        return None
    dt = max(.002, float(np.median(np.diff(t))))
    grid = np.arange(t[0], t[-1], dt)
    position = np.column_stack([col(data, 'ee_position_'+c) for c in 'xyz'])
    if not np.isfinite(position).all():
        raise ValueError('Motion alignment requires finite ee_position_x/y/z columns; '
                         'joint velocities recorded only with CBF on cannot align both trials.')
    # A 100 ms centered local linear fit suppresses simulation velocity chatter.
    width = max(5, int(round(.1/dt)) | 1)
    if len(grid) < width:
        return None
    offsets = (np.arange(width) - width//2)*dt
    kernel = offsets / np.dot(offsets, offsets)
    velocity = []
    for i in range(3):
        x = np.interp(grid, t, position[:, i])
        velocity.append(np.correlate(np.pad(x, width//2, mode='edge'), kernel, mode='valid'))
    speed = np.linalg.norm(np.column_stack(velocity), axis=1)
    # Do not interpolate a motion decision through a large recording gap.
    for k in np.flatnonzero(np.diff(t) > .1):
        speed[(grid >= t[k]-.05) & (grid <= t[k+1]+.05)] = np.nan
    def first_run(mask, duration, start=0):
        count = max(1, int(np.ceil(duration/dt)))
        consecutive = 0
        for i in range(start, len(mask)):
            consecutive = consecutive+1 if mask[i] else 0
            if consecutive >= count:
                return i-count+1
        return None
    begin = first_run(speed >= args.motion_speed, .08)
    if begin is None:
        return None
    # Stop only after the last sustained motion burst, not a temporary pause.
    moving = speed >= args.motion_speed
    bursts = []
    edges = np.flatnonzero(np.diff(np.r_[False, moving, False]))
    for left, right in zip(edges[::2], edges[1::2]):
        if (right-left)*dt >= .08:
            bursts.append(right-1)
    end = first_run(np.isfinite(speed) & (speed <= args.stop_speed),
                    args.settle_time, bursts[-1]+1)
    return {'onset': float(grid[begin]),
            'stop': None if end is None else float(grid[end]),
            'peak_ee_speed': float(np.nanmax(speed)),
            'method': '100 ms local-linear EE velocity; sustained onset 80 ms'}

def choose_trials(data, args):
    trials = {}
    selected = [(0, args.off_id), (1, args.on_id)]
    for active, exp in selected:
        mask = data['experiment_id'] == exp
        if args.segment is not None:
            mask &= data['time_segment'] == args.segment
        if not mask.any():
            raise ValueError('Experiment {} not found'.format(exp))
        run = {k: v[mask].copy() for k, v in data.items()}
        if len(np.unique(run['time_segment'])) != 1:
            raise ValueError('Experiment ID reused after restart; select --segment')
        if not np.all(run['cbf_active'] == active):
            raise ValueError('Experiment {} is not consistently CBF {}'.format(exp, active))
        info = detect_motion(run, args) if args.align == 'motion' else None
        if args.align == 'motion' and info is None:
            raise ValueError('No sustained motion detected for experiment {}; inspect data or use --align command'.format(exp))
        origin = info['onset'] if info else run['time_s'][0]
        run['time_s'] -= origin
        trim = run['time_s'] >= -args.pre_motion
        if args.duration is not None:
            trim &= run['time_s'] <= args.duration
        run = {k: v[trim] for k, v in run.items()}
        if not len(run['time_s']):
            raise ValueError('Empty plotting interval')
        if np.any(col(run, 'task_aborted') == 1):
            print('WARNING: experiment {} contains abort samples. Raw torque comparison hides these samples.'.format(exp))
        print('CBF {}: experiment {}, alignment {:.9f} s, {} samples'.format(
            'on' if active else 'off', exp, origin, len(run['time_s'])))
        trials[active] = run
    return trials


def make_comparison(plt, trials):
    figures = []
    colors = {0: '#1769aa', 1: '#d64b27'}
    def new(name, n, size):
        fig, axes = plt.subplots(n, 1, sharex=True, figsize=size, constrained_layout=True)
        axes = np.atleast_1d(axes)
        figures.append((name, fig, axes))
        return axes
    def draw(ax, run, key, label, **style):
        values = col(run, key)
        if np.isfinite(values).any():
            line(ax, run, values, label, **style)
        else:
            print('Unavailable: {} ({})'.format(key, label))
            # A legend entry explains missing data without inventing a curve.
            ax.plot([], [], label=label+' [unavailable]', **style)
    def overlay(ax, key):
        for active, run in trials.items():
            draw(ax, run, key, 'CBF '+('on' if active else 'off'), color=colors[active])
    axes = new('energy_distance', 3, (12, 10))
    overlay(axes[0], 'kinetic_energy_dir')
    for active, run in trials.items():
        draw(axes[0], run, 'Kmax', 'Kmax '+('on' if active else 'off'),
             color=colors[active], ls='--')
    axes[0].set_ylabel('Directional energy [J]')
    axes[0].set_title('Directional energy and dashed energy bounds')
    overlay(axes[1], 'kinetic_energy_total')
    axes[1].set_ylabel('Total energy [J]')
    axes[1].set_title('Total kinetic energy (Kmax bounds directional energy only)')
    overlay(axes[2], 'ee_target_distance')
    axes[2].set_ylabel('Target distance [m]')
    axes = new('joint_trajectories', 7, (12, 15))
    for j, ax in enumerate(axes, 1):
        overlay(ax, 'q_{}'.format(j))
        ax.set_ylabel('q{} [rad]'.format(j))
    axes[0].set_title('Joint trajectories')
    axes = new('cbf_barrier_constraints', 3, (12, 10))
    for ax, key, label in zip(axes, ('cbf_h', 'cbf_constraint_nom', 'cbf_constraint_safe'),
                              ('h [J]', 'Nominal residual [J/s]', 'Applied-command residual [J/s]')):
        overlay(ax, key)
        ax.axhline(0, color='black', ls='--', lw=1, label='Zero boundary')
        ax.set_ylabel(label)
        ax.set_title(key)
    axes = new('joint_controls', 7, (12, 15))
    off, on = trials[0], trials[1]
    for j, ax in enumerate(axes, 1):
        for run, field, label, color, ls in (
            (off, 'u_nom', 'CBF off: u_nom', '#1769aa', '-'),
            (on, 'u_safe', 'CBF on: u_safe', '#d64b27', '-'),
            (on, 'u_nom', 'CBF on: u_nom', '#258443', '--')):
            key = 'control_{}_{}'.format(field, j)
            view = dict(run)
            view[key] = np.where(col(run, 'task_aborted') == 0, col(run, key), np.nan)
            draw(ax, view, key, label, color=color, ls=ls)
        ax.set_ylabel('Joint {} [Nm]'.format(j))
    axes[0].set_title('Raw control: Coriolis excluded, before final limits/filtering')
    for _, fig, axes in figures:
        for ax in axes:
            ax.grid(True, alpha=.25)
            ax.legend(loc='best', fontsize=8)
        axes[-1].set_xlabel('Time from alignment [s]')
    return figures


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv', type=Path, help='/directional_cbf/diagnostics CSV')
    p.add_argument('--cbf-info', required=True, type=Path, help='/cbf_info CSV')
    p.add_argument('--off-id', required=True, type=int)
    p.add_argument('--on-id', required=True, type=int)
    p.add_argument('--segment', type=int)
    p.add_argument('--align', choices=('motion', 'command'), default='motion',
                   help='motion: detected EE motion onset; command: first sample of experiment ID')
    p.add_argument('--pre-motion', type=float, default=.2)
    p.add_argument('--duration', type=float, help='Seconds after alignment; default entire trial')
    p.add_argument('--motion-speed', type=float, default=.01)
    p.add_argument('--stop-speed', type=float, default=.005)
    p.add_argument('--settle-time', type=float, default=.5)
    p.add_argument('--output-dir', type=Path, default=Path('plots_two_tests'))
    p.add_argument('--format', choices=('png', 'pdf', 'svg'), default='png')
    p.add_argument('--no-show', action='store_true')
    args = p.parse_args()
    if args.off_id == args.on_id:
        raise ValueError('Select two different experiment IDs')
    for name in ('pre_motion', 'motion_speed', 'stop_speed', 'settle_time', 'duration'):
        value = getattr(args, name)
        if value is not None and (not np.isfinite(value) or value < 0 or (name != 'pre_motion' and value == 0)):
            raise ValueError('Invalid '+name)
    if args.stop_speed >= args.motion_speed:
        raise ValueError('stop-speed must be less than motion-speed')
    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data = load_csv(args.csv)
    attach_controls(data, args.cbf_info)
    trials = choose_trials(data, args)
    figures = make_comparison(plt, trials)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, fig, _ in figures:
        path = args.output_dir / (name+'.'+args.format)
        fig.savefig(str(path), dpi=150)
        print('Saved '+str(path))
    if not args.no_show:
        plt.show()
    plt.close('all')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError) as exc:
        sys.exit('Plot failed: '+str(exc))
