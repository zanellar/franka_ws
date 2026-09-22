#!/usr/bin/env python3
"""Standalone CBF comparison and diagnostics (Python 3 + numpy + matplotlib).

python3 plot_directional_cbf.py /path/to/cbf.csv --show
python3 plot_directional_cbf.py /path/to/cbf.csv --experiments 2 4 --format pdf

Only the distance/energy comparison is displayed. All individual diagnostic
figures are always saved in the plots/ directory beside the CSV. --no-show
saves without opening a window. No ROS, workspace, URDF or metadata is required.
URDF/metadata/contact files are used only when available beside the CSV.

Motion is detected from a 100 ms local-linear fit to the EE position: speed
above 0.01 m/s for 80 ms starts motion; speed below 0.005 m/s for 0.5 s marks
settling after the last sustained burst. Each trial is aligned independently,
with 1 s before onset and 2 s after stopping. Unrecorded history stays blank.
Joint debug is not reconstructed when absent. URDF bounds require a companion
URDF or metadata; no default joint bounds are guessed from the robot name.
"""
import argparse
import csv
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np


INTEGER_FIELDS = {'stamp_ns', 'sample_index', 'experiment_id', 'time_segment'}


def load_csv(path):
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        names = reader.fieldnames or []
        required = {'time_s', 'time_segment', 'experiment_id', 'sample_index',
                    'cbf_h', 'cbf_constraint_safe', 'kinetic_energy_dir', 'Kmax', 'cbf_active'}
        absent = required - set(names)
        if absent:
            raise ValueError('Missing columns: ' + ', '.join(sorted(absent)))
        rows = list(reader)
    if not rows:
        raise ValueError('CSV contains no samples')
    data = {}
    for key in names:
        if key == 'frame_id':
            continue
        dtype = np.int64 if key in INTEGER_FIELDS else float
        try:
            data[key] = np.array([dtype(row[key]) if row[key] else np.nan
                                  for row in rows], dtype=dtype)
        except (ValueError, TypeError):
            if key in required or key in INTEGER_FIELDS:
                raise ValueError('Invalid values in ' + key)
            # Ignore unknown nonnumeric extension fields.
    if not np.isfinite(data['time_s']).all():
        raise ValueError('Nonfinite time_s')
    return data


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


def unavailable(ax, message):
    ax.text(.5, .5, message, ha='center', va='center', transform=ax.transAxes,
            fontsize=10, bbox=dict(facecolor='white', alpha=.85, edgecolor='none'))


def read_json(path):
    return json.loads(path.read_text()) if path.is_file() else {}


def joint_limits(urdf, metadata, arm_id=None):
    limits = dict(metadata.get('joint_limits_rad', {}))
    xml_limits = {}
    if urdf.is_file():
        for joint in ET.parse(str(urdf)).getroot().findall('joint'):
            limit = joint.find('limit')
            if limit is not None and 'lower' in limit.attrib and 'upper' in limit.attrib:
                xml_limits[joint.attrib['name']] = {
                    k: float(limit.attrib[k]) for k in ('lower', 'upper')}
    limits.update(xml_limits)  # The supplied URDF takes precedence.
    names = metadata.get('joint_names', [])
    if arm_id:
        names = ['{}_joint{}'.format(arm_id, i) for i in range(1, 8)]
    if not names:
        prefixes = [k[:-1] for k in limits if k.endswith('joint1') and
                    all(k[:-1] + str(i) in limits for i in range(1, 8))]
        if len(prefixes) == 1:
            names = [prefixes[0] + str(i) for i in range(1, 8)]
    if len(names) != 7:
        names = ['joint{}'.format(i) for i in range(1, 8)]
        print('WARNING: joint mapping unavailable/ambiguous; use --arm-id or metadata.json.')
    if not all(name in limits for name in names):
        print('WARNING: some joint limits unavailable; no default robot limits are assumed.')
    return names, limits


def debug_col(data, key):
    values = col(data, key).copy()
    valid = (col(data, 'cbf_active') == 1)
    if 'debug_valid' in data:
        valid &= data['debug_valid'] == 1
    values[~valid] = np.nan
    return values


def derivative_check(data):
    """Forward interval derivative; model at interval start, no cross-event differences."""
    measured = np.full(len(data['time_s']), np.nan)
    model = col(data, 'cbf_constraint_safe') - col(data, 'alpha') * data['cbf_h']
    if 'stamp_ns' in data:
        dt = np.diff(data['stamp_ns']).astype(float) * 1e-9
    else:
        dt = np.diff(data['time_s'])
    valid = (np.diff(data['sample_index']) == 1) & (dt > 0)
    for name in ('experiment_id', 'time_segment', 'Kmax', 'alpha', 'cbf_active',
                 'task_aborted', 'direction_x', 'direction_y', 'direction_z'):
        values = col(data, name)
        valid &= np.isfinite(values[:-1]) & (values[:-1] == values[1:])
    valid &= (col(data, 'cbf_active')[:-1] == 1)
    valid &= (col(data, 'task_aborted')[:-1] == 0)
    differences = np.full(len(dt), np.nan)
    np.divide(np.diff(data['cbf_h']), dt, out=differences, where=valid)
    measured[:-1] = differences
    model[~np.isfinite(measured)] = np.nan
    return measured, model


def contact_rows(path, data, origin):
    if not path.is_file():
        return []
    experiments = set(int(x) for x in data['experiment_id'])
    segment = int(data['time_segment'][0])
    lo, hi = data['time_s'][0], data['time_s'][-1]
    rows = []
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            if int(row['time_segment']) != segment or int(row['experiment_id']) not in experiments:
                continue
            # Subtract integer stamps BEFORE conversion to float.
            t = (int(row['stamp_ns']) - origin[0]) * 1e-9 + origin[1]
            if lo <= t <= hi:
                row['t'] = t
                rows.append(row)
    return rows


def make_figures(plt, data, names, limits, contacts, debug=False):
    figures = []
    def new(name, rows, size=None):
        fig, axes = plt.subplots(rows, 1, sharex=True, figsize=size or (13, 3*rows),
                                 constrained_layout=True, squeeze=False)
        figures.append((name, fig, axes.ravel()))
        return axes.ravel()

    axes = new('directional_energy', 1, (7.2, 12.8))
    line(axes[0], data, data['kinetic_energy_dir'], 'Directional kinetic energy', color='#165a9f')
    line(axes[0], data, data['Kmax'], 'Kmax', color='black', ls='--', drawstyle='steps-post')
    axes[0].set_ylabel('Directional kinetic energy [J]')
    axes[0].set_title('Directional kinetic energy')
    axes[0].set_ylim(bottom=0)

    axes = new('distance', 1, (12, 5))
    distance = col(data, 'ee_target_distance')
    if not np.isfinite(distance).any():
        ee = np.column_stack([col(data, 'ee_position_'+c) for c in 'xyz'])
        target = np.column_stack([col(data, 'target_position_'+c) for c in 'xyz'])
        distance = np.linalg.norm(ee-target, axis=1)
    if np.isfinite(distance).any():
        line(axes[0], data, distance, 'Distance to requested target', color='#165a9f')
    else:
        unavailable(axes[0], 'Target distance unavailable in this CSV')
    axes[0].set_ylabel('EE distance from target [m]')
    axes[0].set_title('End-effector distance from requested target')
    axes[0].set_ylim(bottom=0)

    axes = new('cbf_h', 1)
    ax_h = axes[0]
    ax_residual = new('cbf_constraint_safe', 1)[0]
    axes = [ax_h, ax_residual]
    line(axes[0], data, data['cbf_h'], 'h = Kmax - Kdir')
    axes[0].axhline(0, color='black', ls='--', lw=.8)
    axes[0].set_ylabel('cbf_h [J]')
    line(axes[1], data, data['cbf_constraint_safe'], 'applied-command model residual')
    if 'cbf_constraint_qp' in data:
        line(axes[1], data, data['cbf_constraint_qp'], 'QP residual', ls=':')
    line(axes[1], data, -col(data, 'cbf_residual_tolerance'), '-solver tolerance', color='gray', ls='--')
    axes[1].axhline(0, color='black', ls='--', lw=.8)
    axes[1].set_ylabel('CBF residual [J/s]')
    position_axes = new('joint_positions', 7, (12, 16))
    velocity_axes = new('joint_velocities', 7, (12, 16))
    axes = np.column_stack([position_axes, velocity_axes])
    for i, name in enumerate(names, 1):
        axq, axdq = axes[i-1]
        q = debug_col(data, 'q_{}'.format(i))
        line(axq, data, q, name, color='C0')
        if name in limits:
            lower, upper = limits[name]['lower'], limits[name]['upper']
            axq.axhline(lower, color='black', ls='--', lw=.8, label='lower {:.4f}'.format(lower))
            axq.axhline(upper, color='black', ls='--', lw=.8, label='upper {:.4f}'.format(upper))
            violated = (q < lower) | (q > upper)
            axq.plot(data['time_s'][violated], q[violated], 'r.', ms=3, label='outside URDF limits')
        else:
            axq.text(.01, .05, 'Limits unavailable', transform=axq.transAxes)
        dq = debug_col(data, 'dq_{}'.format(i))
        line(axdq, data, dq, name, color='C1')
        for ax, values in ((axq, q), (axdq, dq)):
            if not np.isfinite(values).any():
                unavailable(ax, 'No recorded joint debug data')
        axq.set_ylabel('q{} [rad]'.format(i)); axdq.set_ylabel('dq{} [rad/s]'.format(i))
    axes[0, 0].set_title('Position - dashed URDF bounds; red: violations')
    axes[0, 1].set_title('Velocity - recorded only with CBF enabled')

    if debug:
        axes = new('joint_margins', 1, (13, 5))
        for i, name in enumerate(names, 1):
            if name in limits:
                q = debug_col(data, 'q_{}'.format(i))
                margin = np.minimum(q-limits[name]['lower'], limits[name]['upper']-q)
                line(axes[0], data, margin, name)
        axes[0].axhline(0, color='black', ls='--', lw=.8)
        axes[0].set_ylabel('Signed distance to nearest joint bound [rad]')
        axes[0].set_title('min(q - lower, upper - q): negative values are outside URDF limits')

    axes = new('jacobian', 2)
    singular = [col(data, 'svd_jacobian_{}'.format(i)) for i in range(1, 7)]
    for i, values in enumerate(singular, 1):
        line(axes[0], data, values, 'sigma{}'.format(i))
    ratio = np.full(len(data['time_s']), np.nan)
    np.divide(singular[0], singular[-1], out=ratio, where=singular[-1] > 0)
    line(axes[1], data, ratio, 'sigma1 / sigma6')
    if np.any(np.isfinite(ratio) & (ratio > 0)):
        axes[1].set_yscale('log')
    if np.any(singular[-1] == 0):
        axes[1].text(.02, .9, 'sigma6 = 0 occurs: infinite ratio omitted', transform=axes[1].transAxes)
    axes[0].set_ylabel('Singular values of full J (6 x 7)')
    axes[1].set_ylabel('Condition ratio')
    axes[0].set_title('Full Jacobian mixes linear/angular units; ratio depends on scaling')

    if debug:
        axes = new('energy_torque', 3)
        line(axes[0], data, col(data, 'kinetic_energy_total'), 'Ktotal')
        for i in range(1, 8):
            line(axes[1], data, debug_col(data, 'tau_command_{}'.format(i)), 'tau{}'.format(i))
        line(axes[2], data, debug_col(data, 'robot_mode'), 'robot_mode', drawstyle='steps-post')
        for ax, label in zip(axes, ('Ktotal [J]', 'Commanded torque [Nm]', 'Robot mode')):
            ax.set_ylabel(label)
        axes[2].set_yticks(range(7))
        axes[2].set_yticklabels(['Other', 'Idle', 'Move', 'Guiding', 'Reflex', 'Stopped', 'Recovery'])
        axes[1].set_title('Controller command; not a measurement of the applied Gazebo torque')

        axes = new('model_check', 3)
        measured, model = derivative_check(data)
        line(axes[0], data, measured, 'observed [h(k+1)-h(k)] / dt')
        line(axes[0], data, model, 'model at k: residual - alpha*h', ls='--')
        line(axes[1], data, measured-model, 'observed - model')
        axes[1].axhline(0, color='black', ls='--', lw=.8)
        axes[0].set_ylabel('h derivative [J/s]'); axes[1].set_ylabel('Mismatch [J/s]')
        axes[0].set_title('Forward-interval comparison; only contiguous, active, non-aborted, constant-parameter samples')
        if not np.isfinite(measured).any():
            unavailable(axes[0], 'No eligible intervals for derivative comparison')
        for field in ('cbf_active', 'task_aborted', 'cbf_solution_applied', 'solver_status'):
            line(axes[2], data, col(data, field), field, drawstyle='steps-post')
        axes[2].set_ylabel('Flags / solver code')

        axes = new('parameters_timing', 4)
        line(axes[0], data, data['Kmax'], 'Kmax', drawstyle='steps-post')
        line(axes[1], data, col(data, 'alpha'), 'alpha', drawstyle='steps-post')
        for component in 'xyz':
            line(axes[2], data, col(data, 'direction_'+component), 'direction_'+component)
        observed_dt = np.full(len(data['time_s']), np.nan)
        if 'stamp_ns' in data:
            observed_dt[1:] = np.diff(data['stamp_ns']).astype(float) * 1e-6
        else:
            observed_dt[1:] = np.diff(data['time_s']) * 1000
        line(axes[3], data, observed_dt, 'recorded stamp interval [ms]')
        line(axes[3], data, col(data, 'dt') * 1000, 'controller period [ms]', ls='--')
        missing = col(data, 'missing_samples_before') > 0
        axes[3].plot(data['time_s'][missing], observed_dt[missing], 'rx', ms=4, label='missing samples before row')
        for ax, label in zip(axes, ('Kmax [J]', 'alpha [1/s]', 'Unit direction', 'Period [ms]')):
            ax.set_ylabel(label)

        axes = new('contacts', 3)
        axes[0].set_ylabel('Reported contact pairs')
        axes[1].set_ylabel('Max point force [N]')
        axes[2].set_ylabel('Max penetration [mm]')
        axes[0].set_title('Physical contact reports; joint stops are not necessarily collision contacts')
        if not contacts:
            for ax in axes:
                unavailable(ax, 'No contact samples available for this selection\n(missing data does not mean zero contacts)')
        else:
            axes[0].plot([r['t'] for r in contacts], [int(r['contact_count']) for r in contacts],
                         '.', ms=2, label='contact_count')
            pairs = sorted({(r['collision1'], r['collision2']) for r in contacts if int(r['contact_count']) > 0})
            for i, pair in enumerate(pairs, 1):
                print('Contact pair {}: {} <-> {}'.format(i, *pair))
                selected = [r for r in contacts if (r['collision1'], r['collision2']) == pair]
                for ax, key, scale in ((axes[1], 'max_point_force_norm', 1), (axes[2], 'max_depth_m', 1000)):
                    ax.plot([r['t'] for r in selected], [float(r[key])*scale if r[key] else np.nan for r in selected],
                            '.', ms=2, label='pair {}'.format(i))
            zeros = [r for r in contacts if int(r['contact_count']) == 0]
            for ax in axes[1:]:
                ax.plot([r['t'] for r in zeros], [0]*len(zeros), '.', color='gray', ms=2,
                        label='reported no contacts')
            aged = [r for r in contacts if int(r['cbf_age_ns']) > 0]
            if aged:
                axes[0].plot([r['t'] for r in aged], [int(r['contact_count']) for r in aged],
                             'x', ms=3, label='non-exact CBF time association')
            axes[2].set_title('\n'.join('pair {}: {} <-> {}'.format(i, *pair)
                                       for i, pair in enumerate(pairs[:4], 1)), fontsize=7)
    return figures


def state_spans(data, field):
    """Piecewise constant states; never shade across missing samples or resets."""
    times, states = data['time_s'], col(data, field)
    for i in range(len(times)-1):
        if (data['sample_index'][i+1] != data['sample_index'][i]+1 or
                data['time_segment'][i+1] != data['time_segment'][i] or
                data['experiment_id'][i+1] != data['experiment_id'][i] or
                times[i+1] <= times[i] or not np.isfinite(states[i])):
            continue
        yield float(times[i]), float(times[i+1]), states[i]


def merged_spans(data, field):
    previous = None
    for left, right, state in state_spans(data, field):
        if previous is not None and previous[1] == left and previous[2] == state:
            previous = (previous[0], right, state)
        else:
            if previous is not None:
                yield previous
            previous = (left, right, state)
    if previous is not None:
        yield previous


def decorate(figures, data, title, events):
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    states = list(merged_spans(data, 'cbf_active'))
    aborted = [span for span in merged_spans(data, 'task_aborted') if span[2] == 1]
    colors = {0: '#d74a49', 1: '#32965a'}
    for name, fig, axes in figures:
        fig.suptitle(title, fontsize=11)
        for ax in axes:
            for left, right, state in states:
                if state in colors:
                    ax.axvspan(left, right, color=colors[state], alpha=.12, linewidth=0, zorder=0)
            for left, right, _ in aborted:
                ax.axvspan(left, right, facecolor='none', edgecolor='#777777',
                           hatch='///', linewidth=0, zorder=1)
            for event in events:
                if data['time_s'][0] <= event <= data['time_s'][-1]:
                    ax.axvline(event, color='#b40000', ls=':', lw=1, alpha=.8)
            ax.grid(True, alpha=.20)
            ax.set_axisbelow(True)
            handles, labels = ax.get_legend_handles_labels()
            if name == 'directional_energy':
                for state, label in ((0, 'CBF off'), (1, 'CBF on')):
                    if any(span[2] == state for span in states):
                        handles.append(Patch(facecolor=colors[state], alpha=.12))
                        labels.append(label)
                if aborted:
                    handles.append(Patch(facecolor='none', edgecolor='#777777', hatch='///'))
                    labels.append('Task aborted')
                if events:
                    handles.append(Line2D([], [], color='#b40000', ls=':'))
                    labels.append('First h < 0 with CBF on')
            if handles:
                ax.legend(handles, labels, loc='best', fontsize=8,
                          ncol=1 if name == 'directional_energy' else min(3, len(labels)))
            ax.set_xlabel('Simulation time since recording segment start [s]')
            ax.ticklabel_format(axis='x', style='plain', useOffset=False)
        if name != 'directional_energy':
            fig.supxlabel('Background: red = CBF off, green = CBF on; blank gaps = no interval data', fontsize=8)


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


def motion_trials(all_data, args):
    trials = []
    allowed = [args.experiment] if args.experiment is not None else args.experiments
    # Boundaries are positional, including repeated IDs after controller resets.
    boundaries = np.r_[0, np.flatnonzero(
        (np.diff(all_data['experiment_id']) != 0) |
        (np.diff(all_data['time_segment']) != 0) |
        (np.diff(all_data['cbf_active']) != 0)) + 1, len(all_data['time_s'])]
    for left, right in zip(boundaries[:-1], boundaries[1:]):
        segment = int(all_data['time_segment'][left])
        experiment = int(all_data['experiment_id'][left])
        active = int(all_data['cbf_active'][left])
        if args.segment is not None and segment != args.segment:
            continue
        if allowed is not None and experiment not in allowed:
            continue
        if active not in (0, 1):
            continue
        run = {k: v[left:right] for k, v in all_data.items()}
        info = detect_motion(run, args)
        if info is None:
            print('Skipping segment {}, experiment {}: no sustained motion detected.'.format(segment, experiment))
            continue
        if args.start is not None and info['onset'] < args.start:
            continue
        if args.end is not None and info['onset'] > args.end:
            continue
        origin = info['onset'] - args.pre_motion
        end = run['time_s'][-1] if info['stop'] is None else min(
            run['time_s'][-1], info['stop'] + args.post_motion)
        if args.end is not None:
            end = min(end, args.end)
        # Pre-command context may have CBF off even for a subsequent CBF-on trial.
        # Retain the measured samples and flags; curve color identifies the trial.
        first = left
        while (first > 0 and all_data['time_segment'][first-1] == segment and
               all_data['time_s'][first-1] >= origin):
            first -= 1
        indexes = np.arange(first, right)
        indexes = indexes[(all_data['time_s'][indexes] >= origin) &
                          (all_data['time_s'][indexes] <= end)]
        if args.start is not None:
            indexes = indexes[all_data['time_s'][indexes] >= args.start]
        if not len(indexes):
            continue
        view = {k: v[indexes].copy() for k, v in all_data.items()}
        # Before the command, measure distance to THIS trial's target, not the old one.
        pre = indexes < left
        if pre.any():
            # The previous trial's threshold is not the threshold for this motion.
            view['Kmax'][pre] = np.nan
            goal = np.array([col(run, 'target_position_'+c)[0] for c in 'xyz'])
            if np.isfinite(goal).all():
                pos = np.column_stack([col(view, 'ee_position_'+c) for c in 'xyz'])
                dist = col(view, 'ee_target_distance').copy()
                dist[pre] = np.linalg.norm(pos[pre]-goal, axis=1)
                view['ee_target_distance'] = dist
            else:
                dist = col(view, 'ee_target_distance').copy()
                dist[pre] = np.nan
                view['ee_target_distance'] = dist
        available_pre = info['onset'] - view['time_s'][0]
        available_post = None if info['stop'] is None else view['time_s'][-1] - info['stop']
        info.update(segment=segment, experiment=experiment, cbf_active=active,
                    origin=origin, first_recorded=float(view['time_s'][0]),
                    last_recorded=float(view['time_s'][-1]),
                    available_pre_seconds=float(available_pre),
                    available_post_seconds=None if available_post is None else float(available_post))
        print('Experiment {} (CBF {}): onset {:.3f} s, stop {}, pre {:.3f} s, post {}.'.format(
            experiment, 'on' if active else 'off', info['onset'],
            'not confirmed' if info['stop'] is None else '{:.3f} s'.format(info['stop']),
            available_pre, 'not available' if available_post is None else '{:.3f} s'.format(available_post)))
        if available_pre < args.pre_motion-.02:
            print('  WARNING: requested pre-motion history is incomplete; missing data remain blank.')
        if info['stop'] is None:
            print('  WARNING: no sustained stop detected; retaining the available trial tail.')
        elif available_post < args.post_motion-.02:
            print('  WARNING: recording ends before the requested post-stop duration.')
        view['time_s'] -= origin
        trials.append((info, view))
    if not trials:
        raise ValueError('No motion detected in the selected trials. Check --motion-speed or experiment selection.')
    return trials


def comparison_figure(plt, trials):
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(7.2, 12.8), constrained_layout=True)
    colors = {0: '#c74740', 1: '#22864f'}
    styles = ['-', '--', '-.', ':']
    counts = {0: 0, 1: 0}
    for info, data in trials:
        active = info['cbf_active']; style = styles[counts[active] % len(styles)]
        counts[active] += 1
        label = 'CBF {} - experiment {}'.format('on' if active else 'off', info['experiment'])
        for ax, key in zip(axes, ('ee_target_distance', 'kinetic_energy_dir')):
            line(ax, data, col(data, key), label, color=colors[active], ls=style)
        line(axes[1], data, data['Kmax'], 'Kmax - experiment {}'.format(info['experiment']),
             color=colors[active], ls=':', drawstyle='steps-post')
    # Make each threshold visibly dashed, even when both trials share Kmax.
    for curve in axes[1].lines:
        if curve.get_label().startswith('Kmax'):
            curve.set_linestyle('--')
    onset = trials[0][0]['onset']-trials[0][0]['origin']
    for ax in axes:
        ax.axvline(onset, color='gray', ls=':', lw=.8, label='Detected motion onset')
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    axes[0].set_title('End-effector distance from target')
    axes[0].set_ylabel('Distance [m]')
    axes[1].set_title('Directional kinetic energy')
    axes[1].set_ylabel('Energy [J]')
    axes[1].set_xlabel('Aligned time [s] (motion onset at {:.2f} s)'.format(onset))
    fig.suptitle('CBF off / on comparison', fontsize=13)
    return fig


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path, help='cbf.csv or the session directory')
    parser.add_argument('--debug', action='store_true', help='Compatibility flag: all diagnostic plots are now always saved')
    selection_group = parser.add_mutually_exclusive_group()
    selection_group.add_argument('--experiment', type=int, help='Select one experiment ID')
    selection_group.add_argument('--experiments', type=int, nargs='+', help='Select multiple IDs, e.g. --experiments 2 4')
    parser.add_argument('--segment', type=int)
    parser.add_argument('--start', type=float, help='Lower bound in CSV time_s seconds')
    parser.add_argument('--end', type=float, help='Upper bound in CSV time_s seconds')
    parser.add_argument('--urdf', type=Path, help='Default: robot.urdf beside CSV')
    parser.add_argument('--metadata', type=Path, help='Default: metadata.json beside CSV')
    parser.add_argument('--contacts', type=Path, help='Default: contacts.csv beside CSV')
    parser.add_argument('--arm-id', help='Joint name prefix, e.g. fr3 or panda')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--format', choices=['png', 'pdf', 'svg'], default='png')
    parser.add_argument('--output', type=Path, help='Compatibility option: use this filename as PREFIX for figure files')
    display = parser.add_mutually_exclusive_group()
    display.add_argument('--show', dest='show', action='store_true', help='Show only the comparison (default)')
    display.add_argument('--no-show', dest='show', action='store_false', help='Save all plots without opening a GUI')
    parser.set_defaults(show=True)
    parser.add_argument('--pre-motion', type=float, default=1.0, help='Seconds before detected onset (default 1)')
    parser.add_argument('--post-motion', type=float, default=2.0, help='Seconds after detected stop (default 2)')
    parser.add_argument('--motion-speed', type=float, default=.01, help='EE onset speed in m/s (default .01)')
    parser.add_argument('--stop-speed', type=float, default=.005, help='EE settling speed in m/s (default .005)')
    parser.add_argument('--settle-time', type=float, default=.5, help='Time below stop speed in seconds (default .5)')
    args = parser.parse_args(argv)
    if args.start is not None and args.end is not None and args.start > args.end:
        raise ValueError('--start must not exceed --end')
    for key in ('pre_motion', 'post_motion', 'motion_speed', 'stop_speed', 'settle_time'):
        value = getattr(args, key)
        if not np.isfinite(value) or value < 0 or (key in ('motion_speed', 'stop_speed', 'settle_time') and value == 0):
            raise ValueError('Invalid --' + key.replace('_', '-'))
    if args.stop_speed >= args.motion_speed:
        raise ValueError('--stop-speed must be smaller than --motion-speed')
    import matplotlib
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if args.csv.is_dir():
        args.csv = args.csv / 'cbf.csv'
    all_data = load_csv(args.csv)
    session = args.csv.parent
    metadata = read_json(args.metadata or session/'metadata.json')
    names, limits = joint_limits(args.urdf or session/'robot.urdf', metadata, args.arm_id)
    out = args.output_dir or session/'plots'
    extension, stem = args.format, args.csv.stem
    if args.output:
        out, stem = args.output.parent, args.output.stem
        extension = args.output.suffix.lstrip('.') or extension
        if extension not in ('png', 'pdf', 'svg'):
            raise ValueError('--output extension must be png, pdf or svg')
    out.mkdir(parents=True, exist_ok=True)
    trials = motion_trials(all_data, args)
    comparison = comparison_figure(plt, trials)
    selection_suffix = ''
    selected_ids = [args.experiment] if args.experiment is not None else args.experiments
    if selected_ids is not None:
        selection_suffix = '_exp' + '-'.join(str(x) for x in sorted(set(selected_ids)))
    if args.segment is not None:
        selection_suffix += '_segment{}'.format(args.segment)
    comparison_path = out / (stem + selection_suffix + '_comparison.' + extension)
    comparison.savefig(str(comparison_path), dpi=160)
    print('Saved ' + str(comparison_path))
    (out / (stem + selection_suffix + '_alignment.json')).write_text(json.dumps(
        {'settings': {k: getattr(args, k) for k in ('pre_motion', 'post_motion', 'motion_speed', 'stop_speed', 'settle_time')},
         'trials': [info for info, _ in trials]}, indent=2))
    count = 0
    for segment in np.unique(all_data['time_segment']):
        if args.segment is not None and segment != args.segment:
            continue
        selection = all_data['time_segment'] == segment
        first = np.flatnonzero(selection)[0]
        origin = (int(all_data['stamp_ns'][first]), all_data['time_s'][first]) if 'stamp_ns' in all_data else None
        if args.experiment is not None:
            selection &= all_data['experiment_id'] == args.experiment
        elif args.experiments is not None:
            selection &= np.isin(all_data['experiment_id'], args.experiments)
        # Determine event locations BEFORE trimming the time window.
        events = []
        for exp in np.unique(all_data['experiment_id'][selection]):
            violation = selection & (all_data['experiment_id'] == exp) & (all_data['cbf_h'] < 0) & (col(all_data, 'cbf_active') == 1)
            if violation.any():
                events.append(all_data['time_s'][np.flatnonzero(violation)[0]])
        if args.start is not None:
            selection &= all_data['time_s'] >= args.start
        if args.end is not None:
            selection &= all_data['time_s'] <= args.end
        if not selection.any():
            continue
        data = {k: v[selection] for k, v in all_data.items()}
        contacts = contact_rows(args.contacts or session/'contacts.csv', data, origin) if origin else []
        experiments = ','.join(str(int(x)) for x in np.unique(data['experiment_id']))
        title = '{} - segment {}\nExperiments: {}'.format(args.csv.name, segment, experiments)
        figures = make_figures(plt, data, names, limits, contacts, debug=True)
        decorate(figures, data, title, events)
        suffix = '_segment{}'.format(segment)
        if args.experiment is not None:
            suffix += '_exp{}'.format(args.experiment)
        elif args.experiments is not None:
            suffix += '_exp' + '-'.join(str(x) for x in sorted(set(args.experiments)))
        if args.start is not None or args.end is not None:
            suffix += '_t{}-{}'.format(args.start if args.start is not None else 'start',
                                      args.end if args.end is not None else 'end')
        for label, fig, _ in figures:
            path = out / '{}{}_{}.{}'.format(stem, suffix, label, extension)
            fig.savefig(str(path), dpi=160)
            print('Saved ' + str(path))
            plt.close(fig)  # Only the comparison remains open, even with --show.
        count += len(data['time_s'])
        missing = np.nansum(col(data, 'missing_samples_before'))
        print('{}: {} samples, {} reported missing before selected rows'.format(title, len(data['time_s']), int(missing)))
        for key in ('ee_target_distance', 'q_1', 'tau_command_1', 'kinetic_energy_total'):
            if not np.isfinite(col(data, key)).any():
                print('Unavailable in selection: ' + key)
        print('Kmax: {}; alpha: {}; direction samples: {}'.format(
            np.unique(data['Kmax']), np.unique(col(data, 'alpha')),
            np.unique(np.column_stack([col(data, 'direction_'+s) for s in 'xyz']), axis=0)))
    if count == 0:
        raise ValueError('No samples match the requested selection')
    health = read_json(session/'health.json')
    if health:
        print('Recorder health (whole session): ' + json.dumps(health, sort_keys=True))
    if args.show:
        plt.show()
        plt.close('all')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, ET.ParseError) as error:
        sys.exit('Plot failed: {}'.format(error))
