#!/usr/bin/env python3
"""Plot CBF, joint limits, Jacobian and contact diagnostics (no ROS required)."""
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
                    'cbf_h', 'cbf_constraint_safe', 'kinetic_energy_dir', 'Kmax'}
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
        (np.diff(data['time_s']) <= 0)) + 1, len(data['time_s'])]
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


def make_figures(plt, data, names, limits, contacts):
    figures = []
    def new(name, rows, size=None):
        fig, axes = plt.subplots(rows, 1, sharex=True, figsize=size or (13, 3*rows),
                                 constrained_layout=True, squeeze=False)
        figures.append((name, fig, axes.ravel()))
        return axes.ravel()

    axes = new('cbf', 3)
    line(axes[0], data, data['cbf_h'], 'h = Kmax - Kdir')
    axes[0].axhline(0, color='black', ls='--', lw=.8)
    axes[0].set_ylabel('cbf_h [J]')
    line(axes[1], data, data['cbf_constraint_safe'], 'applied-command model residual')
    if 'cbf_constraint_qp' in data:
        line(axes[1], data, data['cbf_constraint_qp'], 'QP residual', ls=':')
    line(axes[1], data, -col(data, 'cbf_residual_tolerance'), '-solver tolerance', color='gray', ls='--')
    axes[1].axhline(0, color='black', ls='--', lw=.8)
    axes[1].set_ylabel('CBF residual [J/s]')
    if np.isfinite(col(data, 'ee_target_distance')).any():
        line(axes[2], data, col(data, 'ee_target_distance'), 'distance to requested target')
        if np.isfinite(col(data, 'ee_reference_distance')).any():
            line(axes[2], data, col(data, 'ee_reference_distance'), 'distance to filtered reference', ls='--')
    else:
        unavailable(axes[2], 'Target distance unavailable in this CSV.\nApply telemetry patch for future recordings.')
    axes[2].set_ylabel('EE position error [m]')

    fig, axes = plt.subplots(7, 2, sharex=True, figsize=(14, 17), constrained_layout=True)
    figures.append(('joints', fig, axes.ravel()))
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

    axes = new('energy_torque', 4)
    line(axes[0], data, data['kinetic_energy_dir'], 'Kdir')
    line(axes[0], data, data['Kmax'], 'Kmax', color='black', ls='--')
    line(axes[1], data, col(data, 'kinetic_energy_total'), 'Ktotal')
    for i in range(1, 8):
        line(axes[2], data, debug_col(data, 'tau_command_{}'.format(i)), 'tau{}'.format(i))
    line(axes[3], data, debug_col(data, 'robot_mode'), 'robot_mode', drawstyle='steps-post')
    for ax, label in zip(axes, ('Kdir [J]', 'Ktotal [J]', 'Commanded torque [Nm]', 'Robot mode')):
        ax.set_ylabel(label)
    axes[3].set_yticks(range(7))
    axes[3].set_yticklabels(['Other', 'Idle', 'Move', 'Guiding', 'Reflex', 'Stopped', 'Recovery'])
    axes[2].set_title('Controller command; not a measurement of the applied Gazebo torque')

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


def decorate(figures, data, title, events):
    for _, fig, axes in figures:
        fig.suptitle(title + '\nGrey: CBF off | red shading: task aborted | red vertical: first h < 0 with CBF on', fontsize=11)
        for ax in axes:
            for sl in slices(data):
                t = data['time_s'][sl]
                for field, predicate, color in (
                    ('cbf_active', lambda x: x == 0, 'gray'),
                    ('task_aborted', lambda x: x == 1, 'red')):
                    ax.fill_between(t, 0, 1, where=predicate(col(data, field)[sl]),
                                    transform=ax.get_xaxis_transform(), color=color, alpha=.10)
            for event in events:
                if data['time_s'][0] <= event <= data['time_s'][-1]:
                    ax.axvline(event, color='red', ls=':', lw=.8, alpha=.7)
            ax.grid(True, alpha=.25)
            handles, labels = ax.get_legend_handles_labels()
            if handles:
                ax.legend(handles, labels, loc='best', fontsize=7, ncol=min(3, len(labels)))
            ax.set_xlabel('CSV time_s [s] (simulation time since recording segment start)')
            ax.ticklabel_format(axis='x', style='plain', useOffset=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--experiment', type=int)
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
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args(argv)
    if args.start is not None and args.end is not None and args.start > args.end:
        raise ValueError('--start must not exceed --end')
    import matplotlib
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
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
    count = 0
    for segment in np.unique(all_data['time_segment']):
        if args.segment is not None and segment != args.segment:
            continue
        selection = all_data['time_segment'] == segment
        first = np.flatnonzero(selection)[0]
        origin = (int(all_data['stamp_ns'][first]), all_data['time_s'][first]) if 'stamp_ns' in all_data else None
        if args.experiment is not None:
            selection &= all_data['experiment_id'] == args.experiment
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
        title = '{} | segment {} | experiment {}'.format(args.csv.name, segment, experiments)
        figures = make_figures(plt, data, names, limits, contacts)
        decorate(figures, data, title, events)
        suffix = '_segment{}'.format(segment)
        if args.experiment is not None:
            suffix += '_exp{}'.format(args.experiment)
        if args.start is not None or args.end is not None:
            suffix += '_t{}-{}'.format(args.start if args.start is not None else 'start',
                                      args.end if args.end is not None else 'end')
        for label, fig, _ in figures:
            path = out / '{}{}_{}.{}'.format(stem, suffix, label, extension)
            fig.savefig(str(path), dpi=160)
            print('Saved ' + str(path))
            if not args.show:
                plt.close(fig)
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
