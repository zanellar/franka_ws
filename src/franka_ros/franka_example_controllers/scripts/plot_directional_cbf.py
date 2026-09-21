#!/usr/bin/env python3
"""Plot a recorded directional CBF CSV without ROS or pandas."""
import argparse
import csv
from pathlib import Path
import sys

import numpy as np


def load_csv(path):
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = ['time_s', 'time_segment', 'experiment_id', 'sample_index', 'cbf_h',
                    'cbf_constraint_safe', 'kinetic_energy_dir', 'Kmax',
                    'task_aborted', 'cbf_active', 'cbf_solution_applied',
                    'missing_samples_before', 'cbf_residual_tolerance'] + [
                        'svd_jacobian_{}'.format(i) for i in range(1, 7)]
        absent = set(required) - set(reader.fieldnames or [])
        if absent:
            raise ValueError('Missing CSV columns: ' + ', '.join(sorted(absent)))
        rows = list(reader)
    if not rows:
        raise ValueError('CSV contains no samples')
    try:
        data = {key: np.array([float(row[key]) for row in rows]) for key in required}
    except (TypeError, ValueError) as error:
        raise ValueError('Invalid or incomplete CSV row: {}'.format(error))
    if not np.isfinite(data['time_s']).all():
        raise ValueError('Nonfinite time values')
    return data


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path)
    parser.add_argument('--output', type=Path, help='PNG/PDF/SVG path (default: beside CSV)')
    parser.add_argument('--experiment', type=int, help='Select a controller experiment_id')
    parser.add_argument('--segment', type=int, help='Select a time segment after clock reset')
    parser.add_argument('--show', action='store_true', help='Also open an interactive window')
    args = parser.parse_args(argv)
    import matplotlib
    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    data = load_csv(args.csv)
    selection = np.ones(len(data['time_s']), dtype=bool)
    if args.experiment is not None:
        selection &= data['experiment_id'] == args.experiment
    if args.segment is not None:
        selection &= data['time_segment'] == args.segment
    if not selection.any():
        raise ValueError('No samples match the requested experiment/segment')
    data = {key: value[selection] for key, value in data.items()}
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True, constrained_layout=True)
    colours = plt.get_cmap('tab10')
    # Never join different attempts, clock resets or known missing samples.
    boundaries = np.r_[0, np.flatnonzero(
        (np.diff(data['time_segment']) != 0) | (np.diff(data['experiment_id']) != 0) |
        (np.diff(data['sample_index']) != 1)) + 1, len(data['time_s'])]
    for first, last in zip(boundaries[:-1], boundaries[1:]):
        sl = slice(first, last)
        t = data['time_s'][sl]
        experiment = int(data['experiment_id'][first])
        segment = int(data['time_segment'][first])
        colour = colours(experiment % 10)
        label = 'segment {} / experiment {}'.format(segment, experiment)
        axes[0].plot(t, data['cbf_h'][sl], color=colour, label=label, marker='.', markersize=1)
        axes[1].plot(t, data['cbf_constraint_safe'][sl], color=colour, marker='.', markersize=1)
        axes[1].plot(t, -data['cbf_residual_tolerance'][sl], color='gray', linestyle=':',
                     label='negative tolerance')
        axes[2].plot(t, data['kinetic_energy_dir'][sl], color=colour, marker='.', markersize=1)
        axes[2].plot(t, data['Kmax'][sl], color='black', linestyle='--', label='Kmax')
        for j in range(1, 7):
            axes[3].plot(t, data['svd_jacobian_{}'.format(j)][sl], color=colours(j - 1),
                         label=r'$\sigma_{}$'.format(j), marker='.', markersize=1)
        aborted = data['task_aborted'][sl] != 0
        bypass = (data['cbf_active'][sl] == 0) & ~aborted
        for ax in axes:
            ax.fill_between(t, 0, 1, where=aborted, transform=ax.get_xaxis_transform(),
                            color='red', alpha=0.10, label='task aborted')
            ax.fill_between(t, 0, 1, where=bypass, transform=ax.get_xaxis_transform(),
                            color='gray', alpha=0.10, label='CBF disabled')
    axes[0].axhline(0, color='black', linestyle='--', linewidth=0.8)
    axes[1].axhline(0, color='black', linestyle='--', linewidth=0.8)
    axes[0].set_ylabel('cbf_h [J]')
    axes[1].set_ylabel('cbf_constraint_safe [J/s]')
    axes[2].set_ylabel('kinetic_energy_dir [J]')
    axes[3].set_ylabel('svd_jacobian (full 6 x 7 J)')
    axes[3].set_xlabel('Simulation time since segment start [s]')
    for ax in axes:
        ax.grid(True, alpha=0.25)
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        if unique:
            ax.legend(unique.values(), unique.keys(), loc='best', fontsize=8, ncol=3)
    fig.suptitle(args.csv.name)
    suffix = ''
    if args.segment is not None: suffix += '_segment{}'.format(args.segment)
    if args.experiment is not None: suffix += '_experiment{}'.format(args.experiment)
    output = args.output or args.csv.with_name(args.csv.stem + suffix + '.png')
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    print('Saved {}'.format(output))
    print('Samples: {}; missing samples between recorded rows: {}'.format(
        len(data['time_s']), int(data['missing_samples_before'].sum())))
    if np.unique(data['time_segment']).size > 1:
        print('Multiple clock segments overlap on the time axis; use --segment to isolate one.')
    if args.show: plt.show()
    plt.close(fig)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError) as error:
        sys.exit('Plot failed: {}'.format(error))
