#!/usr/bin/env python3
"""Offline checks of timestamp precision and diagnostic interval selection."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest
import numpy as np

path = Path(__file__).resolve().parents[1] / 'scripts' / 'plot_directional_cbf.py'
spec = importlib.util.spec_from_file_location('plot_cbf', path)
plot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plot)


class PlotTests(unittest.TestCase):
    def data(self):
        n = 5
        data = {k: np.zeros(n) for k in (
            'experiment_id', 'time_segment', 'task_aborted', 'direction_y', 'direction_z')}
        data.update({k: np.ones(n) for k in ('alpha', 'Kmax', 'cbf_active', 'direction_x')})
        data.update(time_s=np.arange(n)*.001, sample_index=np.arange(n),
                    experiment_id=np.zeros(n, dtype=np.int64),
                    time_segment=np.zeros(n, dtype=np.int64),
                    stamp_ns=np.int64(1789982098000000000)+np.arange(n)*1000000,
                    cbf_h=np.arange(n)*.002, cbf_constraint_safe=np.arange(n)*.002+2)
        return data

    def test_nan_gaps_and_changed_parameters(self):
        data = self.data()
        observed, model = plot.derivative_check(data)
        np.testing.assert_allclose(observed[:-1], 2)
        np.testing.assert_allclose(model[:-1], 2)
        self.assertTrue(np.isnan(observed[-1]))
        data['sample_index'][2:] += 1
        data['Kmax'][3:] = 2
        data['task_aborted'][4] = 1
        observed, _ = plot.derivative_check(data)
        self.assertEqual(observed[0], 2)
        self.assertTrue(np.isnan(observed[1:]).all())
        self.assertEqual(len(plot.slices(data)), 2)

    def test_int64_csv_and_missing_optional_fields(self):
        data = self.data()
        data['kinetic_energy_dir'] = data['Kmax']-data['cbf_h']
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'cbf.csv'
            with path.open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(data))
                writer.writeheader()
                for i in range(5):
                    writer.writerow({k: v[i] for k, v in data.items()})
            loaded = plot.load_csv(path)
        self.assertEqual(loaded['stamp_ns'].dtype, np.int64)
        np.testing.assert_array_equal(loaded['stamp_ns'], data['stamp_ns'])
        self.assertTrue(np.isnan(plot.col(loaded, 'ee_target_distance')).all())

    def test_urdf_precedence_and_contact_time_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            urdf = root/'robot.urdf'
            urdf.write_text('<robot>' + ''.join(
                '<joint name="fr3_joint%d"><limit lower="-2" upper="3"/></joint>' % i
                for i in range(1, 8)) + '</robot>')
            names, limits = plot.joint_limits(urdf, {'joint_limits_rad': {'fr3_joint1': {'lower': 0, 'upper': 1}}})
            self.assertEqual(names[5], 'fr3_joint6')
            self.assertEqual(limits[names[0]]['lower'], -2)
            data = self.data()
            contacts = root/'contacts.csv'
            contacts.write_text('stamp_ns,time_segment,experiment_id\n{},0,0\n{},1,0\n'.format(
                int(data['stamp_ns'][2]), int(data['stamp_ns'][2])))
            rows = plot.contact_rows(contacts, data, (int(data['stamp_ns'][0]), 0))
            self.assertEqual(len(rows), 1)
            self.assertAlmostEqual(rows[0]['t'], .002)


if __name__ == '__main__':
    unittest.main()
