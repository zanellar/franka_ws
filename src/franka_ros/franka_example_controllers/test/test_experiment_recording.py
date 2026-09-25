#!/usr/bin/env python3
"""Offline recorder lifecycle checks; no ROS master or Gazebo is needed."""
import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

path = Path(__file__).resolve().parents[1] / 'scripts' / 'record_directional_cbf.py'
spec = importlib.util.spec_from_file_location('recorder', path)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class Stamp:
    def __init__(self, ns=100_000_000_000):
        self.ns = ns

    def to_nsec(self):
        return self.ns

    @staticmethod
    def now():
        return Stamp()


def message(index=1, ns=100_001_000_000, active=True):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=Stamp(ns), seq=index, frame_id='fr3_link0'),
        clock=Stamp(ns), sample_index=index, experiment_id=1, dt=.001,
        cbf_h=.05, cbf_constraint_safe=.005, cbf_constraint_qp=.005,
        energy_mode=1, cbf_constraint_nom=-.01, u_nom=[.4]*7, u_safe=[.3]*7,
        kinetic_energy_dir=0., svd_jacobian=[1.] * 6, Kmax=.05, alpha=.1,
        cbf_residual_tolerance=1e-5, direction=[1., 0., 0.], cbf_active=active,
        task_aborted=False, cbf_solution_applied=active, solver_status=1,
        ee_position=[.1, .2, .3], target_position=[.2, .2, .3],
        ee_target_distance=.1, ee_reference_distance=.02,
        kinetic_energy_total=.01, q=[.1]*7, dq=[.2]*7, tau_command=[.3]*7, robot_mode=2)


class CommonSchemaTests(unittest.TestCase):
    def test_ab_same_schema_and_exact_controls(self):
        for mode in (0, 1):
            for active in (False, True):
                msg = message(active=active)
                msg.energy_mode = mode
                msg.u_nom = [float(i) for i in range(7)]
                msg.u_safe = [float(i)/2 for i in range(7)]
                values = recorder.CsvRows().convert(msg)
                self.assertEqual(len(values), len(recorder.FIELDS))
                row = dict(zip(recorder.FIELDS, values))
                self.assertEqual(row['energy_mode'], mode)
                self.assertEqual(row['cbf_constraint_nom'], -.01)
                for i in range(7):
                    self.assertEqual(row['u_nom_'+str(i+1)], msg.u_nom[i])
                    self.assertEqual(row['u_safe_'+str(i+1)], msg.u_safe[i])


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp.name) / 'sessions'
        urdf = '<robot name="fr3">' + ''.join(
            '<joint name="fr3_joint%d"><limit lower="-1" upper="1"/></joint>' % i
            for i in range(1, 8)) + '</robot>'
        self.params = {'~output_dir': str(self.directory), '/robot_description': urdf,
                       '/franka_state_controller/publish_rate': 1000}
        self.shutdown = False
        modules = {}
        ros = types.ModuleType('rospy')
        ros.get_param = lambda name, default=None: self.params.get(name, default)
        ros.Time = Stamp
        ros.Subscriber = lambda *a, **kw: types.SimpleNamespace(unregister=lambda: None)
        ros.Service = lambda *a, **kw: types.SimpleNamespace(shutdown=lambda: None)
        ros.is_shutdown = lambda: self.shutdown
        ros.loginfo = ros.logerr = ros.logwarn_throttle = lambda *a, **kw: None
        modules['rospy'] = ros
        for name, classes in {
            'franka_msgs.msg': ['DirectionalCbfDiagnostics', 'FrankaState'],
            'std_msgs.msg': ['String'],
            'gazebo_msgs.msg': ['ContactsState'], 'std_srvs.srv': ['Trigger', 'TriggerResponse'],
        }.items():
            module = types.ModuleType(name)
            for cls in classes:
                setattr(module, cls, type(cls, (), {}))
            if name == 'std_srvs.srv':
                module.TriggerResponse = lambda success, message: types.SimpleNamespace(
                    success=success, message=message)
            modules[name] = module
        self.patch = patch.dict(sys.modules, modules)
        self.patch.start()
        self.instance = recorder.ExperimentRecorder()

    def tearDown(self):
        self.instance.close_files()
        self.patch.stop()
        self.temp.cleanup()

    def ready(self):
        for topic, _ in self.instance.topics:
            self.instance.receive(message(ns=99_000_000_000), topic)

    def close_session(self):
        self.shutdown = True
        self.instance.run()

    def rows(self, name):
        with (self.instance.session/name).open() as stream:
            return list(csv.DictReader(stream))

    def test_no_files_before_first_command(self):
        self.ready()
        self.assertFalse(self.directory.exists())
        self.assertTrue(self.instance.pending.empty())

    def test_missing_diagnostics_rejects_start(self):
        self.assertFalse(self.instance.start(None).success)
        self.assertFalse(self.directory.exists())

    def test_contacts_not_required_to_start_core_csv(self):
        self.instance.receive(message(), '/directional_cbf/diagnostics')
        self.assertTrue(self.instance.start(None).success)
        self.assertFalse((self.instance.session/'contacts.csv').exists())

    def test_idempotence_and_pretrigger_filter(self):
        self.ready()
        self.instance.start(None)
        session = self.instance.session
        self.instance.receive(message(ns=99_000_000_000), '/directional_cbf/diagnostics')
        self.assertTrue(self.instance.pending.empty())
        self.instance.receive(message(active=False), '/directional_cbf/diagnostics')
        self.assertTrue(self.instance.start(None).success)
        self.assertEqual(session, self.instance.session)
        self.close_session()
        row = self.rows('cbf.csv')[0]
        self.assertEqual(row['debug_valid'], '0')
        self.assertEqual(row['q_1'], '')
        self.assertEqual(row['tau_command_7'], '')
        self.assertEqual(row['ee_target_distance'], '0.1')
        self.assertEqual(row['ee_reference_distance'], '0.02')
        self.assertEqual(row['ee_position_x'], '0.1')
        self.assertEqual(row['target_position_x'], '0.2')
        self.assertEqual(row['kinetic_energy_total'], '0.01')
        self.assertFalse((session/'contacts.csv').exists())
        self.assertFalse(any(session.glob('*.bag')))

    def test_core_continues_debug_switches_on_off_on(self):
        self.ready()
        self.instance.start(None)
        for index, active in enumerate((False, True, False, True), start=1):
            self.instance.receive(message(index, 100_000_000_000+index*1000000, active),
                                  '/directional_cbf/diagnostics')
        self.close_session()
        rows = self.rows('cbf.csv')
        self.assertEqual(len(rows), 4)
        self.assertEqual([r['debug_valid'] for r in rows], ['0', '1', '0', '1'])
        self.assertEqual([r['q_7'] for r in rows], ['', '0.1', '', '0.1'])
        self.assertEqual([r['robot_mode'] for r in rows], ['', '2', '', '2'])
        health = json.loads((self.instance.session/'health.json').read_text())
        self.assertEqual(health['written']['debug_rows'], 2)
        self.assertTrue(health['clean_shutdown'])

    def test_contacts_wait_for_matching_cbf_time_and_preserve_late_active_frames(self):
        self.ready()
        self.instance.start(None)
        def diag(index, active):
            self.instance.receive(message(index, 100_000_000_000+index*1000000, active),
                                  '/directional_cbf/diagnostics')
        def contact(ns):
            self.instance.receive(types.SimpleNamespace(
                header=types.SimpleNamespace(stamp=Stamp(ns), seq=1), states=[]),
                '/directional_cbf/contacts')
        diag(1, False)
        contact(100_002_000_000)  # arrives before the corresponding active diagnostic
        diag(2, True)
        diag(3, False)
        contact(100_002_500_000)  # arrives late, but still belongs to the active interval
        contact(100_003_000_000)
        self.close_session()
        rows = self.rows('contacts.csv')
        self.assertEqual([r['stamp_ns'] for r in rows], ['100002000000', '100002500000'])
        self.assertEqual([r['cbf_age_ns'] for r in rows], ['0', '500000'])
        self.assertTrue(all(r['contact_count'] == '0' for r in rows))

    def test_future_false_contact_is_not_written_as_active(self):
        calls = []
        gate = recorder.ContactGate(lambda *args: calls.append(args))
        gate.diagnostic(message(ns=10, active=True), 0)
        gate.contact(types.SimpleNamespace(header=types.SimpleNamespace(stamp=Stamp(20))))
        self.assertEqual(calls, [])
        gate.diagnostic(message(ns=20, active=False), 0)
        self.assertEqual(calls, [])
        self.assertEqual(gate.filtered_inactive, 1)

    def test_stale_contact_is_counted_not_mislabeled(self):
        calls = []
        gate = recorder.ContactGate(lambda *args: calls.append(args), max_age_ns=5)
        gate.diagnostic(message(ns=10, active=True), 0)
        gate.contact(types.SimpleNamespace(header=types.SimpleNamespace(stamp=Stamp(20))))
        gate.drain(final=True)
        self.assertEqual(calls, [])
        self.assertEqual(gate.unmatched, 1)

    def test_contact_summary_preserves_names_depth_and_force(self):
        self.ready()
        self.instance.start(None)
        self.instance.receive(message(), '/directional_cbf/diagnostics')
        v = types.SimpleNamespace(x=3., y=4., z=0.)
        w = types.SimpleNamespace(force=v, torque=v)
        state = types.SimpleNamespace(collision1_name='fr3::link::collision',
            collision2_name='ground::collision', contact_positions=[v], depths=[.002],
            total_wrench=w, wrenches=[w])
        self.instance.receive(types.SimpleNamespace(header=message().header, states=[state]),
                              '/directional_cbf/contacts')
        self.close_session()
        row = self.rows('contacts.csv')[0]
        self.assertEqual(row['collision2'], 'ground::collision')
        self.assertEqual(row['max_point_force_norm'], '5.0')
        self.assertEqual(row['max_depth_m'], '0.002')
        self.assertNotIn(None, row)

    def test_clock_reset(self):
        self.ready()
        self.instance.start(None)
        topic = '/directional_cbf/diagnostics'
        self.instance.receive(message(), topic)
        self.instance.receive(message(0, 1000000), topic)
        self.close_session()
        self.assertEqual(self.rows('cbf.csv')[1]['time_segment'], '1')

    def test_queue_overflow_is_counted(self):
        self.ready()
        self.instance.start(None)
        self.instance.pending = recorder.queue.Queue(maxsize=1)
        self.instance.receive(message(), '/directional_cbf/diagnostics')
        self.instance.receive(message(2), '/directional_cbf/diagnostics')
        self.assertEqual(self.instance.dropped['/directional_cbf/diagnostics'], 1)


    def test_real_records_off_state_and_initialization_without_fake_contacts(self):
        self.instance.close_files()
        self.params['~runtime_environment']='real'
        self.instance=recorder.ExperimentRecorder()
        self.ready()
        self.assertTrue(self.instance.start(None).success)
        diag=message(active=False)
        for field in recorder.HARDWARE_FIELDS[:5]:
            setattr(diag, field, 1.)
        diag.debug_valid=True
        diag.tau_J_d=[.4]*7; diag.tau_J=[.5]*7; diag.tau_predicted=[.6]*7
        self.instance.receive(diag, '/directional_cbf/diagnostics')
        self.instance.receive_phase(types.SimpleNamespace(data='initializing'))
        state=types.SimpleNamespace(header=diag.header, robot_mode=2,
                                    control_command_success_rate=1.)
        class Errors:
            __slots__=('joint_reflex',)
            def __init__(self): self.joint_reflex=False
        state.current_errors=Errors(); state.last_motion_errors=Errors()
        for name, size in recorder.STATE_ARRAYS.items(): setattr(state, name, [.1]*size)
        for name in ('m_ee', 'm_load', 'm_total'): setattr(state, name, 0.)
        for name in ('F_x_Cee','I_ee','F_x_Cload','I_load','F_x_Ctotal','I_total',
                     'F_T_EE','F_T_NE','NE_T_EE','EE_T_K'): setattr(state, name, [0.])
        self.instance.receive(state, '/franka_state_controller/franka_states')
        self.instance.receive_phase(types.SimpleNamespace(data='ready'))
        self.close_session()
        row=self.rows('cbf.csv')[0]
        self.assertEqual(row['debug_valid'], '1')
        self.assertEqual(row['cbf_active'], '0')
        self.assertEqual(row['tau_J_d_7'], '0.4')
        self.assertEqual(len(self.rows('robot_state.csv')), 1)
        self.assertEqual([r['phase'] for r in self.rows('events.csv')], ['initializing','ready'])
        self.assertFalse((self.instance.session/'contacts.csv').exists())
        self.assertTrue((self.instance.session/'robot_model_state.json').exists())
        self.assertNotIn(None, row)


if __name__ == '__main__':
    unittest.main()
