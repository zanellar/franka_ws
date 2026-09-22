#!/usr/bin/env python3
"""Offline diagnostics/GUI handoff checks; ROS, Gazebo and Qt are not required."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

SCRIPTS = Path(__file__).resolve().parents[1]/'scripts'


def load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS/(name+'.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


diag = load('joint_limit_diagnostics')
plot = load('plot_joint_limits')
NAMES = ['fr3_joint{}'.format(i) for i in range(1, 8)]
BOUNDS = [(-2.3093, 2.3093), (-1.5133, 1.5133), (-2.4937, 2.4937),
          (-2.7478, -.4461), (-2.48, 2.48), (.8521, 4.2094), (-2.6895, 2.6895)]
URDF = '<robot name="test">' + ''.join(
    '<joint name="{}" type="revolute"><limit lower="{}" upper="{}"/>'
    '<safety_controller soft_lower_limit="0" soft_upper_limit="0"/></joint>'.format(name, *bounds)
    for name, bounds in zip(NAMES, BOUNDS)) + '</robot>'


class Message:
    def __init__(self, data=None):
        self.data = data


class FakeRos(types.ModuleType):
    class ROSException(Exception):
        pass
    class ROSInterruptException(ROSException):
        pass

    def __init__(self):
        super().__init__('rospy')
        self.params = {'/robot_description': URDF}
        self.sent = []
        self.publishers = {}
        self.stopped = False
        self.loginfo = self.logwarn_throttle = self.loginfo_throttle = self.logerr = lambda *a: None
        self.get_param = lambda key, default=None: self.params.get(key, default)
        self.has_param = lambda key: key in self.params
        self.is_shutdown = lambda: self.stopped
        self.resolve_name = lambda name: '/joint_limit_diagnostics/' + name[1:] if name.startswith('~') else '/' + name.lstrip('/')
        self.init_node = lambda name: None

    def signal_shutdown(self, reason):
        self.stopped = True

    def Publisher(self, topic, cls, **kwargs):
        topic = self.resolve_name(topic)
        publisher = types.SimpleNamespace(topic=topic, kwargs=kwargs,
            publish=lambda msg: self.sent.append((topic, msg.data)))
        self.publishers[topic] = publisher
        return publisher

    def Subscriber(self, topic, cls, callback, **kwargs):
        self.callback = callback
        return types.SimpleNamespace(topic=topic, kwargs=kwargs)


def modules(ros):
    sensor = types.ModuleType('sensor_msgs.msg'); sensor.JointState = Message
    std = types.ModuleType('std_msgs.msg'); std.Float64 = std.String = Message
    return {'rospy': ros, 'sensor_msgs.msg': sensor, 'std_msgs.msg': std}


class MarginTests(unittest.TestCase):
    def setUp(self):
        self.curves = diag.build_curves(URDF, NAMES)

    def test_actual_fr3_layout_and_signed_margins(self):
        self.assertEqual(len(self.curves), 9)
        self.assertEqual([c['kind'] for c in self.curves if c['joint'] == NAMES[5]],
                         ['lower_margin', 'upper_margin'])
        q = [0, 0, 0, -1, 0, .8500707118, 0]
        result = diag.calculate_margins(self.curves, NAMES, q)
        self.assertAlmostEqual(result[0], 2.3093)
        self.assertAlmostEqual(result[6], -.0020292882)
        self.assertGreater(result[7], 0)
        # Exact limits, then violations on both sides of a symmetric joint.
        for q1, expected in [(2.3093, 0), (-2.3093, 0), (2.4, -.0907), (-2.4, -.0907)]:
            q[0] = q1
            self.assertAlmostEqual(diag.calculate_margins(self.curves, NAMES, q)[0], expected)
        q[5] = BOUNDS[5][1] + .02
        self.assertAlmostEqual(diag.calculate_margins(self.curves, NAMES, q)[7], -.02)

    def test_names_reordered_and_extra_finger_ignored(self):
        q = [0, 0, 0, -1, 0, 1, 0]
        expected = diag.calculate_margins(self.curves, NAMES, q)
        actual = diag.calculate_margins(self.curves, ['finger'] + NAMES[::-1], [99] + q[::-1])
        self.assertEqual(actual, expected)

    def test_missing_nonfinite_and_malformed_input_never_become_zero(self):
        partial = diag.calculate_margins(self.curves, [NAMES[0]], [0])
        self.assertEqual(partial[0], 2.3093)
        self.assertTrue(all(math.isnan(v) for v in partial[1:]))
        for names, q in [(NAMES, [0]), ([NAMES[0]]*7, [0]*7), (NAMES, [float('nan')]*7),
                         (NAMES, [float('inf')]*7)]:
            self.assertTrue(all(math.isnan(v) for v in diag.calculate_margins(self.curves, names, q)))

    def test_invalid_urdf_and_names_fail_explicitly(self):
        for urdf in (URDF.replace('lower="0.8521"', 'lower="8"'),
                     URDF.replace('type="revolute"', 'type="continuous"'),
                     URDF.replace('upper="4.2094"', 'upper="nan"'), '<robot/>'):
            with self.assertRaises(ValueError):
                diag.build_curves(urdf, NAMES)
        for names in ([], NAMES+[NAMES[0]], 'fr3_joint1', ['bad/name']):
            with self.assertRaises(ValueError):
                diag.build_curves(URDF, names)
        with self.assertRaises(ValueError):
            diag.build_curves(URDF, NAMES, -1)

    def test_symmetry_classification_and_exact_bound_preservation(self):
        urdf = '<robot><joint name="j" type="revolute"><limit lower="-2" upper="2.0000000005"/></joint></robot>'
        curves = diag.build_curves(urdf, ['j'])
        self.assertEqual(len(curves), 1)
        self.assertEqual(diag.calculate_margins(curves, ['j'], [-2])[0], 0)
        self.assertEqual(diag.calculate_margins(curves, ['j'], [2.0000000005])[0], 0)
        self.assertEqual(len(diag.build_curves(urdf, ['j'], 0)), 2)

    def test_callback_publishes_every_state_without_cbf_or_service(self):
        ros = FakeRos()
        with patch.dict(sys.modules, modules(ros)):
            node = diag.JointLimitDiagnostics(ros)
            self.assertEqual(len(ros.sent), 1)  # Only latched configuration before state.
            config = json.loads(ros.sent[0][1])
            self.assertEqual(len(config['plot_topics']), 10)
            self.assertTrue(ros.publishers['/joint_limit_diagnostics/configuration'].kwargs['latch'])
            node.receive(types.SimpleNamespace(name=NAMES, position=[0, 0, 0, -1, 0, 1, 0]))
            self.assertEqual(len(ros.sent), 11)
            self.assertEqual(ros.sent[-1], ('/joint_limit_diagnostics/zero', 0))
            node.receive(types.SimpleNamespace(name=[], position=[]))
            self.assertTrue(all(math.isnan(v) for _, v in ros.sent[-10:-1]))

    def test_plot_handoff_waits_and_uses_only_announced_topics(self):
        ros = FakeRos()
        curves = [{'topic': '/joint_limit_diagnostics/' + c['topic']} for c in self.curves]
        config = dict(schema_version=1, zero_topic='/joint_limit_diagnostics/zero',
            plot_topics=[c['topic']+'/data' for c in curves]+['/joint_limit_diagnostics/zero/data'])
        calls = []
        def wait(topic, cls, timeout):
            calls.append(topic)
            if len(calls) == 1:
                raise ros.ROSException('Not available yet')
            return Message(json.dumps(config)) if topic.endswith('/configuration') else Message(0)
        ros.wait_for_message = wait
        with patch.dict(sys.modules, modules(ros)), patch.object(plot.os, 'execvp') as execute:
            self.assertEqual(plot.main(), 0)
        self.assertTrue(ros.stopped)
        self.assertEqual(calls, ['/joint_limit_diagnostics/configuration']*2+['/joint_limit_diagnostics/zero'])
        execute.assert_called_once_with('rosrun', ['rosrun', 'rqt_plot', 'rqt_plot', '--empty']+config['plot_topics'])

    def test_plot_rejects_invalid_configuration(self):
        for config in ({}, {'schema_version': 2}, {'schema_version': 1, 'plot_topics': ['--help']},
                       {'schema_version': 1, 'plot_topics': ['/a/data', '/a/data']}):
            with self.assertRaises(ValueError):
                plot.plot_command(config)

    def test_launch_wiring_and_optional_gui(self):
        root = SCRIPTS.parent
        launch = ET.parse(root/'launch/joint_limit_diagnostics.launch').getroot()
        args = {a.get('name'): a.get('default') for a in launch.findall('arg')}
        self.assertEqual(args['joint_limit_diagnostics'], 'true')
        self.assertEqual(args['plot_joint_limits'], 'false')
        nodes = {node.get('name'): node for node in launch.findall('node')}
        for node in nodes.values():
            self.assertEqual(node.get('required'), 'false')
            self.assertTrue((SCRIPTS/node.get('type')).is_file())
        self.assertEqual(nodes['plot_joint_limits'].get('respawn'), 'false')
        for diagnostic, gui in [(False, False), (True, False), (False, True), (True, True)]:
            expression = nodes['joint_limit_diagnostics'].get('if')[7:-1]
            values = {'joint_limit_diagnostics': diagnostic, 'plot_joint_limits': gui}
            self.assertEqual(eval(expression, {'arg': values.__getitem__}), diagnostic or gui)


if __name__ == '__main__':
    unittest.main()
