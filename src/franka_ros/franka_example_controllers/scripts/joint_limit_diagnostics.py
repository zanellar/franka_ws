#!/usr/bin/env python3
"""Publish signed URDF joint-position margins for live rqt_plot inspection."""
import json
import math
import re
import sys
import time
import xml.etree.ElementTree as ET


def build_curves(urdf, joint_names, symmetry_tolerance=1e-9):
    """Use hard position limits, not safety_controller soft limits."""
    if not math.isfinite(symmetry_tolerance) or symmetry_tolerance < 0:
        raise ValueError('symmetry_tolerance must be finite and nonnegative')
    if not isinstance(joint_names, list) or not joint_names:
        raise ValueError('joint_names must be a nonempty list')
    if any(not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name)
           for name in joint_names):
        raise ValueError('joint_names must be valid simple ROS topic identifiers')
    if len(set(joint_names)) != len(joint_names):
        raise ValueError('joint_names contains duplicates')
    root = ET.fromstring(urdf)
    if root.tag != 'robot':
        raise ValueError('robot_description must contain a <robot> root')
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    curves = []
    for name in joint_names:
        joint = joints.get(name)
        if joint is None:
            raise ValueError('URDF joint not found: ' + name)
        if joint.get('type') != 'revolute':
            raise ValueError('Expected a bounded revolute arm joint: ' + name)
        limit = joint.find('limit')
        if limit is None or not {'lower', 'upper'} <= set(limit.attrib):
            raise ValueError('Missing position limits for ' + name)
        lower, upper = float(limit.get('lower')), float(limit.get('upper'))
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError('Invalid position limits for ' + name)
        symmetric = math.isclose(lower, -upper, rel_tol=0, abs_tol=symmetry_tolerance)
        for kind in (('margin',) if symmetric else ('lower_margin', 'upper_margin')):
            curves.append(dict(joint=name, kind=kind, lower=lower, upper=upper,
                               topic=name + '/' + kind))
    return curves


def calculate_margins(curves, names, positions):
    """Name-based mapping; missing/invalid input is NaN, never a retained value."""
    if len(names) != len(positions) or len(set(names)) != len(names):
        return [float('nan')] * len(curves)
    q_by_name = dict(zip(names, positions))
    result = []
    for curve in curves:
        q = q_by_name.get(curve['joint'], float('nan'))
        if not math.isfinite(q):
            result.append(float('nan'))
            continue
        lower_margin = q - curve['lower']
        upper_margin = curve['upper'] - q
        if curve['kind'] == 'lower_margin':
            result.append(lower_margin)
        elif curve['kind'] == 'upper_margin':
            result.append(upper_margin)
        else:
            # Equals L-abs(q) for exact symmetry; preserves actual URDF bounds
            # when symmetry was accepted within numerical tolerance.
            result.append(min(lower_margin, upper_margin))
    return result


class JointLimitDiagnostics:
    def __init__(self, rospy):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64, String
        self.ros = rospy
        self.scalar_type = Float64
        arm = str(rospy.get_param('~arm_id', 'fr3'))
        names = rospy.get_param('~joint_names', ['{}_joint{}'.format(arm, i) for i in range(1, 8)])
        description = rospy.get_param('~robot_description_param', '/robot_description')
        # Wall time keeps startup interruptible even with /use_sim_time and a paused clock.
        while not rospy.is_shutdown() and not rospy.has_param(description):
            rospy.logwarn_throttle(5.0, 'Waiting for URDF parameter %s', description)
            time.sleep(.1)
        if rospy.is_shutdown():
            return
        self.curves = build_curves(rospy.get_param(description), names,
                                   float(rospy.get_param('~symmetry_tolerance', 1e-9)))
        self.publishers = []
        for curve in self.curves:
            topic = rospy.resolve_name('~' + curve['topic'])
            curve['topic'] = topic
            self.publishers.append(rospy.Publisher(topic, Float64, queue_size=10))
        self.zero_topic = rospy.resolve_name('~zero')
        self.zero = rospy.Publisher(self.zero_topic, Float64, queue_size=10)
        self.configuration = rospy.Publisher('~configuration', String, queue_size=1, latch=True)
        joint_topic = rospy.get_param('~joint_states_topic', '/franka_state_controller/joint_states')
        configuration = dict(schema_version=1, units='rad', curves=self.curves,
                             joint_states_topic=rospy.resolve_name(joint_topic),
                             zero_topic=self.zero_topic,
                             plot_topics=[curve['topic'] + '/data' for curve in self.curves] +
                                         [self.zero_topic + '/data'])
        self.configuration.publish(String(data=json.dumps(configuration, sort_keys=True)))
        for curve in self.curves:
            rospy.loginfo('%s: %s, limits [%g, %g] rad', curve['topic'], curve['kind'],
                          curve['lower'], curve['upper'])
        self.subscriber = rospy.Subscriber(joint_topic, JointState, self.receive,
                                           queue_size=100, tcp_nodelay=True)
        rospy.loginfo('Joint margins ready: %d curves + zero; input %s. '
                      'Positive=inside, zero=limit, negative=outside.',
                      len(self.curves), joint_topic)

    def receive(self, message):
        values = calculate_margins(self.curves, message.name, message.position)
        if not all(math.isfinite(value) for value in values):
            self.ros.logwarn_throttle(2.0, 'Missing, duplicate or invalid joint state: '
                                     'affected margins published as NaN')
        for publisher, value in zip(self.publishers, values):
            publisher.publish(self.scalar_type(data=value))
        # Published for each incoming state. No timer repeats stale joint values.
        self.zero.publish(self.scalar_type(data=0.0))


def main():
    import rospy
    rospy.init_node('joint_limit_diagnostics')
    try:
        node = JointLimitDiagnostics(rospy)
        rospy.spin()
        return 0
    except (ValueError, KeyError, TypeError, ET.ParseError) as error:
        rospy.logfatal('Joint-limit diagnostics configuration error: %s', error)
        return 1
    except rospy.ROSInterruptException:
        return 0


if __name__ == '__main__':
    sys.exit(main())
