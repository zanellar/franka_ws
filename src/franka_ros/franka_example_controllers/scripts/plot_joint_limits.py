#!/usr/bin/env python3
"""Wait for joint-margin publishers, then exec rqt_plot with the URDF-derived curves."""
import json
import os
import sys


def plot_command(configuration):
    if configuration.get('schema_version') != 1:
        raise ValueError('Unsupported diagnostics configuration schema')
    topics = configuration.get('plot_topics')
    if (not isinstance(topics, list) or not topics or
            any(not isinstance(topic, str) or not topic.startswith('/') or
                not topic.endswith('/data') for topic in topics)):
        raise ValueError('Invalid plot_topics configuration')
    if len(set(topics)) != len(topics):
        raise ValueError('Duplicate plot topics')
    # --empty prevents restoration of unrelated topics from an earlier rqt session.
    return ['rosrun', 'rqt_plot', 'rqt_plot', '--empty'] + topics


def wait_for_message(rospy, topic, message_type):
    while not rospy.is_shutdown():
        try:
            return rospy.wait_for_message(topic, message_type, timeout=1.0)
        except rospy.ROSException:
            rospy.loginfo_throttle(5.0, 'Waiting for %s before opening rqt_plot', topic)
    return None


def main():
    import rospy
    from std_msgs.msg import Float64, String
    rospy.init_node('plot_joint_limits')
    try:
        namespace = rospy.resolve_name(rospy.get_param('~diagnostics_node', 'joint_limit_diagnostics'))
        message = wait_for_message(rospy, namespace.rstrip('/') + '/configuration', String)
        if message is None:
            return 0
        configuration = json.loads(message.data)
        command = plot_command(configuration)
        # The configuration is latched after all publishers are registered.
        # Wait for a state-driven zero sample as well, avoiding a clock-at-zero start.
        zero_topic = configuration.get('zero_topic')
        if not isinstance(zero_topic, str) or zero_topic + '/data' not in command[4:]:
            raise ValueError('Missing zero reference in diagnostics configuration')
        if wait_for_message(rospy, zero_topic, Float64) is None:
            return 0
        if rospy.is_shutdown():
            return 0
        rospy.loginfo('Opening rqt_plot with %d margin curves and zero', len(command) - 5)
        rospy.signal_shutdown('Handing process to rqt_plot')
        # Replace this process: roslaunch owns the GUI PID and shuts it down normally.
        # No shell, detached children or forwarding of private ROS arguments.
        os.execvp(command[0], command)
    except (ValueError, TypeError, OSError) as error:
        rospy.logerr('Cannot start joint-limit plot: %s', error)
        return 1
    except rospy.ROSInterruptException:
        return 0
    return 0


if __name__ == '__main__':
    sys.exit(main())
