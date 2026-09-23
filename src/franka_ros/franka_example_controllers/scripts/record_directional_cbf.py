#!/usr/bin/env python3
"""Triggered CBF CSV; hardware state remains recorded across controller switches."""
import csv
import math
from bisect import bisect_right
from collections import deque
import json
import threading
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
import queue
import time
import uuid

FIELDS = [
    'stamp_ns', 'time_s', 'time_segment', 'sample_index', 'experiment_id', 'dt',
    'cbf_h', 'cbf_constraint_safe', 'cbf_constraint_qp', 'kinetic_energy_dir',
] + ['svd_jacobian_{}'.format(i) for i in range(1, 7)] + [
    'Kmax', 'alpha', 'cbf_residual_tolerance', 'direction_x', 'direction_y', 'direction_z',
    'cbf_active', 'task_aborted', 'cbf_solution_applied', 'solver_status',
    'missing_samples_before', 'frame_id', 'kinetic_energy_total',
    'ee_position_x', 'ee_position_y', 'ee_position_z',
    'target_position_x', 'target_position_y', 'target_position_z',
    'ee_target_distance', 'ee_reference_distance',
]


class CsvRows:
    """Segment time rewinds/controller restarts, preserving original integer time."""
    def __init__(self):
        self.origin_ns = None
        self.previous_ns = None
        self.previous_index = None
        self.segment = 0
        self.last_missing = 0

    def convert(self, msg):
        ns = msg.header.stamp.to_nsec()
        index = int(msg.sample_index)
        restarted = self.previous_ns is not None and (
            ns < self.previous_ns or index <= self.previous_index)
        if restarted:
            self.segment += 1
            self.origin_ns = None
        if self.origin_ns is None:
            self.origin_ns = ns
        missing = 0 if self.previous_index is None or restarted else max(
            0, index - self.previous_index - 1)
        self.last_missing = missing
        self.previous_ns, self.previous_index = ns, index
        return [ns, (ns - self.origin_ns) * 1e-9, self.segment, index,
                msg.experiment_id, msg.dt, msg.cbf_h, msg.cbf_constraint_safe,
                msg.cbf_constraint_qp, msg.kinetic_energy_dir] + list(msg.svd_jacobian) + [
                    msg.Kmax, msg.alpha, msg.cbf_residual_tolerance,
                ] + list(msg.direction) + [
                    int(msg.cbf_active), int(msg.task_aborted), int(msg.cbf_solution_applied),
                    msg.solver_status, missing, msg.header.frame_id, msg.kinetic_energy_total,
                ] + list(msg.ee_position) + list(msg.target_position) + [
                    msg.ee_target_distance, msg.ee_reference_distance,
                ]



DEBUG_FIELDS = ['debug_valid', 'robot_mode'] + [
    '{}_{}'.format(name, i) for name in ('q', 'dq', 'tau_command') for i in range(1, 8)
]
CONTACT_FIELDS = [
    'stamp_ns', 'time_segment', 'experiment_id', 'contact_seq',
    'cbf_sample_index', 'cbf_stamp_ns', 'cbf_age_ns', 'contact_count', 'contact_index',
    'collision1', 'collision2', 'point_count', 'max_depth_m',
    'force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z',
    'max_point_force_norm',
]


HARDWARE_FIELDS = ['runtime_environment', 'qp_time_us', 'update_time_us',
                   'torque_prediction_error', 'control_command_success_rate'] + [
    '{}_{}'.format(name, i) for name in ('tau_J_d', 'tau_J', 'tau_predicted') for i in range(1, 8)]
STATE_ARRAYS = {'q': 7, 'dq': 7, 'tau_J': 7, 'tau_J_d': 7, 'tau_ext_hat_filtered': 7,
                'O_F_ext_hat_K': 6, 'joint_contact': 7, 'joint_collision': 7,
                'cartesian_contact': 6, 'cartesian_collision': 6, 'O_T_EE': 16}
STATE_FIELDS = ['stamp_ns', 'received_ros_ns', 'robot_mode', 'control_command_success_rate',
                'current_errors', 'last_motion_errors'] + [
    '{}_{}'.format(name, i) for name, size in STATE_ARRAYS.items() for i in range(1, size+1)]


def error_names(errors):
    return ';'.join(name for name in errors.__slots__ if getattr(errors, name))


def hardware_row(msg):
    return [getattr(msg, field) for field in HARDWARE_FIELDS[:5]] + list(msg.tau_J_d) + list(msg.tau_J) + list(msg.tau_predicted)


class ContactGate:
    """Associate by simulation time, never by callback arrival order.

    Contacts ahead of the CBF watermark wait in memory. A bounded history
    and explicit discard counters make transport gaps visible.
    """
    def __init__(self, write, max_age_ns=5000000, capacity=10000):
        self.write = write
        self.max_age_ns = max_age_ns
        self.capacity = capacity
        self.stamps = []
        self.labels = []
        self.pending = deque()
        self.unmatched = 0
        self.overflow = 0
        self.filtered_inactive = 0
        self.last_contact_ns = None

    def diagnostic(self, msg, segment):
        ns = msg.header.stamp.to_nsec()
        if self.stamps and (segment != self.labels[-1][0] or ns < self.stamps[-1]):
            self.unmatched += len(self.pending)
            self.pending.clear()
            self.stamps.clear()
            self.labels.clear()
        self.stamps.append(ns)
        self.labels.append((segment, msg.experiment_id, bool(msg.cbf_active), msg.sample_index))
        self.drain()
        if len(self.stamps) > 4096:
            del self.stamps[:2048]
            del self.labels[:2048]

    def contact(self, msg):
        ns = msg.header.stamp.to_nsec()
        if self.last_contact_ns is not None and ns < self.last_contact_ns:
            # Cross-stream ordering during a clock reset cannot be inferred.
            self.unmatched += len(self.pending) + 1
            self.pending.clear()
            self.stamps.clear()
            self.labels.clear()
            self.last_contact_ns = ns
            return
        self.last_contact_ns = ns
        if len(self.pending) >= self.capacity:
            self.pending.popleft()
            self.overflow += 1
        self.pending.append(msg)
        self.drain()

    def drain(self, final=False):
        while self.pending and self.stamps:
            msg = self.pending[0]
            ns = msg.header.stamp.to_nsec()
            if ns > self.stamps[-1] and not final:
                break
            self.pending.popleft()
            index = bisect_right(self.stamps, ns) - 1
            if index < 0 or ns - self.stamps[index] > self.max_age_ns:
                self.unmatched += 1
                continue
            label = self.labels[index]
            if label[2]:
                self.write(msg, label, self.stamps[index])
            else:
                self.filtered_inactive += 1
        if final:
            self.unmatched += len(self.pending)
            self.pending.clear()


class ExperimentRecorder:
    """Core CSV from first service request; debug only for actual CBF-active samples."""
    def __init__(self):
        import rospy
        from franka_msgs.msg import DirectionalCbfDiagnostics
        from std_srvs.srv import Trigger, TriggerResponse
        self.ros = rospy
        environment=rospy.get_param('~runtime_environment', 'gazebo')
        if environment not in ('gazebo', 'real'):
            raise ValueError('runtime_environment must be gazebo or real')
        self.real_robot = environment == 'real'
        self.state_stream = self.event_stream = None
        self.state_sequence_gaps = 0
        self.previous_state_seq = None
        self.have_robot_metadata = False
        self.response_type = TriggerResponse
        self.lock = threading.RLock()
        capacity = int(rospy.get_param('~queue_capacity', 20000))
        if capacity <= 0:
            raise ValueError('queue_capacity must be positive')
        self.pending = queue.Queue(maxsize=capacity)
        self.output_dir = Path(rospy.get_param('~output_dir', '~/.ros/directional_cbf')).expanduser()
        self.arm_id = rospy.get_param('~arm_id', 'fr3')
        self.active = False
        self.stopping = False
        self.error = ''
        self.session = None
        self.stream = self.contact_stream = None
        self.start_ns = None
        self.admitted = set()
        self.seen = {}
        self.written = {'cbf_rows': 0, 'debug_rows': 0, 'contact_frames': 0, 'contact_rows': 0}
        self.dropped = {}
        self.cbf_missing = 0
        self.contact_sequence_gaps = 0
        self.previous_contact_seq = None
        self.converter = CsvRows()
        self.last_cbf_active = False
        self.gate = ContactGate(self.write_contacts)
        self.topics = [('/directional_cbf/diagnostics', DirectionalCbfDiagnostics)]
        if self.real_robot:
            from franka_msgs.msg import FrankaState
            self.topics.append(('/franka_state_controller/franka_states', FrankaState))
        else:
            from gazebo_msgs.msg import ContactsState
            self.topics.append(('/directional_cbf/contacts', ContactsState))
        self.subscribers = [rospy.Subscriber(
            topic, msg_type, self.receive, callback_args=topic,
            queue_size=5000, buff_size=8 * 1024 * 1024, tcp_nodelay=True)
            for topic, msg_type in self.topics]
        if self.real_robot:
            from std_msgs.msg import String
            self.subscribers.append(rospy.Subscriber('/directional_cbf/phase', String,
                self.receive_phase, queue_size=100, tcp_nodelay=True))
        self.service = rospy.Service('~start', Trigger, self.start)
        rospy.loginfo('CSV recorder armed: waiting for the first set_experiment_command.')

    def receive_phase(self, msg):
        # Events have reception stamps, not a claimed synchronous controller stamp.
        with self.lock:
            if not self.active or self.stopping:
                return
            try:
                self.pending.put_nowait(('/directional_cbf/phase',
                    (self.ros.Time.now().to_nsec(), time.monotonic_ns(), msg.data)))
            except queue.Full:
                topic='/directional_cbf/phase'
                self.dropped[topic]=self.dropped.get(topic, 0)+1

    def receive(self, msg, topic):
        with self.lock:
            self.seen[topic] = time.monotonic()
            if not self.active or self.stopping:
                return
            if topic not in self.admitted:
                if msg.header.stamp.to_nsec() < self.start_ns:
                    return
                self.admitted.add(topic)
            try:
                payload=(msg, self.ros.Time.now().to_nsec()) if topic == '/franka_state_controller/franka_states' else msg
                self.pending.put_nowait((topic, payload))
            except queue.Full:
                self.dropped[topic] = self.dropped.get(topic, 0) + 1
                self.ros.logwarn_throttle(2.0, 'CSV queue full: samples lost; see health.json')

    def start(self, _request):
        with self.lock:
            if self.stopping or self.error:
                return self.response_type(False, self.error or 'Recorder is stopping')
            if self.active:
                return self.response_type(True, 'Already recording: ' + str(self.session))
            if time.monotonic() - self.seen.get('/directional_cbf/diagnostics', -float('inf')) > 2.0:
                return self.response_type(False, 'No recent CBF diagnostics. Check running controller and clock.')
            try:
                urdf = self.ros.get_param('/robot_description')
                tree = ET.fromstring(urdf)
                limits = {}
                for joint in tree.findall('joint'):
                    limit = joint.find('limit')
                    if limit is not None and 'lower' in limit.attrib and 'upper' in limit.attrib:
                        limits[joint.attrib['name']] = {
                            'lower': float(limit.attrib['lower']), 'upper': float(limit.attrib['upper'])}
                joints = [self.arm_id + '_joint' + str(i) for i in range(1, 8)]
                if any(name not in limits for name in joints):
                    raise ValueError('Missing joint limits in /robot_description')
                name = 'experiment_{}_{}'.format(
                    datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'), uuid.uuid4().hex[:8])
                self.session = self.output_dir / name
                self.session.mkdir(parents=True, exist_ok=False)
                (self.session / 'robot.urdf').write_text(urdf)
                self.start_ns = self.ros.Time.now().to_nsec()
                metadata = {
                    'start_stamp_ns': self.start_ns, 'joint_names': joints,
                    'joint_limits_rad': limits, 'core_fields': FIELDS,
                    'debug_fields': DEBUG_FIELDS, 'contact_fields': CONTACT_FIELDS,
                    'contact_max_cbf_age_ns': self.gate.max_age_ns,
                    'robot_mode_encoding': 'libfranka RobotMode: Other=0 Idle=1 Move=2 Guiding=3 '
                                           'Reflex=4 UserStopped=5 AutomaticErrorRecovery=6',
                    'note': 'No rosbag. CBF fields are always recorded after the trigger. '
                            'Joint debug fields are empty unless cbf_active=true. '
                            'Joint state and commanded torque share the CBF sample. '
                            'Contact labels use the latest available CBF stamp at or before '
                            'the contact stamp; cbf_age_ns exposes non-exact matches. '
                            'Wrenches are copied from Gazebo body_1_wrench values.',
                }
                (self.session / 'metadata.json').write_text(json.dumps(metadata, indent=2))
                self.stream = (self.session / 'cbf.csv').open('x', newline='')
                self.writer = csv.writer(self.stream)
                self.writer.writerow(FIELDS + DEBUG_FIELDS + (HARDWARE_FIELDS if self.real_robot else []))
                if self.real_robot:
                    metadata.update({
                        'runtime_environment': 'real', 'hardware_fields': HARDWARE_FIELDS,
                        'global_cbf_parameters': {key: self.ros.get_param('/'+key, None) for key in
                            ('direction_x', 'direction_y', 'direction_z', 'damping_ratio', 'mobility_epsilon')},
                        'state_fields': STATE_FIELDS,
                        'franka_control': self.ros.get_param('/franka_control', {}),
                        'controller_parameters': self.ros.get_param(
                            '/cartesian_impedance_directional_kinetic_energy_cbf_controller', {}),
                        'trajectory_parameters': self.ros.get_param('/trajectory_publisher', {}),
                        'note': 'CBF core plus joint debug recorded with CBF on AND off. '
                                'No contacts.csv on hardware. robot_state.csv continues during joint '
                                'initialization while CBF diagnostics stop. Events use reception time. '
                                'tau_predicted is a NEXT-cycle desired-torque prediction; compare '
                                'with next tau_J_d. tau_J includes gravity. Force estimates/contact '
                                'flags are libfranka signals, not Gazebo contact wrenches. '
                                'No CBF guarantee is claimed during initialization or aborted braking.',
                    })
                    (self.session / 'metadata.json').write_text(json.dumps(metadata, indent=2))
                    self.state_stream=(self.session / 'robot_state.csv').open('x', newline='')
                    self.state_writer=csv.writer(self.state_stream)
                    self.state_writer.writerow(STATE_FIELDS)
                    self.event_stream=(self.session / 'events.csv').open('x', newline='')
                    self.event_writer=csv.writer(self.event_stream)
                    self.event_writer.writerow(['received_ros_ns', 'received_monotonic_ns', 'phase'])
                self.stream.flush()
                self.active = True
                self.ros.loginfo('CSV recording started: %s', self.session)
                return self.response_type(True, 'Recording: ' + str(self.session))
            except Exception as error:
                self.error = 'Cannot open recording: ' + str(error)
                self.close_files()
                self.ros.logerr(self.error)
                return self.response_type(False, self.error)

    def open_contacts(self):
        if self.contact_stream is None:
            self.contact_stream = (self.session / 'contacts.csv').open('x', newline='')
            self.contact_writer = csv.writer(self.contact_stream)
            self.contact_writer.writerow(CONTACT_FIELDS)

    def write_message(self, topic, msg):
        if topic == '/directional_cbf/phase':
            self.event_writer.writerow(msg)
            return
        if topic == '/franka_state_controller/franka_states':
            msg, received_ns=msg
            seq=int(msg.header.seq)
            if self.previous_state_seq is not None:
                delta=(seq-self.previous_state_seq) % (1 << 32)
                if 1 < delta < (1 << 31):
                    self.state_sequence_gaps += delta-1
            self.previous_state_seq=seq
            row=[msg.header.stamp.to_nsec(), received_ns, msg.robot_mode,
                 msg.control_command_success_rate, error_names(msg.current_errors),
                 error_names(msg.last_motion_errors)]
            for name in STATE_ARRAYS:
                row.extend(getattr(msg, name))
            self.state_writer.writerow(row)
            if not self.have_robot_metadata:
                scalars=('m_ee', 'm_load', 'm_total')
                arrays=('F_x_Cee', 'I_ee', 'F_x_Cload', 'I_load', 'F_x_Ctotal', 'I_total',
                        'F_T_EE', 'F_T_NE', 'NE_T_EE', 'EE_T_K')
                snapshot={key: getattr(msg, key) for key in scalars}
                snapshot.update({key: list(getattr(msg, key)) for key in arrays})
                (self.session / 'robot_model_state.json').write_text(json.dumps(snapshot, indent=2))
                self.have_robot_metadata=True
            self.written['state_rows']=self.written.get('state_rows', 0)+1
            return
        if topic == '/directional_cbf/diagnostics':
            row = self.converter.convert(msg)
            debug = [0] + [''] * (len(DEBUG_FIELDS) - 1)
            if msg.cbf_active or (self.real_robot and msg.debug_valid):
                if not self.real_robot:
                    self.open_contacts()
                debug = [1, msg.robot_mode] + list(msg.q) + list(msg.dq) + list(msg.tau_command)
                self.written['debug_rows'] += 1
            self.writer.writerow(row + debug + (hardware_row(msg) if self.real_robot else []))
            self.written['cbf_rows'] += 1
            self.cbf_missing += self.converter.last_missing
            self.last_cbf_active = bool(msg.cbf_active)
            if not self.real_robot:
                self.gate.diagnostic(msg, self.converter.segment)
            if self.converter.last_missing:
                self.ros.logwarn_throttle(2.0, 'CBF sample-index gaps detected; see CSV')
        else:
            seq = int(msg.header.seq)
            if self.previous_contact_seq is not None:
                delta = (seq - self.previous_contact_seq) % (1 << 32)
                if 1 < delta < (1 << 31):
                    self.contact_sequence_gaps += delta - 1
            self.previous_contact_seq = seq
            self.gate.contact(msg)

    def write_contacts(self, msg, label, cbf_stamp):
        self.open_contacts()
        ns = msg.header.stamp.to_nsec()
        prefix = [ns, label[0], label[1], msg.header.seq, label[3], cbf_stamp,
                  ns - cbf_stamp, len(msg.states)]
        self.written['contact_frames'] += 1
        if not msg.states:
            # A received empty frame proves the stream is alive at this instant.
            self.contact_writer.writerow(prefix + [''] * (len(CONTACT_FIELDS) - len(prefix)))
            self.written['contact_rows'] += 1
        for index, state in enumerate(msg.states):
            force = state.total_wrench.force
            torque = state.total_wrench.torque
            max_force = max((math.sqrt(w.force.x**2 + w.force.y**2 + w.force.z**2)
                             for w in state.wrenches), default=float('nan'))
            self.contact_writer.writerow(prefix + [
                index, state.collision1_name, state.collision2_name,
                len(state.contact_positions), max(state.depths, default=float('nan')),
                force.x, force.y, force.z, torque.x, torque.y, torque.z, max_force])
            self.written['contact_rows'] += 1

    def flush(self):
        self.stream.flush()
        for extra_stream in (self.contact_stream, self.state_stream, self.event_stream):
            if extra_stream is not None:
                extra_stream.flush()
        with self.lock:
            now = time.monotonic()
            ages = {topic: (None if topic not in self.seen else now-self.seen[topic])
                    for topic, _ in self.topics}
            health = {
                'written': dict(self.written), 'local_queue_drops': dict(self.dropped),
                'cbf_missing_samples': self.cbf_missing,
                'state_ros_sequence_gaps': self.state_sequence_gaps,
                'contact_ros_sequence_gaps': self.contact_sequence_gaps,
                'contact_unmatched': self.gate.unmatched,
                'contact_buffer_overflow': self.gate.overflow,
                'contact_frames_discarded_cbf_inactive': self.gate.filtered_inactive,
                'stream_wall_age_seconds': ages,
                'error': self.error, 'clean_shutdown': self.stopping and not self.error,
            }
        temp = self.session / 'health.json.tmp'
        temp.write_text(json.dumps(health, indent=2))
        temp.replace(self.session / 'health.json')
        if self.real_robot:
            return
        age = ages['/directional_cbf/contacts']
        if self.last_cbf_active and (age is None or age > 2.0) and not self.stopping:
            self.ros.logwarn_throttle(2.0, 'CBF debug: no recent Gazebo contact stream; '
                                     'an empty contacts.csv is NOT evidence of no contact')

    def close_files(self):
        for stream in (self.stream, self.contact_stream, self.state_stream, self.event_stream):
            if stream is not None:
                try:
                    stream.close()
                except Exception as error:
                    self.ros.logerr('CSV close failed: %s', error)
        self.stream = self.contact_stream = self.state_stream = self.event_stream = None

    def run(self):
        last_flush = time.monotonic()
        try:
            while not self.ros.is_shutdown():
                try:
                    item = self.pending.get(timeout=0.1)
                except queue.Empty:
                    item = None
                if item is not None:
                    self.write_message(*item)
                if self.active and time.monotonic() - last_flush >= 1.0:
                    self.flush()
                    last_flush = time.monotonic()
        except Exception as error:
            with self.lock:
                self.error = 'CSV recording failed: ' + str(error)
                self.active = False
            self.ros.logerr(self.error)
            while not self.ros.is_shutdown():
                time.sleep(0.1)
        finally:
            with self.lock:
                self.stopping = True
                self.active = False
            for subscriber in self.subscribers:
                subscriber.unregister()
            self.service.shutdown()
            try:
                if self.stream is not None and not self.error:
                    while not self.pending.empty():
                        self.write_message(*self.pending.get_nowait())
                    self.gate.drain(final=True)
                    self.flush()
            finally:
                self.close_files()
            if self.session is not None:
                self.ros.loginfo('CSV recording closed: %s', self.session)


def main():
    import rospy
    rospy.init_node('directional_cbf_recorder')
    ExperimentRecorder().run()


if __name__ == '__main__':
    main()
