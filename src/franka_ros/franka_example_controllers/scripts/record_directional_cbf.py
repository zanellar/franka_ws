#!/usr/bin/env python3
"""ROS subscriber -> buffered CSV. File I/O runs outside the controller process."""
import csv
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
    'missing_samples_before', 'frame_id',
]


class CsvRows:
    """Segment time rewinds/controller restarts, preserving original integer time."""
    def __init__(self):
        self.origin_ns = None
        self.previous_ns = None
        self.previous_index = None
        self.segment = 0

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
        self.previous_ns, self.previous_index = ns, index
        return [ns, (ns - self.origin_ns) * 1e-9, self.segment, index,
                msg.experiment_id, msg.dt, msg.cbf_h, msg.cbf_constraint_safe,
                msg.cbf_constraint_qp, msg.kinetic_energy_dir] + list(msg.svd_jacobian) + [
                    msg.Kmax, msg.alpha, msg.cbf_residual_tolerance,
                ] + list(msg.direction) + [
                    int(msg.cbf_active), int(msg.task_aborted), int(msg.cbf_solution_applied),
                    msg.solver_status, missing, msg.header.frame_id,
                ]


def main():
    import rospy
    from franka_msgs.msg import DirectionalCbfDiagnostics
    rospy.init_node('directional_cbf_recorder')
    output_dir = Path(rospy.get_param('~output_dir', '~/.ros/directional_cbf')).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    filename = 'directional_cbf_{}_{}.csv'.format(
        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'), uuid.uuid4().hex[:8])
    path = output_dir / filename
    pending = queue.Queue(maxsize=20000)
    dropped = [0]

    def receive(msg):
        try:
            pending.put_nowait(msg)
        except queue.Full:
            dropped[0] += 1
            rospy.logwarn_throttle(2.0, 'CBF CSV queue full: samples are being dropped')

    converter = CsvRows()
    subscriber = rospy.Subscriber('/directional_cbf/diagnostics', DirectionalCbfDiagnostics,
                                  receive, queue_size=10000, buff_size=4 * 1024 * 1024,
                                  tcp_nodelay=True)
    written = 0
    missing = 0
    try:
        # Never overwrite an existing experiment file.
        with path.open('x', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(FIELDS)
            stream.flush()
            rospy.loginfo('Recording directional CBF CSV: %s', path)
            last_flush = time.monotonic()
            stopping = False
            while True:
                if rospy.is_shutdown() and not stopping:
                    subscriber.unregister()
                    stopping = True
                if stopping and pending.empty():
                    break
                try:
                    msg = pending.get(timeout=0.2)
                except queue.Empty:
                    msg = None
                if msg is not None:
                    row = converter.convert(msg)
                    writer.writerow(row)
                    written += 1
                    missing += row[-2]
                    if row[-2]:
                        rospy.logwarn_throttle(2.0, 'CBF sample-index gaps detected; see CSV')
                if time.monotonic() - last_flush >= 1.0:
                    stream.flush()
                    last_flush = time.monotonic()
            stream.flush()
    finally:
        subscriber.unregister()
        rospy.loginfo('CBF CSV: %d rows, %d index gaps, %d local queue drops: %s',
                      written, missing, dropped[0], path)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError) as error:
        raise SystemExit('CBF CSV recording failed: {}'.format(error))
