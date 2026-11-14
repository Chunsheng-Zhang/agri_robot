"""Task-scoped read-only telemetry. Disk IO and plotting never run in ROS callbacks."""
import base64
import csv
import json
import logging
import math
import os
import subprocess
import sys
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
from time import monotonic, monotonic_ns, time_ns

from .missions import ROOT, atomic_json, now


TERMINAL = frozenset(('COMPLETED', 'CANCELED', 'FAULT', 'FAILED'))
MOTION_COLUMNS = ['received_ros_ns', 'received_unix_ns', 'received_monotonic_ns',
                  'source_stamp_ns', 'topic', 'frame_id', 'linear_x', 'linear_y', 'angular_z']
MGX_COLUMNS = ['received_ros_ns', 'received_unix_ns', 'received_monotonic_ns',
               'source_stamp_ns', 'topic', 'frame_id', 'v1', 'v2', 'v3', 'status']
ODOM_COLUMNS = ['received_ros_ns', 'received_unix_ns', 'received_monotonic_ns',
                'source_stamp_ns', 'frame_id', 'x', 'y', 'yaw', 'angular_z']
TIMING_COLUMNS = ['received_ros_ns', 'received_unix_ns', 'received_monotonic_ns',
                  'topic', 'source_stamp_ns', 'source_age_ms']
HTTP_COLUMNS = ['received_unix_ns', 'received_monotonic_ns', 'method', 'path',
                'status', 'duration_ms', 'outcome']


class CaptureSession:
    """One bounded producer queue and one writer per task; files are never reused."""
    def __init__(self, directory, source, queue_size=10000):
        self.directory, self.source = Path(directory).resolve(), source
        self.queue = Queue(maxsize=queue_size)
        self.lock, self.finished = Lock(), Event()
        self.accepting, self.stopping = True, False
        self.terminal = None
        self.stop_at = math.inf
        self.last_status = monotonic()
        self.armed_at = monotonic()
        self.seen_active = False
        self.dropped = Counter()
        self.drop_events, self.drop_events_truncated = [], 0
        self.counts, self.errors = Counter(), []
        self.raw_gaps, self.raw_sessions, self.last_raw = [], [], None
        self.raw_size = 0
        self.started_at = now()
        self.files = []
        try:
            (self.directory / 'raw').mkdir(exist_ok=True)
            (self.directory / 'telemetry').mkdir(exist_ok=True)
            (self.directory / 'analysis').mkdir(exist_ok=True)
            (self.directory / 'communication').mkdir(exist_ok=True)
            self.raw = self.open_file('raw/mgx23.bin', 'xb')
            self.chunks = self.csv_file('raw/chunks.csv', ['offset', 'size', 'session', 'sequence',
                'source_ros_ns', 'source_unix_ns', 'received_ros_ns', 'received_unix_ns', 'transport'])
            self.motion = self.csv_file('telemetry/motion.csv', MOTION_COLUMNS)
            self.mgx = self.csv_file('telemetry/mgx23.csv', MGX_COLUMNS)
            self.odom = self.csv_file('telemetry/odometry.csv', ODOM_COLUMNS)
            self.events = self.open_file('telemetry/status.jsonl', 'x')
            self.timing = self.csv_file('communication/ros_timing.csv', TIMING_COLUMNS)
            self.http = self.csv_file('communication/http_server.csv', HTTP_COLUMNS)
            atomic_json(self.directory / 'communication/metadata.json', {
                'schema_version': 1, 'started_at': self.started_at,
                'scope': 'Driver-to-recorder ROS delivery and backend application HTTP handling',
                'observer_qos': 'raw/status: RELIABLE depth 100; motion/MGX/odom/scan/joy: sensor_data BEST_EFFORT',
                'latency_reference_ms': 100, 'http_reference_ms': 2000,
                'notes': ['Raw sequence is generated after bytes reach the MGX23 driver, not by the device.',
                    'Negative source ages indicate clock anomalies and are excluded from latency distributions.',
                    'Headerless messages have no measured source-to-receiver latency.',
                    'HTTP duration excludes browser/proxy/network round trip. HTTP failures are not IP packet loss.',
                    'No active ping, packet capture, camera stream or motion is started by this recorder.'],
            })
            self.write_manifest('RECORDING')
        except Exception:
            for file in self.files:
                file.close()
            raise
        self.thread = Thread(target=self.write_loop, name='experiment-writer', daemon=True)
        self.thread.start()

    def open_file(self, name, mode):
        file = (self.directory / name).open(mode, **({} if 'b' in mode else {'encoding': 'utf-8', 'newline': ''}))
        self.files.append(file)
        return file

    def csv_file(self, name, columns):
        writer = csv.writer(self.open_file(name, 'x'))
        writer.writerow(columns)
        return writer

    def put(self, kind, topic, payload):
        # Queue insertion and stop share one lock: nothing can arrive after close.
        with self.lock:
            if not self.accepting:
                return
            try:
                self.queue.put_nowait((kind, topic, payload))
            except Full:
                self.dropped[topic] += 1
                if len(self.drop_events) < 1000:
                    self.drop_events.append({'received_monotonic_ns': monotonic_ns(), 'topic': topic})
                else:
                    self.drop_events_truncated += 1

    def request_stop(self, state, reason='', tail=0.5):
        with self.lock:
            if self.terminal is None:
                self.terminal = {'state': state, 'reason': reason, 'observed_at': now()}
                self.stop_at = monotonic() + tail

    def stop(self):
        with self.lock:
            self.accepting, self.stopping = False, True

    def write_manifest(self, state):
        atomic_json(self.directory / 'telemetry/recording.json', {
            'schema_version': 1, 'state': state, 'started_at': self.started_at,
            'updated_at': now(), 'terminal': self.terminal, 'source': self.source,
            'samples_by_topic': dict(self.counts), 'queue_drops_by_topic': dict(self.dropped),
            'queue_drop_events': list(self.drop_events), 'queue_drop_events_truncated': self.drop_events_truncated,
            'errors': self.errors, 'raw_bytes': self.raw_size,
            'raw_sequence_gaps': self.raw_gaps, 'raw_sessions': self.raw_sessions,
            'communication': {'metadata': 'communication/metadata.json', 'timing': 'communication/ros_timing.csv',
                              'http_server': 'communication/http_server.csv'},
            'notes': ['Capture begins before the explicit Web start service request.',
                'Unstamped Twist messages have receipt time only; source_stamp_ns is empty.',
                'MGX23 CSV v1/v2/v3: position=lat/lon/alt; attitude=roll/pitch/north-clockwise yaw (rad); velocity=north/east/down (m/s).',
                'Raw bytes are received chunks before protocol validation; gaps/restarts are reported.',
                'A missing topic is missing data, never a zero measurement.'],
        })

    def write_raw(self, receipt, text):
        value = json.loads(text)
        if value.get('schema_version') != 1:
            raise ValueError('Unsupported raw packet schema')
        raw = base64.b64decode(value['data_base64'], validate=True)
        session, sequence = str(value['session']), int(value['sequence'])
        if not raw or len(raw) > 65535 or sequence < 1:
            raise ValueError('Invalid raw chunk')
        if self.last_raw is not None and self.last_raw[0] == session:
            if sequence <= self.last_raw[1]:
                raise ValueError('Duplicate or out-of-order raw chunk')
            if sequence != self.last_raw[1] + 1:
                self.raw_gaps.append({'session': session, 'after': self.last_raw[1], 'before': sequence})
        if session not in self.raw_sessions:
            self.raw_sessions.append(session)
        self.raw.write(raw)
        self.chunks.writerow([self.raw_size, len(raw), session, sequence, value['stamp_ns'],
            value['unix_ns'], receipt[0], receipt[1], value['transport']])
        self.raw_size += len(raw)
        self.last_raw = session, sequence
        self.write_timing(receipt, '/mgx23/raw', value['stamp_ns'])

    def write_timing(self, receipt, topic, stamp=''):
        source = int(stamp) if stamp not in ('', None) else 0
        age = (int(receipt[0])-source)/1e6 if source > 0 else ''
        self.timing.writerow(list(receipt) + [topic, source or '', age])

    def write_loop(self):
        last_flush = monotonic()
        failed = False
        try:
            while True:
                try:
                    kind, topic, payload = self.queue.get(timeout=0.1)
                except Empty:
                    if self.stopping:
                        break
                else:
                    try:
                        if kind == 'raw':
                            self.write_raw(*payload)
                        elif kind == 'event':
                            self.events.write(json.dumps({'topic': topic, **payload}, allow_nan=False) + '\n')
                            stamp=payload['data'].get('stamp')
                            stamp=int(stamp*1e9) if isinstance(stamp,(float,int)) and math.isfinite(stamp) else 0
                            self.write_timing([payload['received_ros_ns'], payload.get('received_unix_ns', ''),
                                payload.get('received_monotonic_ns', '')], topic, stamp)
                        elif kind == 'timing':
                            self.write_timing(payload[:3], topic, payload[3])
                        else:
                            getattr(self, kind).writerow(payload)
                            if kind in ('motion', 'mgx', 'odom'):
                                self.write_timing(payload[:3], topic, payload[3])
                        self.counts[topic] += 1
                    except (ValueError, KeyError, TypeError) as exc:
                        if len(self.errors) < 100:
                            self.errors.append(f'{topic}: {exc}')
                    finally:
                        self.queue.task_done()
                if monotonic() - last_flush >= 1:
                    for file in self.files:
                        file.flush()
                    last_flush = monotonic()
        except Exception as exc:
            failed = True
            self.errors.append(f'Recording IO failed: {exc}')
            self.stop()
        finally:
            for file in self.files:
                try:
                    file.close()
                except OSError as exc:
                    failed = True
                    self.errors.append(str(exc))
            try:
                self.write_manifest('ERROR' if failed else 'CLOSED')
            except OSError:
                logging.getLogger(__name__).exception('Cannot finalize experiment recording')
            self.finished.set()


class ExperimentRecorder:
    """ROS adapter: never publishes motion, opens devices or waits for disk IO."""
    def __init__(self, node):
        from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import NavSatFix, LaserScan, Joy
        from std_msgs.msg import String, Float32, UInt32
        from rclpy.qos import qos_profile_sensor_data
        self.node, self.session = node, None
        self.lock = Lock()
        self.closed = False
        self.analysis = ThreadPoolExecutor(max_workers=1, thread_name_prefix='experiment-analysis')
        self.pending_analysis = {}
        self.subscriptions = []
        def sub(cls, topic, callback, qos=100):
            self.subscriptions.append(node.create_subscription(cls, topic, callback, qos))
        for topic in ('/cmd_vel', '/base/driver_cmd_vel', '/vel_raw'):
            sub(Twist, topic, lambda msg, t=topic: self.on_motion(t, msg), qos_profile_sensor_data)
        for topic in ('/control/waypoint/cmd_vel', '/control/tracking/cmd_vel', '/control/following/cmd_vel'):
            sub(TwistStamped, topic, lambda msg, t=topic: self.on_motion(t, msg), qos_profile_sensor_data)
        sub(String, '/mgx23/raw', self.on_raw)
        sub(NavSatFix, '/mgx23/position', self.on_position, qos_profile_sensor_data)
        for topic in ('/mgx23/attitude_stamped', '/mgx23/velocity_stamped'):
            sub(Vector3Stamped, topic, lambda msg, t=topic: self.on_vector(t, msg), qos_profile_sensor_data)
        for cls, topic in ((Float32, '/mgx23/accuracy'), (UInt32, '/mgx23/status')):
            sub(cls, topic, lambda msg, t=topic: self.on_scalar(t, msg), qos_profile_sensor_data)
        sub(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        for cls, topic in ((LaserScan, '/scan'), (Joy, '/joy')):
            sub(cls, topic, lambda msg, t=topic: self.put('timing', t, self.receipt()+[self.stamp(msg)]), qos_profile_sensor_data)
        for topic in ('/tracking/status', '/following/status', '/waypoint/status', '/safety/status', '/safety/events'):
            sub(String, topic, lambda msg, t=topic: self.on_status(t, msg))
        self.timer = node.create_timer(0.1, self.tick)

    def begin(self, directory, source):
        # API thread only. Give a previous terminal run time to finish its short
        # stop tail; the ROS executor and disk writer continue independently.
        previous = self.session
        if previous and previous.terminal is not None:
            previous.finished.wait(timeout=2)
            self.tick()
        with self.lock:
            if self.closed:
                raise RuntimeError('实验记录器已关闭，请重启后端')
            if self.session:
                raise RuntimeError('上一轮实验数据仍在收尾，请稍后再开始')
            directory = Path(directory).resolve()
            future = self.pending_analysis.get(directory)
            if future and not future.done():
                raise RuntimeError('该轮实验仍在分析，请稍后再开始')
            # x mode refuses overwriting a prior attempt, even on a repeated start.
            self.session = CaptureSession(directory, source)

    def abort(self, reason):
        if self.session:
            self.session.request_stop('START_REJECTED', reason, tail=0)

    def busy(self, directory):
        directory = Path(directory).resolve()
        with self.lock:
            session = self.session
            future = self.pending_analysis.get(directory)
            return bool((session and session.directory == directory) or (future and not future.done()))

    def receipt(self):
        return [self.node.get_clock().now().nanoseconds, time_ns(), monotonic_ns()]

    @staticmethod
    def stamp(msg):
        header = getattr(msg, 'header', None)
        return header.stamp.sec * 1000000000 + header.stamp.nanosec if header else ''

    def put(self, kind, topic, values):
        session = self.session
        if session:
            session.put(kind, topic, values)

    def on_motion(self, topic, msg):
        twist = getattr(msg, 'twist', msg)
        self.put('motion', topic, self.receipt() + [self.stamp(msg), topic,
            getattr(getattr(msg, 'header', None), 'frame_id', 'base_link'),
            twist.linear.x, twist.linear.y, twist.angular.z])

    def on_raw(self, msg):
        self.put('raw', '/mgx23/raw', (self.receipt(), msg.data))

    def on_position(self, msg):
        self.put('mgx', '/mgx23/position', self.receipt() + [self.stamp(msg), '/mgx23/position',
            msg.header.frame_id, msg.latitude, msg.longitude, msg.altitude, msg.status.status])

    def on_vector(self, topic, msg):
        self.put('mgx', topic, self.receipt() + [self.stamp(msg), topic, msg.header.frame_id,
            msg.vector.x, msg.vector.y, msg.vector.z, ''])

    def on_scalar(self, topic, msg):
        self.put('mgx', topic, self.receipt() + ['', topic, '', msg.data, '', '', ''])

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self.put('odom', '/odom', self.receipt() + [self.stamp(msg), msg.header.frame_id,
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, msg.twist.twist.angular.z])

    def on_status(self, topic, msg):
        session = self.session
        if not session:
            return
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
            receipt = self.receipt()
            self.put('event', topic, dict(zip(('received_ros_ns', 'received_unix_ns', 'received_monotonic_ns'), receipt), data=value))
            if topic != f'/{session.source}/status':
                return
            # Reject periodic/latched status from an old run or controller.
            expected = str(session.directory / 'tracks')
            if value.get('run_directory') != expected:
                return
            session.last_status = monotonic()
            if value.get('state') in ('ACQUIRING', 'RUNNING', 'PAUSED'):
                session.seen_active = True
            if value.get('state') in TERMINAL and session.seen_active:
                session.request_stop(value['state'], value.get('reason', ''))
        except (ValueError, TypeError):
            pass

    def tick(self):
        with self.lock:
            if self.closed:
                return
            session = self.session
            if not session:
                return
            if session.terminal is None:
                if not session.seen_active and monotonic() - session.armed_at > 10:
                    session.request_stop('START_UNCONFIRMED', 'No active task state observed within 10 seconds; motion is not confirmed stopped', tail=0)
                elif monotonic() - session.last_status > 10:
                    session.request_stop('INTERRUPTED', 'Controller status unavailable for 10 seconds', tail=0)
            if monotonic() >= session.stop_at:
                session.stop()
            if session.finished.is_set():
                self.pending_analysis[session.directory] = self.analysis.submit(self.analyze, session.directory)
                self.session = None
            self.pending_analysis = {p: f for p, f in self.pending_analysis.items() if not f.done()}

    @staticmethod
    def analyze(directory):
        try:
            # Plotting gets its own Python process: rendering cannot hold the
            # ROS bridge's interpreter lock or grow its long-lived image heap.
            with (directory / 'analysis/analysis.log').open('w', encoding='utf-8') as log:
                subprocess.run([sys.executable, '-B', '-m', 'agri_backend.experiment_analysis', str(directory)],
                    cwd=ROOT, env=dict(os.environ, MPLCONFIGDIR=str(ROOT / '.diagnostics/matplotlib'),
                                      PYTHONDONTWRITEBYTECODE='1'),
                    stdout=log, stderr=subprocess.STDOUT, timeout=300, check=True)
        except Exception as exc:
            logging.getLogger(__name__).exception('Experiment analysis failed')
            atomic_json(directory / 'analysis/status.json', {'state': 'FAILED', 'reason': str(exc), 'updated_at': now()})

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            session = self.session
            if session:
                session.request_stop('INTERRUPTED', 'Backend stopped', tail=0)
                session.stop()
        if session:
            session.thread.join(timeout=5)
            if session.finished.is_set():
                self.analysis.submit(self.analyze, session.directory)
        self.analysis.shutdown(wait=True)
