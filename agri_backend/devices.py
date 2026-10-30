"""Read-only device snapshots and ROS-service HTTP adapters; no chassis publisher."""

import copy
import json
import math
import os
from collections import deque
from threading import Event, Lock
from time import monotonic
from urllib.parse import urlencode


class DeviceCache:
    def __init__(self):
        self.lock, self.values = Lock(), {}
        self.events = deque(maxlen=100)

    def add_event(self, value):
        with self.lock:
            self.events.append(copy.deepcopy(value))

    def recent_events(self):
        with self.lock:
            return list(reversed(copy.deepcopy(self.events)))

    def update(self, name, value):
        with self.lock:
            self.values[name] = (copy.deepcopy(value), monotonic())

    def update_json(self, name, text):
        try:
            def reject(value):
                raise ValueError(value)
            value = json.loads(text, parse_constant=reject)
            if not isinstance(value, dict):
                return
            self.update(name, value)
        except (ValueError, TypeError):
            return

    def snapshot(self):
        now = monotonic()
        with self.lock:
            items = copy.deepcopy(self.values)
        result = {}
        for name in ('safety', 'obstacles', 'manual', 'camera', 'waypoint', 'tracking', 'planning', 'gnss', 'origin', 'voltage', 'scan'):
            value, received = items.get(name, ({}, -math.inf))
            age = now - received
            fresh = age < (0.5 if name == 'gnss' else 3.0 if name == 'voltage' else 1.0)
            if name == 'origin':
                fresh = bool(value)  # Fixed configuration, deliberately latched; not a heartbeat.
            result[name] = {**value, 'fresh': fresh, 'age_seconds': round(age, 3) if math.isfinite(age) else None}
            if not fresh:
                if name in ('camera', 'manual'):
                    result[name]['connected'] = False
                if name == 'obstacles':
                    result[name].update(valid=False, global_range=None, front_distance=None, rear_distance=None)
                if name == 'safety':
                    result[name].update(emergency_stop=None, obstacle_stop=None, owner=None,
                                        motion_inhibited=True, localization_valid=False,
                                        joy_connected=False, lidar_connected=False)
        return result


device_cache = DeviceCache()


def install_ros_devices(node):
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import NavSatFix, LaserScan
    from std_msgs.msg import String, Float32
    from std_srvs.srv import SetBool, Trigger
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    for name, topic, durable in (
        ('safety', '/safety/status', True), ('obstacles', '/safety/obstacles', False),
        ('manual', '/manual/status', False), ('camera', '/camera/status', True),
        ('waypoint', '/waypoint/status', True), ('tracking', '/tracking/status', True),
        ('planning', '/planning/status', True),
    ):
        node.create_subscription(String, topic,
            lambda msg, key=name: device_cache.update_json(key, msg.data), latched if durable else 1)
    node.create_subscription(String, '/localization/origin',
        lambda msg: device_cache.update('origin', {'projection': json.loads(msg.data)}), latched)
    node.create_subscription(Float32, '/voltage', lambda msg: device_cache.update(
        'voltage', {'volts': float(msg.data)}) if math.isfinite(msg.data) and msg.data > 0 else None, 1)

    def scan(msg):
        if not math.isfinite(msg.angle_increment) or not math.isfinite(msg.angle_min):
            return
        stride = max(1, math.ceil(len(msg.ranges) / 720))
        points = []
        for index in range(0, len(msg.ranges), stride):
            value = msg.ranges[index]
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max:
                angle = msg.angle_min + index * msg.angle_increment
                points.append({'x': value * math.cos(angle), 'y': value * math.sin(angle)})
        device_cache.update('scan', {'frame_id': msg.header.frame_id, 'points': points,
                                    'range_max': float(msg.range_max) if math.isfinite(msg.range_max) else None})
    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    def event(msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                device_cache.add_event({**value, 'channel': 'ros'})
        except (ValueError, TypeError):
            pass
    node.create_subscription(String, '/safety/events', event, latched)

    def gnss(msg):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            device_cache.update('gnss', {'latitude': msg.latitude, 'longitude': msg.longitude,
                                         'fix_status': msg.status.status})
    node.create_subscription(NavSatFix, '/mgx23/position', gnss, qos_profile_sensor_data)
    node.device_service_clients = {'/safety/emergency_stop': node.create_client(SetBool, '/safety/emergency_stop'),
                                    '/safety/obstacle/reset': node.create_client(Trigger, '/safety/obstacle/reset')}
    for source in ('waypoint', 'tracking'):
        for action in ('start', 'pause', 'resume', 'cancel'):
            name = f'/{source}/{action}'
            node.device_service_clients[name] = node.create_client(Trigger, name)


def service_call(node, name, active=None):
    from fastapi import HTTPException
    from std_srvs.srv import SetBool, Trigger
    client = node.device_service_clients[name]
    if not client.service_is_ready():
        raise HTTPException(503, 'ROS 2 安全/任务服务不可用')
    request = Trigger.Request() if active is None else SetBool.Request(data=active)
    finished = Event()
    try:
        future = client.call_async(request)
        future.add_done_callback(lambda result: finished.set())
        if not finished.wait(2.0):
            raise HTTPException(504, 'ROS 2 响应超时；请求可能仍在执行，请检查状态')
        result = future.result()
    except HTTPException:
        raise
    except Exception as exc:
        raise HTTPException(503, f'ROS 2 服务请求失败：{exc}') from exc
    if not result.success:
        raise HTTPException(409, result.message)
    return {'success': True, 'message': result.message}


def install_routes(app, require_bridge):
    from fastapi import APIRouter, HTTPException
    router = APIRouter()

    @router.get('/api/devices')
    def devices():
        return device_cache.snapshot()

    @router.get('/api/robot/safety')
    def safety():
        snapshot = device_cache.snapshot()
        return {'safety': snapshot['safety'], 'obstacles': snapshot['obstacles'], 'manual': snapshot['manual']}

    @router.get('/api/camera/status')
    def camera():
        return device_cache.snapshot()['camera']

    @router.get('/api/camera/stream-info')
    def stream_info():
        base = os.getenv('AGRI_VIDEO_BASE_URL', '/video').rstrip('/')
        topic = os.getenv('AGRI_CAMERA_TOPIC', '/camera/color/image_raw')
        # The bundled vendor server uses the raw query topic as a ROS name;
        # percent-encoded slashes make it close the stream without a JPEG.
        query = urlencode({'topic': topic, 'type': 'mjpeg', 'quality': 70, 'width': 640, 'height': 480}, safe='/')
        return {'stream_url': f'{base}/stream?{query}', 'camera': device_cache.snapshot()['camera']}

    @router.post('/api/robot/obstacle-stop/reset')
    def reset_obstacle():
        return service_call(require_bridge().node, '/safety/obstacle/reset')

    @router.post('/api/tasks/{source}/{action}')
    def task(source: str, action: str):
        if source not in ('waypoint', 'tracking') or action not in ('start', 'pause', 'resume', 'cancel'):
            raise HTTPException(404, 'Unknown task operation')
        result = service_call(require_bridge().node, f'/{source}/{action}')
        if hasattr(app.state, 'workspace'):
            app.state.workspace.event(f'TASK_{action.upper()}', source=source)
        return result

    app.include_router(router)
