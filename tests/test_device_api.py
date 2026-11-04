import math
import sys
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agri_backend.devices import DeviceCache, mgx23_initialization


@pytest.mark.parametrize('bits', range(16))
def test_mgx23_initialization_requires_each_of_the_four_flags(bits):
    # Explicit protocol masks, independent of the implementation's constants.
    masks = (0x00000002, 0x00040000, 0x00080000, 0x00100000)
    status = sum(mask for index, mask in enumerate(masks) if bits & (1 << index))
    # Other bits must neither substitute a missing flag nor prevent success.
    decoded = mgx23_initialization(status | 0x80000000)
    assert decoded['initialized'] is (bits == 15)
    assert [flag['active'] for flag in decoded['flags']] == [bool(bits & (1 << i)) for i in range(4)]


def test_mgx23_unknown_expired_and_recovered_status(monkeypatch):
    from agri_backend import devices
    clock = [10.0]
    monkeypatch.setattr(devices, 'monotonic', lambda: clock[0])
    cache = DeviceCache()
    assert cache.snapshot()['mgx23']['initialized'] is None
    cache.update('mgx23', {'status_word': 0x001C0002})
    assert cache.snapshot()['mgx23']['initialized'] is True
    clock[0] += 1.01
    expired = cache.snapshot()['mgx23']
    assert not expired['fresh'] and expired['initialized'] is None
    assert expired['status_word'] is None
    assert all(flag['active'] is None for flag in expired['flags'])
    cache.update('mgx23', {'status_word': 0})
    assert cache.snapshot()['mgx23']['initialized'] is False


def test_status_subscription_reaches_http_without_gnss_position(monkeypatch):
    pytest.importorskip('rclpy')
    pytest.importorskip('httpx')
    from std_msgs.msg import UInt32
    from fastapi.testclient import TestClient
    from agri_backend import devices, main
    cache = DeviceCache()
    monkeypatch.setattr(devices, 'device_cache', cache)

    class Node:
        callbacks = {}

        def create_subscription(self, cls, topic, callback, qos):
            self.callbacks[topic] = callback

        def create_client(self, *args):
            return None

    node = Node()
    devices.install_ros_devices(node)
    node.callbacks['/mgx23/status'](UInt32(data=0x001C0002))
    response = TestClient(main.app).get('/api/devices').json()  # No ROS context/lifespan.
    assert response['mgx23']['initialized'] is True
    assert response['gnss']['fresh'] is False


def test_cache_fails_unknown_instead_of_false_safe():
    data = DeviceCache().snapshot()
    assert data['safety']['emergency_stop'] is None
    assert data['safety']['motion_inhibited']
    assert data['obstacles']['global_range'] is None
    assert not data['camera']['connected']


def test_cache_isolated_copies_and_bad_json():
    cache = DeviceCache()
    cache.update_json('safety', '{"owner":"manual","emergency_stop":true}')
    value = cache.snapshot()
    value['safety']['owner'] = 'web'
    assert cache.snapshot()['safety']['owner'] == 'manual'
    cache.update_json('safety', '{"bad":NaN}')
    assert cache.snapshot()['safety']['emergency_stop']
    cache.values['safety'] = ({'emergency_stop': False}, -math.inf)
    assert cache.snapshot()['safety']['emergency_stop'] is None


def test_http_routes_are_read_only_or_call_ros_services(monkeypatch):
    pytest.importorskip('fastapi')
    pytest.importorskip('httpx')
    from fastapi.testclient import TestClient
    from agri_backend import main
    client = TestClient(main.app)  # No lifespan: never starts a ROS node/hardware.
    assert client.get('/api/devices').status_code == 200
    assert client.get('/api/camera/status').json()['connected'] is False
    url = client.get('/api/camera/stream-info').json()['stream_url']
    assert url.startswith('/video/stream?') and 'mjpeg' in url
    # The vendor server does not decode percent escapes in the ROS topic.
    assert 'topic=/camera/color/image_raw&' in url
    assert parse_qs(urlsplit(url).query)['topic'] == ['/camera/color/image_raw']
    assert client.post('/api/tasks/unknown/start').status_code == 404
    assert client.post('/api/tasks/waypoint/start').status_code == 503
    assert client.post('/api/robot/emergency-stop/reset').status_code == 503


def test_path_api_preserves_frame_and_filters_invalid_ros_data(monkeypatch):
    pytest.importorskip('httpx')
    pytest.importorskip('rclpy')
    from fastapi.testclient import TestClient
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from agri_backend import main
    monkeypatch.setattr(main, 'robot_state', main.RobotState())
    client = TestClient(main.app)  # No lifespan or ROS node.
    msg = RosPath()
    msg.header.frame_id = 'map'
    point = PoseStamped()
    point.pose.position.x, point.pose.position.y = 12.3, -4.5
    msg.poses = [point]
    main.RobotStatusNode.reference_path_callback(None, msg)
    main.RobotStatusNode.tracked_path_callback(None, msg)
    for kind in ('reference', 'tracked'):
        assert client.get(f'/api/robot/path/{kind}').json() == {
            'count': 1, 'frame_id': 'map', 'points': [{'x': 12.3, 'y': -4.5}]}
    msg.poses[0].pose.position.x = float('nan')
    main.RobotStatusNode.tracked_path_callback(None, msg)
    assert client.get('/api/robot/path/tracked').json()['points'] == []
