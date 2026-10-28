import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agri_backend.devices import DeviceCache


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
