"""HTTP policy requests must use the ROS service and preserve failures."""
from types import SimpleNamespace

import pytest
from fastapi import FastAPI, HTTPException
from fastapi.testclient import TestClient

from agri_backend import devices


def test_lidar_protection_api_validates_booleans_and_forwards_service(monkeypatch):
    app, node, calls = FastAPI(), object(), []
    devices.install_routes(app, lambda: SimpleNamespace(node=node))

    def service(target, name, active=None):
        calls.append((target, name, active))
        return {'success': True, 'message': 'confirmed'}

    monkeypatch.setattr(devices, 'service_call', service)
    with TestClient(app) as client:
        for enabled in (True, False):
            result = client.post('/api/robot/lidar-protection', json={'enabled': enabled})
            assert result.status_code == 200
            assert result.json() == {'success': True, 'message': 'confirmed', 'enabled': enabled}
        assert calls == [(node, '/safety/lidar_protection', True),
                         (node, '/safety/lidar_protection', False)]
        for payload in ({}, {'enabled': 'false'}, {'enabled': 1}, {'enabled': None}):
            assert client.post('/api/robot/lidar-protection', json=payload).status_code == 422
        assert len(calls) == 2


@pytest.mark.parametrize('code', [409, 503, 504])
def test_lidar_protection_api_does_not_claim_success_after_ros_failure(monkeypatch, code):
    app = FastAPI()
    devices.install_routes(app, lambda: SimpleNamespace(node=object()))

    def reject(*args, **kwargs):
        raise HTTPException(code, 'service unavailable or rejected')

    monkeypatch.setattr(devices, 'service_call', reject)
    with TestClient(app) as client:
        result = client.post('/api/robot/lidar-protection', json={'enabled': True})
        assert result.status_code == code
        assert 'enabled' not in result.json()
