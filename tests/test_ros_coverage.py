"""Opt-in integration against the installed, existing C++ OpenNav server."""

import json
import os
import signal
import subprocess
import tempfile
from pathlib import Path
from time import monotonic

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('AGRI_ROS_TESTS') != '1',
                                reason='Requires OpenNav underlay and isolated ROS domain')


@pytest.mark.parametrize('headland_width,expect_success', [(3.0, True), (0.0, False)])
def test_existing_opennav_action_to_reference_path(tmp_path, headland_width, expect_success):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from ament_index_python.packages import get_package_prefix
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import Path as PathMsg
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from agri_base.coordinates import LocalFrame
    from agri_base.runtime import LATCHED
    from agri_path_planning.node import CoverageAdapter

    root = Path(__file__).resolve().parents[1]
    config = root / 'agri_robot_ws/src/agri_bringup/config'
    server = Path(get_package_prefix('opennav_coverage')) / 'lib/opennav_coverage/opennav_coverage'
    log = tempfile.TemporaryFile(mode='w+t')
    process = subprocess.Popen([str(server), '--ros-args', '--params-file', str(config / 'system.yaml')],
                               stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(args=['--ros-args', '--params-file', str(config / 'system.yaml'),
                    '-p', f'boundary_file:={config / "field.yaml"}',
                    '-p', f'coverage_adapter:headland_width:={headland_width}',
                    '-p', f'output_directory:={tmp_path}'])
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        adapter, bench = CoverageAdapter(), Node('coverage_testbench')
        assert adapter.get_parameter('headland_width').value == headland_width
        nodes = [adapter, bench]
        for node in nodes:
            executor.add_node(node)
        received = []
        bench.create_subscription(PathMsg, '/reference_path', lambda p: received.append(p), LATCHED)
        origin = bench.create_publisher(String, '/localization/origin', LATCHED)
        origin.publish(String(data=json.dumps(LocalFrame(36.6468527633281, 117.021433066391).identity)))

        def until(predicate, timeout=20):
            deadline = monotonic() + timeout
            while monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                if predicate():
                    return
                if process.poll() is not None:
                    break
            log.seek(0)
            raise AssertionError(f'{adapter.state}: {adapter.reason}\n{log.read()[-5000:]}')

        lifecycle = bench.create_client(ChangeState, '/coverage_server/change_state')
        until(lifecycle.service_is_ready)
        for transition in (1, 3):  # configure, activate
            request = ChangeState.Request()
            request.transition.id = transition
            future = lifecycle.call_async(request)
            until(future.done)
            assert future.result().success
        until(lambda: adapter.origin_matches and adapter.client.server_is_ready())
        plan = bench.create_client(Trigger, '/planning/plan')
        until(plan.service_is_ready)
        future = plan.call_async(Trigger.Request())
        until(future.done)
        assert future.result().success
        until(lambda: adapter.state in ('FAILED', 'SUCCEEDED'), 90)
        if not expect_success:
            assert adapter.state == 'FAILED' and 'clearance' in adapter.reason
            assert not list(tmp_path.glob('coverage_*.json'))
            assert not any(p.poses for p in received)
            return
        assert adapter.state == 'SUCCEEDED', adapter.reason
        until(lambda: received and len(received[-1].poses) > 2)
        path = received[-1]
        assert path.header.frame_id == 'map'
        assert all(p.header.frame_id == 'map' for p in path.poses)
        artifacts = list(tmp_path.glob('coverage_*.json'))
        assert len(artifacts) == 1
        saved = json.loads(artifacts[0].read_text())
        assert len(saved['points']) == len(path.poses)
        assert not any(saved['is_backward'])
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=5)
        log.close()
