"""Use the existing C++ web_video_server with a synthetic RGB image publisher."""

import os
import platform
import signal
import socket
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from time import monotonic
from urllib.error import URLError
from urllib.parse import urlencode
from urllib.request import urlopen

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('AGRI_ROS_TESTS') != '1', reason='Opt-in DDS/loopback HTTP test')


def test_reused_video_server_streams_synthetic_images():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from ament_index_python.packages import get_package_prefix

    port = 8080  # Bundled fork does not declare port/address parameter overrides.
    binary = Path(get_package_prefix('web_video_server')) / 'lib/web_video_server/web_video_server'
    with binary.open('rb') as executable:
        header = executable.read(20)
    machine = int.from_bytes(header[18:20], 'little') if header[:4] == b'\x7fELF' else 0
    if (machine == 183 and platform.machine() not in ('aarch64', 'arm64')) or (machine == 62 and platform.machine() != 'x86_64'):
        pytest.skip('Vendor binary architecture differs from host; run on matching Jetson or rebuild vendor underlay')
    with socket.socket() as selected:
        try:
            selected.bind(('127.0.0.1', port))
        except OSError:
            pytest.skip('Port 8080 already in use; do not interfere with an existing video service')
    log = tempfile.TemporaryFile(mode='w+t')
    process = subprocess.Popen([str(binary)], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(args=[])
    node = Node('video_testbench')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    publisher = node.create_publisher(Image, '/camera/color/image_raw', 1)
    def image():
        msg = Image()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.width, msg.height, msg.step, msg.encoding = 32, 24, 96, 'bgr8'
        msg.data = bytes([20, 100, 200]) * (32 * 24)
        publisher.publish(msg)
    node.create_timer(0.05, image)
    try:
        deadline = monotonic() + 6
        ready = False
        while monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            try:
                with urlopen(f'http://127.0.0.1:{port}/', timeout=0.2) as response:
                    ready = response.status == 200
                if ready:
                    break
            except (URLError, TimeoutError):
                continue
        assert ready
        # Discovery must see the image topic before the streamer is requested.
        deadline = monotonic() + 0.5
        while monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        url = f'http://127.0.0.1:{port}/stream?' + urlencode({'topic': '/camera/color/image_raw', 'type': 'mjpeg'})
        def read_frame():
            with urlopen(url, timeout=4) as response:
                return response.headers.get('Content-Type'), response.read(2048)
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(read_frame)
            deadline = monotonic() + 5
            while not future.done() and monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
            kind, content = future.result(timeout=1)
        assert 'multipart' in kind
        assert b'\xff\xd8' in content
    except Exception:
        log.seek(0)
        print(log.read()[-3500:])
        raise
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=4)
        log.close()
