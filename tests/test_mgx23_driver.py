"""Exercise real ROS message construction and stream decoding without a socket/node."""

import struct
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from mgx23_driver.mgx23_node import MGX23Node


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Harness:
    receive_data = MGX23Node.receive_data
    publish_data = MGX23Node.publish_data

    def __init__(self, chunk):
        self.buffer = bytearray()
        self.sock = SimpleNamespace(recv=lambda size: chunk)
        self.transport = 'tcp'
        self.last_device_time = None
        self.raw_log = None
        self.max_accuracy = 0.5
        for key in ('attitude_pub', 'attitude_stamped_pub', 'position_pub', 'velocity_pub',
                    'velocity_stamped_pub', 'status_pub', 'accuracy_pub'):
            setattr(self, key, Publisher())

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123)))

    def get_logger(self):
        return SimpleNamespace(warning=lambda message: None)


def frame(device_time=1, latitude=36, accuracy=0.1):
    result = bytearray(100)
    result[:4], result[98:] = b'fmin', b'ed'
    struct.pack_into('<dddd', result, 4, device_time, latitude, 117.0, 50.0)
    struct.pack_into('<f', result, 60, accuracy)
    struct.pack_into('<H', result, 96, sum(result[:96]) & 0xffff)
    return bytes(result)


def test_checksum_rejection_and_stamped_compatibility():
    corrupted = bytearray(frame())
    corrupted[60] ^= 0x80
    h = Harness(bytes(corrupted) + frame())
    h.receive_data()
    assert len(h.position_pub.messages) == 1
    fix, attitude, velocity = (h.position_pub.messages[0], h.attitude_stamped_pub.messages[0],
                               h.velocity_stamped_pub.messages[0])
    assert fix.header.stamp == attitude.header.stamp == velocity.header.stamp
    assert fix.status.status == -1  # No invented RTK/INS classification.


def test_duplicate_device_sample_does_not_refresh_pose():
    h = Harness(frame() + frame())
    h.receive_data()
    assert len(h.position_pub.messages) == 1


def test_stream_reassembles_split_frame():
    packet = frame()
    h = Harness(packet[:31])
    h.receive_data()
    assert not h.position_pub.messages
    h.sock.recv = lambda size: packet[31:]
    h.receive_data()
    assert len(h.position_pub.messages) == 1


def test_nan_and_low_accuracy_samples_rejected():
    h = Harness(frame(latitude=float('nan')) + frame(device_time=2, accuracy=2))
    h.receive_data()
    assert not h.position_pub.messages
