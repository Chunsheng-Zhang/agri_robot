"""Exercise the actual chassis boundary without DDS, drivers or hardware."""
import json
import math
from functools import partial
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from agri_base import chassis_adapter as module


@pytest.fixture
def bench(monkeypatch):
    clock = [10.]
    output = []
    publishers = {'/cmd_vel': 1, '/base/driver_cmd_vel': 1}
    monkeypatch.setattr(module, 'monotonic', lambda: clock[0])
    adapter = SimpleNamespace(last_cmd=-math.inf, last_safety=-math.inf,
                              enabled=False, inhibited=True, idle_quiet=False,
                              owner='', command=Twist(),
                              pub=SimpleNamespace(publish=lambda m: output.append(
                                  (m.linear.x, m.linear.y, m.angular.z))),
                              count_publishers=lambda name: publishers[name],
                              get_parameter=lambda name: SimpleNamespace(value=True))
    adapter.tick = partial(module.ChassisAdapter.tick, adapter)
    adapter.on_command = partial(module.ChassisAdapter.on_command, adapter)

    def safety(owner='', estop=False, inhibited=False, obstacle=False):
        msg = String(data=json.dumps(dict(owner=owner, emergency_stop=estop,
                                         motion_inhibited=inhibited, obstacle_stop=obstacle)))
        module.ChassisAdapter.on_safety(adapter, msg)

    return adapter, safety, clock, output, publishers


def command(adapter, v=.2, lateral=.1, angular=.3):
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = v, lateral, angular
    adapter.on_command(msg)


def test_unlocked_idle_stops_once_then_silences_even_with_incoming_commands(bench):
    adapter, safety, clock, output, _ = bench
    safety()
    assert output == [(0., 0., 0.)]
    for _ in range(100):
        clock[0] += .05
        command(adapter)  # An unowned command must never reach the driver.
        safety()
        adapter.tick()
    assert output == [(0., 0., 0.)]


@pytest.mark.parametrize('flags', [dict(estop=True), dict(inhibited=True), dict(obstacle=True)])
def test_ownerless_safety_inhibit_continuously_sends_zero(bench, flags):
    adapter, safety, clock, output, _ = bench
    safety()
    for _ in range(10):
        clock[0] += .05
        safety(**flags)
        command(adapter)
        adapter.tick()
    assert len(output) == 21 and all(v == (0., 0., 0.) for v in output)
    # Releasing estop alone never replays a previous command.
    safety()
    count = len(output)
    adapter.tick()
    assert len(output) == count and output[-1] == (0., 0., 0.)


def test_unknown_or_stale_safety_keeps_continuous_zero(bench):
    adapter, safety, clock, output, _ = bench
    for _ in range(4):
        adapter.tick()
    assert len(output) == 4
    safety()
    count = len(output)
    clock[0] += .31
    for _ in range(4):
        adapter.tick()
    assert len(output) == count + 4 and all(v == (0., 0., 0.) for v in output)
    safety()
    count = len(output)
    adapter.tick()
    assert len(output) == count


@pytest.mark.parametrize('topic,count', [('/cmd_vel', 0), ('/cmd_vel', 2),
                                        ('/base/driver_cmd_vel', 2)])
def test_conflicting_or_missing_publishers_prevent_quiet_idle(bench, topic, count):
    adapter, safety, _, output, publishers = bench
    safety()
    publishers[topic] = count
    for _ in range(4):
        adapter.tick()
    assert len(output) == 5 and all(v == (0., 0., 0.) for v in output)


def test_release_stops_once_and_reacquire_requires_new_command(bench):
    adapter, safety, _, output, _ = bench
    safety('web')
    command(adapter)
    adapter.tick()
    assert output[-1] == (.2, .1, .3)
    safety()
    assert output[-1] == (0., 0., 0.)
    count = len(output)
    for _ in range(5):
        adapter.tick()
    assert len(output) == count
    command(adapter, v=.8)
    safety('manual')
    adapter.tick()
    assert output[-1] == (0., 0., 0.)
    command(adapter, v=.1)
    adapter.tick()
    assert output[-1][0] == .1


def test_owned_command_timeout_and_nan_continuously_stop(bench):
    adapter, safety, clock, output, _ = bench
    safety('web')
    command(adapter)
    adapter.tick()
    assert output[-1][0] == .2
    clock[0] += .26
    safety('web')
    for _ in range(4):
        adapter.tick()
    assert output[-4:] == [(0., 0., 0.)] * 4
    command(adapter, v=float('nan'))
    adapter.tick()
    assert output[-1] == (0., 0., 0.)
