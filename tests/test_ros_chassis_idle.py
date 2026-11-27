"""Opt-in isolated DDS check; no real driver, serial port or motion hardware."""
import json
import os
from time import monotonic

import pytest

pytestmark = pytest.mark.skipif(os.getenv('AGRI_ROS_TESTS') != '1', reason='Explicit isolated DDS test')


def test_unowned_idle_is_quiet_but_estop_and_watchdogs_keep_zero():
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 230
    assert os.environ['ROS_DOMAIN_ID'] != '225'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from agri_base.chassis_adapter import ChassisAdapter
    from agri_base.runtime import LATCHED

    rclpy.init()
    executor = SingleThreadedExecutor()
    chassis, bench = None, None
    try:
        chassis, bench = ChassisAdapter(), Node('isolated_chassis_idle_bench')
        for node in (chassis, bench):
            executor.add_node(node)
        output = []
        state = dict(owner='', estop=True, safety=True, command=True, v=0.)
        safety_pub = bench.create_publisher(String, '/safety/status', LATCHED)
        command_pub = bench.create_publisher(Twist, '/cmd_vel', 1)
        bench.create_subscription(Twist, '/base/driver_cmd_vel',
            lambda m: output.append((monotonic(), m.linear.x, m.linear.y, m.angular.z)), 10)

        def inputs():
            if state['safety']:
                safety_pub.publish(String(data=json.dumps(dict(owner=state['owner'],
                    emergency_stop=state['estop'], motion_inhibited=state['estop'], obstacle_stop=False))))
            if state['command']:
                msg = Twist()
                msg.linear.x = state['v']
                command_pub.publish(msg)

        bench.create_timer(.05, inputs)

        def spin(seconds):
            deadline = monotonic()+seconds
            while monotonic() < deadline:
                executor.spin_once(timeout_sec=.005)

        def until(predicate, timeout=4.):
            deadline = monotonic()+timeout
            while monotonic() < deadline:
                executor.spin_once(timeout_sec=.005)
                if predicate():
                    return
            pytest.fail(f'Chassis state did not arrive: {state}, output={output[-5:]}')

        def quiet():
            spin(.2)  # Drain the transition stop and queued DDS deliveries.
            count = len(output)
            spin(.35)
            assert len(output) == count

        def repeated_zero():
            spin(.2)
            count = len(output)
            spin(.3)
            assert len(output) >= count+3
            assert all(row[1:] == (0., 0., 0.) for row in output[count:])

        until(lambda: chassis.last_safety > 0 and len(output) > 3)
        repeated_zero()  # Startup estop must keep publishing, despite no owner.
        state['estop'] = False
        until(lambda: chassis.idle_quiet)
        quiet()

        state.update(owner='web', v=.2)
        until(lambda: output[-1][1] > .1)
        count = len(output)
        state.update(owner='', v=0.)
        until(lambda: len(output) > count and output[-1][1:] == (0., 0., 0.) and chassis.idle_quiet)
        quiet()

        state['estop'] = True
        until(lambda: chassis.inhibited)
        repeated_zero()
        state['estop'] = False
        until(lambda: chassis.idle_quiet)
        quiet()

        state['safety'] = False
        spin(.4)
        repeated_zero()
        state['safety'] = True
        until(lambda: chassis.idle_quiet)
        quiet()

        rival = bench.create_publisher(Twist, '/cmd_vel', 1)
        until(lambda: chassis.count_publishers('/cmd_vel') == 2)
        repeated_zero()
        bench.destroy_publisher(rival)
        until(lambda: chassis.idle_quiet)
        quiet()

        state.update(owner='web', v=.2)
        until(lambda: output[-1][1] > .1)
        state['command'] = False
        spin(.3)
        repeated_zero()
        # Ownership change discards the old command and cannot resume it.
        state['owner'] = ''
        until(lambda: chassis.idle_quiet)
        quiet()
        state['owner'] = 'manual'
        until(lambda: chassis.owner == 'manual')
        repeated_zero()
    finally:
        for node in (chassis, bench):
            if node is not None:
                executor.remove_node(node)
                node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
