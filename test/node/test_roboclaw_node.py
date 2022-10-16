#!/usr/bin/env python
import time
from typing import List

import pytest
import rclpy
from rclpy.node import Node
import rclpy.logging

from roboclaw_interfaces.msg import SpeedCommand, Stats
from roboclaw_driver.roboclaw_node import RoboclawNode

DEFAULT_STATS_TOPIC = "stats"
DEFAULT_CMD_TOPIC = "speed_command"

PARAM_DEV_NAMES = 'dev_names'
PARAM_BAUD_RATE = 'baud_rate'
PARAM_ADDRESS = 'address'


class NodeTestingNode(Node):
    def __init__(self):
        super().__init__('node_tester')

        self.stats = Stats()

        self.speed_pub = self.create_publisher(
            msg_type=SpeedCommand,
            topic=DEFAULT_CMD_TOPIC,
            qos_profile=1
        )

        self.create_subscription(
            msg_type=Stats,
            topic=DEFAULT_STATS_TOPIC,
            callback=self._stats_callback,
            qos_profile=10
        )

    def _stats_callback(self, cmd):
        self.stats = cmd


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_node(ros_context):
    node = NodeTestingNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    return node


@pytest.fixture
def roboclaw_node(ros_context):
    node = RoboclawNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    node.connect(dev_name='/dev/ttyACM0', baud_rate=115200, address=0x80, test_mode=True)
    return node


def test_forward_normal(test_node: NodeTestingNode, roboclaw_node: RoboclawNode):
    nodes = [test_node, roboclaw_node]
    _spin_for_secs(nodes, 1)     # Process stats subscriber messages

    start_m1_dist = test_node.stats.m1_enc_val
    start_m2_dist = test_node.stats.m2_enc_val

    m1_qpps, m2_qpps = 1000, 1000
    max_secs = 2
    qpps_delta = 0
    dist_delta = max(abs(m1_qpps), abs(m2_qpps)) // 2

    cmd = _create_speed_command(m1_qpps, m2_qpps, max_secs)
    test_node.speed_pub.publish(cmd)

    _spin_for_secs(nodes, 3)

    _check_stats(
        test_node.stats,
        0, 0, qpps_delta,
        start_m1_dist + (m1_qpps * max_secs),
        start_m2_dist + (m2_qpps * max_secs),
        dist_delta
    )


def test_reverse_normal(test_node: NodeTestingNode, roboclaw_node: RoboclawNode):
    nodes = [test_node, roboclaw_node]
    _spin_for_secs(nodes, 1)     # Process stats subscriber messages

    start_m1_dist = test_node.stats.m1_enc_val
    start_m2_dist = test_node.stats.m2_enc_val

    m1_qpps, m2_qpps = -2000, -2000
    max_secs = 2
    qpps_delta = 0
    dist_delta = max(abs(m1_qpps), abs(m2_qpps)) // 2

    cmd = _create_speed_command(m1_qpps, m2_qpps, max_secs)
    test_node.speed_pub.publish(cmd)

    _spin_for_secs(nodes, 3)

    _check_stats(
        test_node.stats,
        0, 0, qpps_delta,
        start_m1_dist + (m1_qpps * max_secs),
        start_m2_dist + (m2_qpps * max_secs),
        dist_delta
    )


def test_left_normal(test_node: NodeTestingNode, roboclaw_node: RoboclawNode):
    nodes = [test_node, roboclaw_node]
    _spin_for_secs(nodes, 1)     # Process stats subscriber messages

    start_m1_dist = test_node.stats.m1_enc_val
    start_m2_dist = test_node.stats.m2_enc_val

    m1_qpps, m2_qpps = 1000, -1000
    max_secs = 2
    qpps_delta = 0
    dist_delta = max(abs(m1_qpps), abs(m2_qpps)) // 2

    cmd = _create_speed_command(m1_qpps, m2_qpps, max_secs)
    test_node.speed_pub.publish(cmd)

    _spin_for_secs(nodes, 3)

    _check_stats(
        test_node.stats,
        0, 0, qpps_delta,
        start_m1_dist + (m1_qpps * max_secs),
        start_m2_dist + (m2_qpps * max_secs),
        dist_delta
    )


def test_right_normal(test_node: NodeTestingNode, roboclaw_node: RoboclawNode):
    nodes = [test_node, roboclaw_node]
    _spin_for_secs(nodes, 1)     # Process stats subscriber messages

    start_m1_dist = test_node.stats.m1_enc_val
    start_m2_dist = test_node.stats.m2_enc_val

    m1_qpps, m2_qpps = -1000, 1000
    max_secs = 2
    qpps_delta = 0
    dist_delta = max(abs(m1_qpps), abs(m2_qpps)) // 2

    cmd = _create_speed_command(m1_qpps, m2_qpps, max_secs)
    test_node.speed_pub.publish(cmd)

    _spin_for_secs(nodes, 3)

    _check_stats(
        test_node.stats,
        0, 0, qpps_delta,
        start_m1_dist + (m1_qpps * max_secs),
        start_m2_dist + (m2_qpps * max_secs),
        dist_delta
    )


def _create_speed_command(m1_qpps, m2_qpps, max_secs):
    cmd = SpeedCommand()
    cmd.m1_qpps = m1_qpps
    cmd.m2_qpps = m2_qpps
    cmd.max_secs = max_secs
    return cmd


def _check_stats(
    stats: Stats, m1_qpps: int, m2_qpps: int,
    qpps_delta: int, m1_val: int, m2_val: int, val_delta: int
):
    """Check actual stats values against expected values.
    Uses assertAlmostEqual comparison

    Args:
        m1_qpps (int): Expected motor 1 QPPS value
        m2_qpps (int): Expected motor 2 QPPS value
        qpps_delta (int): Allowed difference between expected & actual QPPS
        m1_val (int):  Expected motor 1 encoder value
        m2_val (int):  Expected motor 2 encoder value
        val_delta (int): Allowed difference between expected & actual encoder value
    """
    tests = [
        ("M1 QPPS", stats.m1_enc_qpps, m1_qpps, qpps_delta),
        ("M2 QPPS", stats.m2_enc_qpps, m2_qpps, qpps_delta),
        ("M1 encoder value", stats.m1_enc_val, m1_val, val_delta),
        ("M1 encoder value", stats.m2_enc_val, m2_val, val_delta),
    ]
    for label, actual_val, expected_val, delta in tests:
        assert expected_val - delta <= actual_val <= expected_val + delta, \
            f"{label} expected/delta: {expected_val}/{delta}, actual: {actual_val}"


def _spin_for_secs(nodes: List[Node], secs: float, timeout_sec: float = 0.001):
    start_time = time.perf_counter()
    while time.perf_counter() < start_time + secs:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=timeout_sec)
