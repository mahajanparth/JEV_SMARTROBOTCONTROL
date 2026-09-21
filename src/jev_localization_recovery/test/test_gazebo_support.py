"""Gazebo's command relay must expire even when simulated time is stopped."""
import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from jev_localization_recovery import gazebo_support


def test_relay_expires_and_rejects_invalid_or_excessive_commands(monkeypatch):
    rclpy.init(args=[], domain_id=199)
    node = gazebo_support.GazeboSupport()
    sent = []
    clock = [10.0]
    monkeypatch.setattr(gazebo_support.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(node.publisher, 'publish', sent.append)
    try:
        command = Twist()
        command.angular.z = .35
        node.on_command(command)
        node.relay()
        assert sent[-1].angular.z == .35
        clock[0] += .41
        node.relay()
        assert sent[-1].angular.z == 0
        for speed in (float('nan'), 1.0):
            command.angular.z = speed
            node.on_command(command)
            node.relay()
            assert sent[-1].angular.z == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
