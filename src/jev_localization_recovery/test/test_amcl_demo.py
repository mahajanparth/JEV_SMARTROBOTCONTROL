"""Opt-in integration check against scripts/run_amcl_demo.sh and real Nav2 AMCL.

JEV_RUN_AMCL_DEMO=1 python3 -m pytest -s test/test_amcl_demo.py
Run the simulator/AMCL bringup first; this test starts its own monitor without
enabling recovery, injects /initialpose, then restores the correct estimate.
"""
import json
import math
import os
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('JEV_RUN_AMCL_DEMO') != '1',
                                reason='Opt-in real AMCL demo; set JEV_RUN_AMCL_DEMO=1')
rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from jev_localization_recovery.recovery_node import RecoveryNode


def test_real_amcl_wrong_initialpose_reduces_monitor_score():
    rclpy.init(args=[], domain_id=87)
    source = monitor = executor = None
    try:
        source = Node('jev_real_amcl_verifier')
        initialpose = source.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        states, estimates = [], []
        source.create_subscription(String, '/jev_recovery/state',
                                   lambda message: states.append(json.loads(message.data)), 10)
        source.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', estimates.append,
                                   qos_profile_sensor_data)
        monitor = RecoveryNode()
        executor = SingleThreadedExecutor()
        executor.add_node(source)
        executor.add_node(monitor)

        def wait_for(predicate, timeout=20.0):
            start, deadline = len(states), time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                for state in states[start:]:
                    if predicate(state):
                        return state
            pytest.fail('Real AMCL monitor timeout; last states: {}'.format(states[-3:]))

        def inject(x, y, yaw):
            message = PoseWithCovarianceStamped()
            message.header.frame_id = 'map'
            message.header.stamp = source.get_clock().now().to_msg()
            message.pose.pose.position.x, message.pose.pose.position.y = x, y
            message.pose.pose.orientation.z = math.sin(yaw / 2)
            message.pose.pose.orientation.w = math.cos(yaw / 2)
            for index in (0, 7, 35):
                message.pose.covariance[index] = 0.005
            initialpose.publish(message)

        healthy = wait_for(lambda state: state['status'] == 'HEALTHY')
        assert estimates, 'AMCL must supply real estimates on /amcl_pose'
        assert healthy['scan_map_score'] >= 0.65
        assert healthy['topics']['pose'] == '/amcl_pose'
        assert healthy['topics']['map'] == '/map'
        assert healthy['topics']['scan'] == '/scan'
        assert initialpose.get_subscription_count() > 0
        inject(9.8, 3.5, 1.0)
        lost = wait_for(lambda state: (
            state['data_ready'] and state['status'] == 'LOST'
            and state['scan_map_score'] < 0.25))
        assert lost['scan_map_score'] < healthy['scan_map_score'] - 0.4
        assert lost['pose_jump_distance'] > 5.0
        print('\nREAL_AMCL baseline score={:.3f}, wrong initialpose score={:.3f}, jump={:.2f}m'.format(
            healthy['scan_map_score'], lost['scan_map_score'], lost['pose_jump_distance']))
        inject(2.0, 2.0, 0.0)
        restored = wait_for(lambda state: state['status'] == 'HEALTHY')
        assert restored['scan_map_score'] >= 0.65
    finally:
        if executor is not None:
            executor.shutdown()
        if monitor is not None:
            monitor.destroy_node()
        if source is not None:
            source.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_real_amcl_production_injection_global_recovery_succeeds():
    """Exercise production selection, AMCL global reset, spin, and evaluation."""
    rclpy.init(args=['--ros-args', '-p', 'enable_recovery:=true'], domain_id=87)
    observer = monitor = executor = injection = None
    try:
        observer = Node('jev_real_recovery_verifier')
        states, events, commands, truth = [], [], [], []
        observer.create_subscription(String, '/jev_recovery/state',
                                     lambda message: states.append(json.loads(message.data)), 10)
        observer.create_subscription(String, '/jev_recovery/events',
                                     lambda message: events.append(json.loads(message.data)), 10)
        observer.create_subscription(Twist, '/cmd_vel', commands.append, 10)
        observer.create_subscription(PoseStamped, '/demo/ground_truth', truth.append, 10)
        monitor = RecoveryNode()
        executor = SingleThreadedExecutor()
        executor.add_node(observer)
        executor.add_node(monitor)

        def wait_for(predicate, timeout):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                if predicate():
                    return
                if any(event['event'] == 'STOP' for event in events):
                    pytest.fail('Recovery latched STOP: {}'.format(events))
            pytest.fail('Recovery timeout; events={}, last states={}'.format(events, states[-3:]))

        wait_for(lambda: bool(states) and states[-1]['status'] == 'HEALTHY' and bool(truth), 20.0)
        baseline = states[-1]
        initial_truth = truth[-1]
        injection = subprocess.Popen([
            'ros2', 'run', 'jev_localization_recovery', 'inject_delocalization',
            '--dx', '7.8', '--dy', '1.5', '--dyaw', '1.0', '--variance', '0.005',
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            env=dict(os.environ, ROS_DOMAIN_ID='87'))
        wait_for(lambda: any(event['event'] == 'DECISION'
                            and event['selected_action'] == 'GLOBAL_RELOCALIZE' for event in events), 20.0)
        wait_for(lambda: any(event['event'] == 'RECOVERY SUCCESS' for event in events), 70.0)
        output, _ = injection.communicate(timeout=2.0)
        assert injection.returncode == 0, output
        success = next(event for event in events if event['event'] == 'RECOVERY SUCCESS')
        assert success['action'] == 'GLOBAL_RELOCALIZE', events
        assert success['after']['status'] == 'HEALTHY'
        assert success['after']['scan_map_score'] >= 0.65
        assert success['after']['scan_map_score'] > success['before']['scan_map_score'] + 0.1
        assert any(command.angular.z > 0.0 for command in commands)
        assert all(command.linear.x == 0.0 for command in commands)
        assert abs(truth[-1].pose.position.x - initial_truth.pose.position.x) < 0.01
        assert abs(truth[-1].pose.position.y - initial_truth.pose.position.y) < 0.01
        assert commands[-1].angular.z == 0.0
        print('\nREAL_AMCL production recovery: baseline={:.3f}, before={:.3f}, after={:.3f}, '
              'action={}, angular commands={}'.format(
                  baseline['scan_map_score'], success['before']['scan_map_score'],
                  success['after']['scan_map_score'], success['action'],
                  sum(command.angular.z > 0.0 for command in commands)))
        print(output.strip())
    finally:
        if injection is not None and injection.poll() is None:
            injection.terminate()
            injection.wait(timeout=3.0)
        if monitor is not None and rclpy.ok():
            monitor.publish_velocity(0.0, 0.0)
        if executor is not None:
            executor.shutdown()
        if monitor is not None:
            monitor.destroy_node()
        if observer is not None:
            observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
