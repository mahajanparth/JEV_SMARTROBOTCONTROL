#!/usr/bin/env python3
"""Observe the running house demo and verify a real injected global recovery."""
import json
import argparse
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--provider', choices=['mock', 'jev'], default='mock')
    options = parser.parse_args()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = Node('jev_gazebo_recovery_check')
    states, events, commands, odometry = [], [], [], []
    node.create_subscription(String, '/jev_recovery/state', lambda m: states.append(json.loads(m.data)), 10)
    node.create_subscription(String, '/jev_recovery/events', lambda m: events.append(json.loads(m.data)), 10)
    node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
    node.create_subscription(Odometry, '/odom', odometry.append, 10)
    injection = None

    def wait_for(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
            if any(e['event'] == 'STOP' for e in events):
                raise RuntimeError('Recovery stopped: ' + json.dumps(events))
        raise RuntimeError('Timed out. Events: {}; latest state: {}'.format(events, states[-1:] ))

    try:
        wait_for(lambda: bool(states) and states[-1]['status'] == 'HEALTHY'
                 and states[-1]['machine'] == 'MONITORING' and bool(odometry), 30)
        baseline = states[-1]
        assert baseline.get('selector_provider', 'mock') == options.provider, 'Wrong decision provider is running'
        first = odometry[-1].pose.pose.position
        injection = subprocess.Popen([
            'ros2', 'run', 'jev_localization_recovery', 'inject_delocalization',
            '--dx', '4.0', '--dy', '4.0', '--dyaw', '1.5', '--variance', '0.005',
            '--ros-args', '-p', 'use_sim_time:=true',
        ])
        wait_for(lambda: any(e['event'] == 'RECOVERY SUCCESS' for e in events), 100)
        assert injection.wait(timeout=3) == 0
        success = next(e for e in events if e['event'] == 'RECOVERY SUCCESS')
        assert success['action'] == 'GLOBAL_RELOCALIZE', events
        assert success.get('provider', 'mock') == options.provider
        if options.provider == 'jev':
            assert success['decision']['provider'] == 'jev'
            assert success['decision']['model'].startswith('jev-')
        assert success['after']['status'] == 'HEALTHY'
        assert any(c.angular.z > 0 for c in commands)
        assert all(c.linear.x == 0 for c in commands)
        angle = 0.0
        last_yaw = None
        for sample in odometry:
            q = sample.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            if last_yaw is not None:
                angle += math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
            last_yaw = yaw
        assert angle > 5.8, angle
        last = odometry[-1].pose.pose.position
        assert math.hypot(last.x - first.x, last.y - first.y) < 0.1
        result = {'environment': 'Gazebo 11 / TurtleBot3 Burger / Jev house / real Nav2 AMCL',
                  'baseline_score': baseline['scan_map_score'], 'rotation_radians': angle,
                  'recovery': success}
        print(json.dumps(result, indent=2))
        report_path = '/ws/docs/jev-gazebo-validation.json' if options.provider == 'jev' else '/ws/docs/gazebo-validation.json'
        with open(report_path, 'w') as report:
            json.dump(result, report, indent=2)
    finally:
        if injection is not None and injection.poll() is None:
            injection.terminate()
            injection.wait(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
