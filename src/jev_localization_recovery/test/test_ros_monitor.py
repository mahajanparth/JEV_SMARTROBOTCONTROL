"""Monitor smoke test over real ROS messages, DDS discovery, and TF transport.

Run in an isolated ROS domain; no simulator, AMCL, or motion controller is needed.
The pure algorithm tests remain runnable on machines without ROS installed.
"""
import json
import math
import os
import time

import pytest

rclpy = pytest.importorskip('rclpy', reason='ROS transport test requires ROS 2')
pytest.importorskip('tf2_ros', reason='ROS transport test requires tf2_ros')

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from jev_localization_recovery.recovery_node import RecoveryNode


def test_monitor_real_transport_health_discovery_and_unknown():
    # A separate domain prevents discovery from binding to a running robot.
    # Namespaces also make the observed topic names explicit in assertion failures.
    namespace = '/jev_monitor_transport_{}'.format(os.getpid())
    rclpy.init(args=['--ros-args', '--remap', '__ns:=' + namespace], domain_id=197)
    source = monitor = executor = None
    try:
        source = Node('monitor_test_source')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = source.create_publisher(OccupancyGrid, 'map', latched)
        pose_pub = source.create_publisher(PoseWithCovarianceStamped, 'amcl_pose',
                                           qos_profile_sensor_data)
        scan_pub = source.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        states = []

        def observe(message):
            # Reject nonstandard JSON numbers such as NaN instead of hiding them.
            def invalid_constant(value):
                raise AssertionError('Invalid JSON numeric constant: ' + value)
            states.append(json.loads(message.data, parse_constant=invalid_constant))

        source.create_subscription(String, 'jev_recovery/state', observe, 10)

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 200
        grid.info.resolution = 0.1
        grid.info.origin.position.x = grid.info.origin.position.y = -10.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (200 * 200)
        # A circular wall around an offset, rotated laser. Compute the fixture
        # geometry independently of the production transform/scoring helpers.
        for i in range(360):
            angle = -math.pi + i * math.pi / 180.0 + 0.17
            x, y = 0.25 + 2.0 * math.cos(angle), -0.1 + 2.0 * math.sin(angle)
            ix, iy = math.floor((x + 10.0) / 0.1), math.floor((y + 10.0) / 0.1)
            grid.data[iy * 200 + ix] = 100
        grid.header.stamp = source.get_clock().now().to_msg()
        # Publish before the monitor subscribes, exercising transient-local QoS.
        map_pub.publish(grid)

        monitor = RecoveryNode()
        executor = SingleThreadedExecutor()
        executor.add_node(source)
        executor.add_node(monitor)
        sample = {'x': 0.0, 'variance': 0.01}

        def publish_measurements():
            stamp = source.get_clock().now().to_msg()
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id, pose.header.stamp = 'map', stamp
            pose.pose.pose.position.x = sample['x']
            pose.pose.pose.orientation.w = 1.0
            for index in (0, 7, 35):
                pose.pose.covariance[index] = sample['variance']
            scan = LaserScan()
            scan.header.frame_id, scan.header.stamp = 'test_laser', stamp
            scan.angle_min = -math.pi
            scan.angle_increment = math.pi / 180.0
            scan.angle_max = scan.angle_min + 359 * scan.angle_increment
            scan.range_min, scan.range_max = 0.05, 10.0
            scan.ranges = [2.0] * 360
            pose_pub.publish(pose)
            scan_pub.publish(scan)

        def wait_for(predicate, publish=False, timeout=10.0):
            start = len(states)
            deadline = time.monotonic() + timeout
            next_publish = 0.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if publish and now >= next_publish:
                    publish_measurements()
                    next_publish = now + 0.05
                executor.spin_once(timeout_sec=0.02)
                for state in states[start:]:
                    if predicate(state):
                        return state
            pytest.fail('Timed out waiting for transported state; last states: {}'.format(states[-3:]))

        unknown = wait_for(lambda state: not state['data_ready'])
        assert unknown['status'] == 'DEGRADED'
        assert unknown['scan_map_score'] is None
        assert unknown['reason'].startswith('UNKNOWN:')

        # Valid sensor data without laser TF must still fail closed.
        missing_tf = wait_for(lambda state: (
            not state['data_ready'] and state['pose_uncertainty'] is not None
            and len(state['topics']) >= 3), publish=True)
        assert missing_tf['scan_map_score'] is None
        assert missing_tf['reason'].startswith('UNKNOWN:')

        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'test_laser'
        transform.header.stamp = source.get_clock().now().to_msg()
        transform.transform.translation.x, transform.transform.translation.y = 0.25, -0.1
        transform.transform.rotation.z = math.sin(0.17 / 2)
        transform.transform.rotation.w = math.cos(0.17 / 2)
        broadcaster = StaticTransformBroadcaster(source)
        broadcaster.sendTransform(transform)

        healthy = wait_for(lambda state: state['status'] == 'HEALTHY', publish=True)
        assert healthy['data_ready'] is True
        assert healthy['scan_map_score'] == pytest.approx(1.0)
        assert healthy['pose_uncertainty'] == pytest.approx(0.025)
        assert 1.7 < healthy['min_obstacle_distance'] < 1.8
        assert healthy['rear_clearance'] is not None
        assert healthy['topics'] == {
            'pose': namespace + '/amcl_pose',
            'scan': namespace + '/scan',
            'map': namespace + '/map',
        }
        assert healthy['machine'] == 'MONITORING'
        assert healthy['current_action'] == 'NONE'

        sample['variance'] = 0.3
        degraded = wait_for(lambda state: (
            state['status'] == 'DEGRADED' and state['data_ready']
            and state['pose_uncertainty'] == pytest.approx(0.75)), publish=True)
        assert degraded['scan_map_score'] == pytest.approx(1.0)

        sample['variance'] = 1.0
        lost_covariance = wait_for(lambda state: (
            state['status'] == 'LOST'
            and state['pose_uncertainty'] == pytest.approx(2.5)), publish=True)
        assert lost_covariance['scan_map_score'] == pytest.approx(1.0)

        sample.update(x=6.0, variance=0.01)
        lost_pose = wait_for(lambda state: (
            state['status'] == 'LOST' and state['scan_map_score'] == 0.0
            and state['pose_uncertainty'] == pytest.approx(0.025)), publish=True)
        assert lost_pose['pose_jump_distance'] == pytest.approx(6.0)
        assert lost_pose['data_ready'] is True

        # A previously valid estimate becomes UNKNOWN when scan traffic stops.
        stale = wait_for(lambda state: (
            not state['data_ready'] and 'stale' in state['reason'].lower()))
        assert stale['status'] == 'DEGRADED'
        assert stale['scan_map_score'] is None
    finally:
        if executor is not None:
            executor.shutdown()
        if monitor is not None:
            monitor.destroy_node()
        if source is not None:
            source.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
