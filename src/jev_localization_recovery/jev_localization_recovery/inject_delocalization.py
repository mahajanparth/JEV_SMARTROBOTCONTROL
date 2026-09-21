"""Perturb AMCL's estimate through initialpose; never move the robot."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped

from .localization_monitor import quaternion_yaw


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dx', type=float, default=0.0)
    parser.add_argument('--dy', type=float, default=0.0)
    parser.add_argument('--dyaw', type=float, default=0.0, help='radians')
    parser.add_argument('--pose-topic', default='', help='explicit AMCL pose topic for ambiguous graphs')
    parser.add_argument('--initialpose-topic', default='', help='explicit AMCL input topic')
    parser.add_argument('--variance', type=float, default=0.05, help='injected x/y/yaw diagonal covariance')
    parser.add_argument('--timeout', type=float, default=10.0)
    options, ros_args = parser.parse_known_args(args)
    if not all(math.isfinite(v) for v in (options.dx, options.dy, options.dyaw, options.variance, options.timeout)):
        parser.error('numeric arguments must be finite')
    if options.variance <= 0 or options.timeout <= 0:
        parser.error('variance and timeout must be positive')
    rclpy.init(args=ros_args)
    node = Node('inject_delocalization')
    pose = []
    subscription = publisher = None
    publisher_created_at = 0.0
    deadline = time.monotonic() + options.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            topics = node.get_topic_names_and_types()
            poses = [name for name, types in topics
                     if 'geometry_msgs/msg/PoseWithCovarianceStamped' in types and node.count_publishers(name) > 0
                     and (name == options.pose_topic if options.pose_topic else name.rsplit('/', 1)[-1] == 'amcl_pose')]
            if subscription is None and len(poses) == 1:
                infos = node.get_publishers_info_by_topic(poses[0])
                qos = qos_profile_sensor_data
                if infos and all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos):
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                subscription = node.create_subscription(PoseWithCovarianceStamped, poses[0],
                                                         lambda msg: pose.append(msg), qos)
                node.get_logger().info('Using current estimate from ' + poses[0])
            inputs = [name for name, types in topics
                      if 'geometry_msgs/msg/PoseWithCovarianceStamped' in types and node.count_subscribers(name) > 0
                      and (name == options.initialpose_topic if options.initialpose_topic else
                           name.rsplit('/', 1)[-1] == 'initialpose')]
            if publisher is None and len(inputs) == 1:
                publisher = node.create_publisher(PoseWithCovarianceStamped, inputs[0], 10)
                publisher_created_at = time.monotonic()
            if (pose and publisher is not None and publisher.get_subscription_count() > 0
                    and time.monotonic() - publisher_created_at >= 0.5
                    and node.get_clock().now().nanoseconds > 0):
                current = pose[-1]
                if not current.header.frame_id:
                    raise RuntimeError('AMCL pose has no frame')
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = current.header.frame_id
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.pose.position.x = current.pose.pose.position.x + options.dx
                msg.pose.pose.position.y = current.pose.pose.position.y + options.dy
                yaw = quaternion_yaw(current.pose.pose.orientation) + options.dyaw
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                for index in (0, 7, 35):
                    msg.pose.covariance[index] = options.variance
                # A short burst survives discovery/data-path races with AMCL's
                # best-effort subscriber. Every message contains the SAME offset.
                for _ in range(3):
                    msg.header.stamp = node.get_clock().now().to_msg()
                    publisher.publish(msg)
                    until = time.monotonic() + 0.2
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=0.02)
                node.get_logger().info('Injected AMCL offset dx={:.2f}, dy={:.2f}, dyaw={:.2f} into {} ({})'.format(
                    options.dx, options.dy, options.dyaw, publisher.topic_name, msg.header.frame_id))
                flush_until = time.monotonic() + 0.5
                while time.monotonic() < flush_until:
                    rclpy.spin_once(node, timeout_sec=0.05)
                return
            rclpy.spin_once(node, timeout_sec=0.1)
        raise RuntimeError('Timed out finding a unique AMCL estimate and initialpose subscriber; '
                           'start AMCL or specify --pose-topic and --initialpose-topic')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
