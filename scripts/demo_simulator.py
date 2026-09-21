#!/usr/bin/env python3
"""Small headless differential-drive simulator; localization is supplied by AMCL.

Publishes a latched occupancy grid, analytic 360-degree lidar, odometry, and TF.
The --initialpose mode sends a real AMCL initialization request; it never publishes
an AMCL estimate or implements AMCL services.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


WIDTH, HEIGHT, RESOLUTION = 12.0, 10.0, 0.05
OBSTACLES = [
    (0.0, 0.0, 12.0, 0.1), (0.0, 9.9, 12.0, 10.0),
    (0.0, 0.0, 0.1, 10.0), (11.9, 0.0, 12.0, 10.0),
    (4.0, 2.0, 4.7, 5.5), (7.5, 6.0, 9.4, 6.8), (2.0, 7.0, 3.0, 8.0),
]


def quaternion(rotation, yaw):
    rotation.z, rotation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def ray_rectangle(x, y, dx, dy, rectangle):
    """Nearest nonnegative ray intersection with an axis-aligned rectangle."""
    near, far = -math.inf, math.inf
    for origin, direction, lower, upper in (
            (x, dx, rectangle[0], rectangle[2]),
            (y, dy, rectangle[1], rectangle[3])):
        if abs(direction) < 1e-12:
            if origin < lower or origin > upper:
                return math.inf
            continue
        a, b = (lower - origin) / direction, (upper - origin) / direction
        near, far = max(near, min(a, b)), min(far, max(a, b))
        if near > far:
            return math.inf
    if far < 0:
        return math.inf
    return max(0.0, near)


class DemoSimulator(Node):
    def __init__(self):
        super().__init__('jev_demo_simulator')
        self.x, self.y, self.yaw = 2.0, 2.0, 0.0
        self.linear = self.angular = 0.0
        self.command_at = -math.inf
        self.last_tick = time.monotonic()
        self.last_scan = -math.inf
        self.colliding = False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.truth_pub = self.create_publisher(PoseStamped, '/demo/ground_truth', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        # Perfect odometry has no jitter. Ask the real AMCL node to process a
        # stationary scan periodically so its published estimate remains fresh.
        self.nomotion = self.create_client(Empty, '/request_nomotion_update')
        self.nomotion_future = None
        self.create_timer(0.5, self.request_stationary_update)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        laser_tf = TransformStamped()
        laser_tf.header.frame_id, laser_tf.child_frame_id = 'base_link', 'laser'
        laser_tf.header.stamp = self.get_clock().now().to_msg()
        laser_tf.transform.translation.x = 0.12
        laser_tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(laser_tf)
        self.publish_map()
        self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info('Simulator ready: /map /scan /odom /cmd_vel; truth=(2,2,0).')

    def request_stationary_update(self):
        if (self.nomotion.service_is_ready()
                and (self.nomotion_future is None or self.nomotion_future.done())):
            self.nomotion_future = self.nomotion.call_async(Empty.Request())

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = RESOLUTION
        grid.info.width, grid.info.height = round(WIDTH / RESOLUTION), round(HEIGHT / RESOLUTION)
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        for low_x, low_y, high_x, high_y in OBSTACLES:
            for iy in range(round(low_y / RESOLUTION), round(high_y / RESOLUTION)):
                for ix in range(round(low_x / RESOLUTION), round(high_x / RESOLUTION)):
                    grid.data[iy * grid.info.width + ix] = 100
        self.map_pub.publish(grid)

    def command(self, message):
        if not all(math.isfinite(value) for value in (message.linear.x, message.angular.z)):
            self.linear = self.angular = 0.0
            return
        self.linear = max(-0.25, min(0.25, message.linear.x))
        self.angular = max(-0.6, min(0.6, message.angular.z))
        self.command_at = time.monotonic()

    def clear(self, x, y):
        radius = 0.18
        return all(math.hypot(x - max(a, min(x, c)), y - max(b, min(y, d))) > radius
                   for a, b, c, d in OBSTACLES)

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(0.1, now - self.last_tick), now
        if now - self.command_at > 0.35:
            self.linear = self.angular = 0.0
        heading = self.yaw + self.angular * dt / 2.0
        next_x, next_y = self.x + self.linear * dt * math.cos(heading), self.y + self.linear * dt * math.sin(heading)
        blocked = not self.clear(next_x, next_y)
        if not blocked:
            self.x, self.y = next_x, next_y
        elif not self.colliding:
            self.get_logger().warning('Simulated contact: translation stopped.')
        self.colliding = blocked
        self.yaw = math.atan2(math.sin(self.yaw + self.angular * dt), math.cos(self.yaw + self.angular * dt))
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
        transform.header.stamp = stamp
        transform.transform.translation.x, transform.transform.translation.y = self.x - 2.0, self.y - 2.0
        quaternion(transform.transform.rotation, self.yaw)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id, odom.header.stamp = 'odom', 'base_link', stamp
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x - 2.0, self.y - 2.0
        quaternion(odom.pose.pose.orientation, self.yaw)
        odom.twist.twist.linear.x = 0.0 if blocked else self.linear
        odom.twist.twist.angular.z = self.angular
        self.odom_pub.publish(odom)
        truth = PoseStamped()
        truth.header.frame_id, truth.header.stamp = 'map', stamp
        truth.pose.position.x, truth.pose.position.y = self.x, self.y
        quaternion(truth.pose.orientation, self.yaw)
        self.truth_pub.publish(truth)
        if now - self.last_scan >= 0.095:
            self.last_scan = now
            self.publish_scan(stamp)

    def publish_scan(self, stamp):
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser', stamp
        scan.angle_min, scan.angle_increment = -math.pi, math.pi / 180.0
        scan.angle_max = scan.angle_min + 359 * scan.angle_increment
        scan.range_min, scan.range_max, scan.scan_time = 0.05, 15.0, 0.1
        laser_x, laser_y = self.x + 0.12 * math.cos(self.yaw), self.y + 0.12 * math.sin(self.yaw)
        ranges = []
        for i in range(360):
            angle = self.yaw + scan.angle_min + i * scan.angle_increment
            distance = min(ray_rectangle(laser_x, laser_y, math.cos(angle), math.sin(angle), rectangle)
                           for rectangle in OBSTACLES)
            ranges.append(distance if distance < scan.range_max else math.inf)
        scan.ranges = ranges
        self.scan_pub.publish(scan)


def inject_initialpose(values, variance):
    node = Node('jev_demo_pose_injector')
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    try:
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError('No AMCL /initialpose subscriber discovered')
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = node.get_clock().now().to_msg()
        message.pose.pose.position.x, message.pose.pose.position.y = values[:2]
        quaternion(message.pose.pose.orientation, values[2])
        for index in (0, 7, 35):
            message.pose.covariance[index] = variance
        publisher.publish(message)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        print('Sent /initialpose to real AMCL: {}'.format(values), flush=True)
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initialpose', type=float, nargs=3, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--variance', type=float, default=0.005)
    args = parser.parse_args()
    rclpy.init(args=[])
    simulator = None
    try:
        if args.initialpose:
            inject_initialpose(args.initialpose, args.variance)
        else:
            simulator = DemoSimulator()
            rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        if simulator is not None:
            simulator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
