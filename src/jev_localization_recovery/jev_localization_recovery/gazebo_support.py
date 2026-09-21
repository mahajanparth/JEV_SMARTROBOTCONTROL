"""Demo-only AMCL stationary updates and an expiring velocity relay for Gazebo."""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


class GazeboSupport(Node):
    def __init__(self):
        super().__init__('jev_gazebo_support')
        self.command = Twist()
        self.command_at = -math.inf
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/recovery/cmd_vel', self.on_command, 10)
        self.client = self.create_client(Empty, '/request_nomotion_update')
        self.future = None
        self.create_timer(1.0, self.update_pose)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self.relay, clock=self.wall_clock)

    def on_command(self, msg):
        if (not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z) or
                abs(msg.linear.x) > 0.15 or abs(msg.angular.z) > 0.6):
            self.command, self.command_at = Twist(), -math.inf
            return
        self.command = Twist()
        self.command.linear.x, self.command.angular.z = msg.linear.x, msg.angular.z
        self.command_at = time.monotonic()

    def relay(self):
        self.publisher.publish(self.command if time.monotonic() - self.command_at < 0.4 else Twist())

    def update_pose(self):
        if self.client.service_is_ready() and (self.future is None or self.future.done()):
            self.future = self.client.call_async(Empty.Request())


def main(args=None):
    rclpy.init(args=args)
    node = GazeboSupport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
