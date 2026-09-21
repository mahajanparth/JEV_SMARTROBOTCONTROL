"""Exclusive, expiring velocity ownership and AMCL stationary updates."""
import json
import math
import time
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .gazebo_support import GazeboSupport

class MissionBridge(GazeboSupport):
    def __init__(self):
        super().__init__()
        self.owner, self.owner_at = 'NONE', 0.
        self.commands = {}
        self.scan_at, self.nearest = 0., None
        self.create_subscription(String, '/demo/owner', self.on_owner, 10)
        self.create_subscription(Twist, '/navigation/cmd_vel', lambda m: self.store('NAV',m), 10)
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)

    def on_owner(self, msg):
        if msg.data != self.owner:
            self.commands.clear()
        self.owner, self.owner_at = msg.data, time.monotonic()

    def on_command(self,msg):
        self.store('RECOVERY',msg)

    def store(self,owner,msg):
        if all(math.isfinite(v) for v in (msg.linear.x,msg.angular.z)) and abs(msg.linear.x)<=.16 and abs(msg.angular.z)<=.61:
            self.commands[owner] = (msg,time.monotonic())
        else:
            self.commands.pop(owner,None)

    def scan(self,msg):
        now_ros = self.get_clock().now().nanoseconds*1e-9
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        valid=[r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        self.nearest=min(valid) if valid and -.1 <= now_ros-stamp < .75 else None
        self.scan_at=time.monotonic()

    def relay(self):
        now=time.monotonic()
        msg,at=self.commands.get(self.owner,(Twist(),0.))
        safe=(now-self.owner_at<.4 and now-at<.4 and now-self.scan_at<.75 and
              self.nearest is not None and self.nearest>.20)
        self.publisher.publish(msg if safe else Twist())

def main(args=None):
    rclpy.init(args=args)
    node=MissionBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
