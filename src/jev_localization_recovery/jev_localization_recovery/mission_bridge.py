"""Exclusive command ownership and a shared, fail-closed obstacle filter."""
import json
import math
import time
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from .gazebo_support import GazeboSupport
from .obstacle_safety import ObstacleSafety, SafetyConfig, scan_points

class MissionBridge(GazeboSupport):
    def __init__(self):
        super().__init__()
        values={}
        for key,value in vars(SafetyConfig()).items():
            self.declare_parameter('safety_'+key,value)
            values[key]=float(self.get_parameter('safety_'+key).value)
        self.safety=ObstacleSafety(SafetyConfig(**values))
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.owner,self.owner_at='NONE',0.;self.commands={}
        self.scan_at=0.;self.scan_stamp=None;self.points=None;self.scan_fault='Waiting for scan and TF'
        self.odom_at=0.;self.measured=(0.,0.);self.odom_stamp=None
        self.safety_pub=self.create_publisher(String,'/demo/safety',10)
        self.create_subscription(String,'/demo/owner',self.on_owner,10)
        self.create_subscription(String,'/demo/replan_ready',self.on_replan_ready,10)
        self.create_subscription(Twist,'/navigation/cmd_vel',lambda m:self.store('NAV',m),10)
        from rclpy.qos import qos_profile_sensor_data
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)

    def on_replan_ready(self,msg):
        # Discard only the old trajectory, never sensor faults or footprint checks.
        # Subsequent commands still undergo the full independent envelope test.
        now=time.monotonic()
        if (msg.data=='CHECK_NEW_ROUTE' and self.owner=='NONE' and now-self.owner_at<.4 and
                now-self.odom_at<self.safety.config.sensor_timeout and
                abs(self.measured[0])<.01 and abs(self.measured[1])<.03):
            self.commands.clear();self.safety.probe=(0.,0.);self.safety.clear_since=None

    def on_owner(self,msg):
        if msg.data!=self.owner:self.commands.clear()
        self.owner,self.owner_at=msg.data,time.monotonic()

    def on_command(self,msg):self.store('RECOVERY',msg)

    def store(self,owner,msg):
        vals=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if (all(math.isfinite(v) for v in vals) and abs(msg.linear.x)<=.16 and
                abs(msg.angular.z)<=.61 and not any(vals[1:5])):
            self.commands[owner]=(msg,time.monotonic())
        else:self.commands.pop(owner,None)

    def on_odom(self,msg):
        v,w=msg.twist.twist.linear.x,msg.twist.twist.angular.z
        if not all(math.isfinite(x) for x in (v,w)) or abs(v)>.5 or abs(w)>2.:self.odom_at=0.;return
        self.measured=(v,w);self.odom_at=time.monotonic()
        self.odom_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def scan(self,msg):
        self.scan_at=time.monotonic();self.scan_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        try:
            tf=self.tf.lookup_transform('base_footprint',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            p,q=tf.transform.translation,tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.points=scan_points(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,(p.x,p.y,yaw))
            self.scan_fault=None
        except Exception:
            self.points=None;self.scan_fault='Invalid scan coverage or scan-time TF unavailable'

    def relay(self):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds*1e-9
        command,at=self.commands.get(self.owner,(Twist(),0.))
        owned=self.owner in ('NAV','RECOVERY') and now-self.owner_at<.4
        valid=owned and now-at<.4
        request=(command.linear.x,command.angular.z) if valid else (0.,0.)
        scan_age=ros_now-self.scan_stamp if self.scan_stamp is not None else None
        odom_age=ros_now-self.odom_stamp if self.odom_stamp is not None else None
        fault=self.scan_fault
        if now-self.odom_at>self.safety.config.sensor_timeout or odom_age is None or not -.1<=odom_age<=self.safety.config.sensor_timeout:
            fault='Odometry stale or unavailable'
        if scan_age is None or not -.1<=scan_age<=self.safety.config.sensor_timeout:
            fault='Scan timestamp stale or unavailable'
        age=max(now-self.scan_at,max(0.,scan_age or 0.))
        state=self.safety.evaluate(request,self.measured,self.points,age,now,fault, self.owner if owned else 'NONE')
        if state['latched'] or not valid:self.commands.clear()
        applied=Twist()
        if valid and not state['latched']:
            applied.linear.x,applied.angular.z=state['applied']
        state['applied']=[applied.linear.x,applied.angular.z]
        state.update(owner=self.owner,scan_age_seconds=scan_age,odom_age_seconds=odom_age,
                     command_valid=valid,measured=list(self.measured))
        self.publisher.publish(applied)
        self.safety_pub.publish(String(data=json.dumps(state,allow_nan=False)))

def main(args=None):
    rclpy.init(args=args);node=MissionBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
