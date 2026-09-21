#!/usr/bin/env python3
"""Insert/remove one test-owned Gazebo obstacle; never changes AMCL estimates."""
import argparse
import math
import time
import rclpy
from gazebo_msgs.srv import GetEntityState, SpawnEntity, DeleteEntity
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

NAME='jev_safety_test_obstacle'

def call(node,kind,service,request):
    client=node.create_client(kind,service)
    try:
        if not client.wait_for_service(timeout_sec=5.):raise RuntimeError(service+' unavailable')
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=5.)
        if not future.done():raise RuntimeError(service+' timed out')
        result=future.result()
        if not result.success:raise RuntimeError(service+' rejected request')
        return result
    finally:node.destroy_client(client)


def pose(node):
    req=GetEntityState.Request();req.name='turtlebot3_burger';req.reference_frame='world'
    return call(node,GetEntityState,'/get_entity_state',req).state.pose


def insert(node,distance=.65):
    if not .4<=distance<=1.5:raise ValueError('Distance must be 0.4..1.5 metres')
    scans=[]
    sub=node.create_subscription(LaserScan,'/scan',scans.append,qos_profile_sensor_data)
    deadline=time.monotonic()+3
    try:
        while not scans and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if not scans:raise RuntimeError('No scan to verify obstacle spawn clearance')
        scan=scans[-1]
        stamp=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
        if abs(node.get_clock().now().nanoseconds*1e-9-stamp)>.5:raise RuntimeError('Stale scan')
        front=[r for i,r in enumerate(scan.ranges) if abs(math.atan2(math.sin(scan.angle_min+i*scan.angle_increment),math.cos(scan.angle_min+i*scan.angle_increment)))<.4]
        if not front or any(math.isnan(r) or r<distance+.3 for r in front):
            raise RuntimeError('Obstacle spawn corridor is not clear')
    finally:node.destroy_subscription(sub)
    robot=pose(node);q=robot.orientation;yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
    x,y=robot.position.x+distance*math.cos(yaw),robot.position.y+distance*math.sin(yaw)
    req=SpawnEntity.Request();req.name=NAME;req.reference_frame='world'
    req.initial_pose.position.x=x;req.initial_pose.position.y=y;req.initial_pose.position.z=.25
    req.initial_pose.orientation.w=1.
    req.xml='''<sdf version="1.6"><model name="jev_safety_test_obstacle"><static>true</static><link name="body"><collision name="collision"><geometry><box><size>0.2 0.2 0.5</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.2 0.2 0.5</size></box></geometry><material><ambient>1 0.2 0.1 1</ambient></material></visual></link></model></sdf>'''
    call(node,SpawnEntity,'/spawn_entity',req)
    return x,y


def remove(node):
    req=DeleteEntity.Request();req.name=NAME
    call(node,DeleteEntity,'/delete_entity',req)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['insert','remove']);parser.add_argument('--distance',type=float,default=.65)
    args=parser.parse_args()
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=rclpy.create_node('jev_obstacle_fixture')
    try:
        if args.action=='insert':print('Obstacle position:',insert(node,args.distance))
        else:remove(node);print('Test obstacle removed')
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
