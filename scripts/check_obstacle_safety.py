#!/usr/bin/env python3
"""Known-pose Gazebo fixture: insert an obstacle during live Jev navigation."""
import json
import math
from pathlib import Path
import time
from urllib.request import urlopen,Request
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from obstacle_fixture import pose,insert,remove


def main():
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=rclpy.create_node('jev_obstacle_validation')
    pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    output=[]
    node.create_subscription(Twist,'/cmd_vel',lambda m:output.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    def state():return json.load(urlopen('http://localhost:8765/api/state',timeout=2))
    def command(name,**kw):
        urlopen(Request('http://localhost:8765/api/command',data=json.dumps(dict(command=name,**kw)).encode(),headers={'Content-Type':'application/json'}),timeout=2).close()
    report={'fixture':'Known-pose AMCL, live Jev navigation, static obstacle insertion','success':False}
    obstacle=None;initialized=False;blocked=False;sent=False;blocked_at=None;at=None
    try:
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05);s=state();now=time.monotonic()
            if s.get('phase')=='IDLE' and not sent:command('start',seed=7);sent=True
            if s.get('phase')=='INITIALIZING' and not initialized:
                initial=PoseWithCovarianceStamped();initial.header.frame_id='map';initial.pose.pose=pose(node)
                initial.pose.covariance[0]=initial.pose.covariance[7]=initial.pose.covariance[35]=.005
                for _ in range(3):
                    initial.header.stamp=node.get_clock().now().to_msg();pub.publish(initial);rclpy.spin_once(node,timeout_sec=.1)
                initialized=True
            if s.get('phase') in ('HELP','STOPPED') and not blocked:raise RuntimeError(s.get('reason'))
            if obstacle is None and s.get('phase')=='NAVIGATING' and output and output[-1][1]>.04:
                try:obstacle=insert(node,.4)
                except RuntimeError as exc:
                    if 'corridor' in str(exc):continue
                    raise
                at=time.monotonic();report['obstacle_xy']=obstacle
            if obstacle is not None:
                robot=pose(node)
                # Conservative box circumscribed radius; positive implies separation.
                clearance=math.hypot(robot.position.x-obstacle[0],robot.position.y-obstacle[1])-.18-math.sqrt(2)*.1
                report['minimum_geometric_clearance_m']=min(report.get('minimum_geometric_clearance_m',math.inf),clearance)
                safety=s.get('safety',{})
                if safety.get('status')=='BLOCKED' and not blocked:
                    blocked=True;blocked_at=time.monotonic();report['block_detected_after_insertion_s']=blocked_at-at
                    report['jev_calls_at_block']=s['jev_calls']
                if blocked and time.monotonic()-blocked_at>2:
                    assert output
                    if safety.get('status')=='BLOCKED':
                        assert s['owner']=='NONE',s['owner']
                        assert all(abs(v)<.001 for v in safety['applied'])
                    assert report['minimum_geometric_clearance_m']>0
                    if s['phase']!='NAVIGATING':
                        assert s['jev_calls']<=report['jev_calls_at_block']+1,'Repeated Jev requests while blocked'
                    report.update(success=True,phase=s['phase'],safety=safety,final_owner=s['owner'])
                    break
                if time.monotonic()-at>15:raise RuntimeError('Obstacle was avoided or did not trigger a safety stop within 15 s')
        else:raise RuntimeError('Obstacle validation timed out')
    except Exception as exc:report['error']=str(exc);raise
    finally:
        try:command('stop')
        except Exception:pass
        if obstacle is not None:
            try:remove(node)
            except Exception:pass
        Path('/ws/docs/mission-runs/obstacle-check.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
