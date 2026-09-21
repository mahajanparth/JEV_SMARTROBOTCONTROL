#!/usr/bin/env python3
"""Focused live supervision check with an explicitly known-pose AMCL test fixture.

This isolates navigation supervision; it is NOT an unknown-start localization test.
The known pose is supplied to AMCL only by this test, never by mission production code.
"""
import json
from pathlib import Path
import time
from urllib.request import urlopen, Request
import rclpy
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist


def main():
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=rclpy.create_node('supervision_known_pose_test')
    pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
    client=node.create_client(GetEntityState,'/get_entity_state')
    moved=[False]
    node.create_subscription(Twist,'/cmd_vel',lambda m:moved.__setitem__(0,True) if m.linear.x>.02 else None,10)
    def read():return json.load(urlopen('http://localhost:8765/api/state',timeout=2))
    def command(value):
        urlopen(Request('http://localhost:8765/api/command',data=json.dumps(value).encode(),headers={'Content-Type':'application/json'}),timeout=2).close()
    end=time.monotonic()+120; initialized=False; sent=False; state={};report={'fixture':'known-pose AMCL; navigation supervision only','success':False}
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            try:state=read()
            except Exception:continue
            if state.get('phase')=='IDLE' and not sent:
                command(dict(command='start',seed=7));sent=True
            if state.get('phase')=='INITIALIZING' and not initialized and client.service_is_ready():
                req=GetEntityState.Request();req.name='turtlebot3_burger';req.reference_frame='world'
                future=client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                assert future.done() and future.result().success
                pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.pose.pose=future.result().state.pose
                pose.pose.covariance[0]=pose.pose.covariance[7]=pose.pose.covariance[35]=.005
                for _ in range(3):
                    pose.header.stamp=node.get_clock().now().to_msg();pub.publish(pose)
                    rclpy.spin_once(node,timeout_sec=.2)
                initialized=True
            if state.get('phase') in ('HELP','STOPPED'):raise RuntimeError(state.get('reason'))
            events=state.get('events',[])
            requests=[e for e in events if e['event']=='JEV_REQUEST' and e.get('purpose')=='NAVIGATION_SUPERVISION']
            continues=[e for e in events if e['event']=='SUPERVISION_RESULT' and e.get('action')=='CONTINUE_NAVIGATION']
            if len(requests)>=20 and len(continues)>=10 and moved[0]:
                span=requests[-1]['time']-requests[0]['time'];rate=(len(requests)-1)/span
                assert rate>=4.,rate
                assert state['owner']=='NAV'
                report.update(success=True,requests=len(requests),applied_continues=len(continues),observed_hz=rate,moving=True)
                break
        else:raise RuntimeError('Supervision smoke test timed out')
    except Exception as exc:report['error']=str(exc);raise
    finally:
        report['state']=state
        path=Path('/ws/docs/mission-runs/supervision-check.json');path.write_text(json.dumps(report,indent=2))
        try:command(dict(command='stop'))
        except Exception:pass
        node.destroy_node();rclpy.shutdown()
        print(json.dumps({k:v for k,v in report.items() if k!='state'},indent=2),flush=True)

if __name__=='__main__':main()
