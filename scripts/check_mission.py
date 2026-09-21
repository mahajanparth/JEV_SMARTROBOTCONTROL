#!/usr/bin/env python3
"""Live seeded mission test: optional ROS service injection during actual navigation."""
import argparse
import json
from pathlib import Path
import time
from urllib.request import urlopen, Request
from urllib.error import URLError
import rclpy
from geometry_msgs.msg import Twist
from jev_demo_interfaces.srv import InjectDelocalization


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--inject',action='store_true')
    parser.add_argument('--observe',action='store_true')
    parser.add_argument('--timeout',type=float,default=600)
    args=parser.parse_args()
    rclpy.init(args=[]);node=rclpy.create_node('mission_end_to_end_check')
    movement=[0.]
    node.create_subscription(Twist,'/cmd_vel',lambda m:movement.__setitem__(0,time.monotonic()) if m.linear.x>.03 else None,10)
    client=node.create_client(InjectDelocalization,'/demo/inject_delocalization')
    def state():return json.load(urlopen('http://localhost:8765/api/state',timeout=3))
    ready_deadline=time.monotonic()+30
    while True:
        try:
            if state().get('phase'):break
        except (URLError,ConnectionError):pass
        if time.monotonic()>ready_deadline:raise RuntimeError('Dashboard did not become ready')
        time.sleep(.2)
    if not args.observe:
        previous=state()
        payload=json.dumps(dict(command='start',seed=args.seed)).encode()
        urlopen(Request('http://localhost:8765/api/command',data=payload,headers={'Content-Type':'application/json'}),timeout=3).close()
        deadline=time.monotonic()+20
        while True:
            current=state()
            if current.get('seed')==args.seed and current.get('elapsed',0)<previous.get('elapsed',0) and current.get('phase')!='SUCCEEDED':break
            if time.monotonic()>deadline:raise RuntimeError('New mission was not accepted: '+str(current.get('events',[])[-2:]))
            time.sleep(.2)
    started=time.monotonic(); injected=False; injection_time=None; future=None; last_phase=None; s={}
    report={'seed':args.seed,'injection_requested':args.inject,'success':False}
    try:
        while time.monotonic()-started<args.timeout:
            rclpy.spin_once(node,timeout_sec=.1)
            s=state()
            if s.get('phase')!=last_phase:
                last_phase=s.get('phase');print(last_phase,s.get('reason'),flush=True)
            if s.get('phase')=='NAVIGATING' and args.inject and not injected and time.monotonic()-movement[0]<.5:
                if not client.service_is_ready():raise RuntimeError('Injection service unavailable')
                req=InjectDelocalization.Request();req.dx=4.;req.dy=4.;req.dyaw=1.5;req.variance=.005
                future=client.call_async(req);injected=True;injection_time=s['elapsed'];print('Injecting while moving',flush=True)
            if future and future.done():
                result=future.result()
                if not result.success:raise RuntimeError(result.message)
                report['injection_response']=result.message;future=None
            if last_phase in ('HELP','STOPPED'):raise RuntimeError(s.get('reason'))
            if last_phase=='SUCCEEDED':
                if args.inject and not injected:raise RuntimeError('Goal reached before moving injection could run')
                events=s['events']
                if args.inject:
                    after=[e for e in events if e['time']>injection_time]
                    assert any(e['event']=='MOTION_REVOKED' and 'Localization deteriorated' in e.get('reason','') for e in after)
                    assert any(e['event']=='ACTION' and e['action']=='RESUME_NAVIGATION' for e in after)
                if not any(e['event']=='GROUND_TRUTH_EVALUATION' for e in events):continue
                truth=next(e for e in reversed(events) if e['event']=='GROUND_TRUTH_EVALUATION')
                assert truth['goal_error_m']<.4,truth
                assert truth['localization_error_m']<.4,truth
                report.update(success=True,evaluation=truth);break
        else:raise RuntimeError('Mission check timed out')
    except Exception as exc:
        report['error']=str(exc);raise
    finally:
        report['final_state']=s
        folder=Path('/ws/docs/mission-runs');folder.mkdir(exist_ok=True,parents=True)
        destination=folder/('check-{}-{}.json'.format(args.seed,time.time_ns()))
        destination.write_text(json.dumps(report,indent=2));print('Report:',destination,flush=True)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
