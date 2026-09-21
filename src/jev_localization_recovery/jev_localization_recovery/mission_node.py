"""Jev mission coordinator. Ground truth is used only by spawn and evaluation."""
from collections import deque
from concurrent.futures import Future
from copy import deepcopy
import json
import math
from pathlib import Path
from queue import Queue, Empty as QueueEmpty
from threading import Thread
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import NavigateToPose, ComputePathToPose, FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from gazebo_msgs.srv import SetEntityState, GetEntityState
from jev_demo_interfaces.srv import InjectDelocalization
from .recovery_node import RecoveryNode
from .recovery_actions import MotionHistory
from .localization_monitor import Health, LocalizationMonitor, angle_delta
from .mission_policy import ACTIONS, MissionSelector, LOCAL_TARGETS, sample_mission
from .local_move import LocalMove, path_clear
from .scan_map_score import transform_point
from .dashboard import Dashboard
from .navigation_supervision import route_deviation

class MissionNode(RecoveryNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('seed',42)
        self.declare_parameter('autostart',False)
        self.declare_parameter('mission_timeout',600.)
        self.declare_parameter('navigation_min_score',.85)
        self.declare_parameter('jev_supervision_interval',.2)
        self.declare_parameter('jev_supervision_max_inflight',4)
        self.supervision_max_inflight=int(self.get_parameter('jev_supervision_max_inflight').value)
        if not 1<=self.supervision_max_inflight<=16:raise ValueError('Supervision concurrency must be 1..16')
        self.supervision_interval=float(self.get_parameter('jev_supervision_interval').value)
        if not math.isfinite(self.supervision_interval) or self.supervision_interval<.2:
            raise ValueError('jev_supervision_interval must be finite and at least 0.2 seconds')
        self.supervision_requests={}; self.nav_epoch=0; self.last_applied_review=0
        self.next_supervision_at=0.; self.request_times=deque(maxlen=100); self.last_veto=None
        self.last_supervision_started=-math.inf; self.last_review_at=None
        self.call_count=0; self.supervision_calls=0; self.supervision_status='INACTIVE'
        self.nav_feedback={}; self.nav_feedback_at=None; self.telemetry_history=deque(maxlen=30)
        self.declare_parameter('dashboard_port',8765)
        self.declare_parameter('report_dir','/ws/docs/mission-runs')
        self.mission_selector=MissionSelector(model=self.p['jev_model'],timeout=self.p['jev_timeout'],key_file=self.p['jev_api_key_file'])
        self.phase='IDLE'; self.machine='IDLE'; self.owner='NONE'; self.reason='Ready for a random mission'
        self.seed=int(self.get_parameter('seed').value); self.goal=None; self.started=time.monotonic()
        self.ready_since=None; self.decision=None; self.events=deque(maxlen=300)
        self.pending=None; self.nav_send=None; self.nav_handle=None; self.nav_result=None; self.cancel_future=None
        self.nav_started=False; self.nav_bad_since=None; self.nav_failures=0; self.local_attempts=0
        self.pause_count=0; self.next_phase='DECIDING'; self.last_ui=0.; self.path=[]
        self.injections=[]; self.report=None; self.eval_future=None; self.spin_lock_since=None
        self.safety_status={'status':'SENSOR_FAULT','reason':'Waiting for safety bridge'}
        self.safety_at=0.; self.safety_wait_started=None
        self.declare_parameter('safety_blocked_timeout',15.)
        self.create_subscription(String,'/demo/safety',self.on_safety,10)
        self.owner_pub=self.create_publisher(String,'/demo/owner',10)
        self.demo_pub=self.create_publisher(String,'/demo/state',10)
        self.demo_events=self.create_publisher(String,'/demo/events',10)
        self.goal_pub=self.create_publisher(PoseStamped,'/demo/goal',10)
        self.initial_pub=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.create_subscription(NavPath,'/plan',lambda m:setattr(self,'path',[[p.pose.position.x,p.pose.position.y] for p in m.poses]),10)
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.follower=ActionClient(self,FollowPath,'/follow_path')
        self.replan_reset=self.create_publisher(String,'/demo/replan_ready',10)
        self.replan_attempts=0;self.replan_limit=2;self.replan_result='Not requested'
        self.blocked_review=False;self.blocked_decision=False;self.replanned_path=None
        self.clear_clients=[self.create_client(ClearEntireCostmap,name) for name in
            ('/global_costmap/clear_entirely_global_costmap','/local_costmap/clear_entirely_local_costmap')]
        self.set_entity=self.create_client(SetEntityState,'/set_entity_state')
        self.get_entity=self.create_client(GetEntityState,'/get_entity_state')
        self.commands=Queue(maxsize=20)
        self.tuning_client=self.create_client(SetParametersAtomically,'/jev_gazebo_support/set_parameters_atomically')
        self.tuning_future=None;self.tuning_result=dict(status='IDLE',message='No settings changed')
        self.create_service(Trigger,'/demo/start',self.start_service)
        self.create_service(Trigger,'/demo/stop',self.stop_service)
        self.create_service(Trigger,'/demo/acknowledge_help',self.ack_service)
        self.create_service(InjectDelocalization,'/demo/inject_delocalization',self.inject_service)
        self.dashboard=Dashboard(self.commands,int(self.get_parameter('dashboard_port').value))
        self.auto=bool(self.get_parameter('autostart').value)
        self.emit('READY',reason='Jev mission controller ready')

    def on_safety(self,msg):
        try:
            state=json.loads(msg.data)
            if state.get('status') not in ('CLEAR','SLOW','BLOCKED','SENSOR_FAULT'):return
        except (ValueError,AttributeError):return
        changed=(state.get('status'),state.get('reason'))!=(self.safety_status.get('status'),self.safety_status.get('reason'))
        self.safety_status=state;self.safety_at=time.monotonic()
        if changed:self.emit('SAFETY',**state)
        if state['status'] in ('BLOCKED','SENSOR_FAULT') and not self.blocked_decision and self.phase in ('NAVIGATING','EXECUTING','NAV_STARTING','PREPARING_NAV','WAITING_JEV'):
            self.safety_wait_started=time.monotonic()
            self.blocked_review=False
            self.halt('SAFETY_WAIT','Obstacle safety: '+state.get('reason','Motion blocked'))
            self.owner_pub.publish(String(data='NONE'))

    def safety_snapshot(self,now):
        if now-self.safety_at>.5:
            return dict(status='SENSOR_FAULT',reason='Safety bridge telemetry stale',latched=True)
        return self.safety_status

    def emit(self,event,**fields):
        data=dict(event=event,time=round(time.monotonic()-self.started,3),**fields)
        if event=='VETO':self.last_veto=data
        self.events.append(data)
        self.demo_events.publish(String(data=json.dumps(data,allow_nan=False)))
        if self.report:
            with self.report.open('a') as f: f.write(json.dumps(data,allow_nan=False)+'\n')
        self.get_logger().info('{} {}'.format(event,fields.get('reason',fields.get('action',''))))

    def set_phase(self,phase,reason=''):
        self.phase=self.machine=phase
        if reason: self.reason=reason
        self.emit('PHASE',phase=phase,reason=reason)

    def start_service(self,req,res):
        res.success,res.message=self.start_mission(self.seed)
        return res

    def stop_service(self,req,res):
        self.halt('STOPPED','Operator stopped the mission')
        res.success=True; res.message='Velocity ownership revoked; navigation cancellation requested'
        return res

    def ack_service(self,req,res):
        if self.phase!='HELP': res.success=False; res.message='No help request is pending'
        elif self.tuning_future is not None:res.success=False;res.message='Wait for safety settings to finish applying'
        elif (self.pending and not self.pending.done()) or self.reviews_inflight(): res.success=False; res.message='Waiting for previous Jev request to finish'
        else:
            # Explicit new bounded recovery budget after an operator intervention.
            self.attempts={k:0 for k in self.attempts}; self.local_attempts=0; self.nav_failures=0
            self.pause_count=0; self.ready_since=None; self.started=time.monotonic()
            self.replan_attempts=0;self.blocked_decision=False;self.blocked_review=False
            self.stage_started=time.monotonic()
            self.set_phase('ACK_RECHECK','Checking fresh localization for two seconds before asking Jev again')
            res.success=True; res.message=self.reason
        return res

    def tune_safety(self,values):
        from .obstacle_safety import SafetyConfig, SAFETY_LIMITS
        if self.phase not in ('IDLE','HELP','STOPPED','SUCCEEDED') or self.owner!='NONE' or self.nav_send or self.nav_handle:
            raise ValueError('Stop the mission before changing safety settings')
        if self.tuning_future is not None:raise ValueError('A settings update is already pending')
        if not isinstance(values,dict) or set(values)!=set(SAFETY_LIMITS):raise ValueError('Supply all eight safety parameters')
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) for v in values.values()):raise ValueError('Numeric values required')
        SafetyConfig(**values)
        if not self.tuning_client.service_is_ready():raise ValueError('Safety bridge parameter service unavailable')
        req=SetParametersAtomically.Request()
        req.parameters=[Parameter('safety_'+k,value=float(v)).to_parameter_msg() for k,v in values.items()]
        self.tuning_future=self.tuning_client.call_async(req)
        self.tuning_result=dict(status='PENDING',message='Waiting for safety bridge')

    def inject_service(self,req,res):
        res.stamp=self.get_clock().now().to_msg()
        values=(req.dx,req.dy,req.dyaw,req.variance)
        if (not all(math.isfinite(v) for v in values) or max(abs(req.dx),abs(req.dy))>12 or
                abs(req.dyaw)>math.pi*2 or not 0<req.variance<=10):
            res.success=False; res.message='Invalid offsets or covariance'; return res
        if self.phase not in ('NAVIGATING','DECIDING','EXECUTING','EVALUATING') or self.pose_message is None:
            res.success=False; res.message='An active localized mission is required'; return res
        msg=deepcopy(self.pose_message)
        q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))+req.dyaw
        msg.pose.pose.position.x+=req.dx; msg.pose.pose.position.y+=req.dy
        q.x=q.y=0.; q.z=math.sin(yaw/2); q.w=math.cos(yaw/2)
        msg.pose.covariance=[0.]*36
        for i in (0,7,35): msg.pose.covariance[i]=req.variance
        # Publishing only; the controller does not use injection knowledge to trigger recovery.
        now=time.monotonic()
        self.injections.extend((now+i*.2,deepcopy(msg)) for i in range(3))
        self.emit('INJECTION',dx=req.dx,dy=req.dy,dyaw=req.dyaw,variance=req.variance)
        res.success=True; res.message='AMCL pose perturbation queued; physical robot unchanged'
        return res

    def start_mission(self,seed):
        if self.tuning_future is not None:return False,'Wait for safety settings to finish applying'
        if self.phase not in ('IDLE','STOPPED','SUCCEEDED','HELP') or self.nav_send or self.nav_handle or (self.pending and not self.pending.done()) or self.reviews_inflight():
            return False,'Stop the current mission and wait for cancellation before starting'
        if self.grid is None or not self.set_entity.service_is_ready() or not self.global_client or not self.global_client.service_is_ready():
            return False,'Waiting for map, Gazebo state service, and AMCL'
        if isinstance(seed,bool) or not isinstance(seed,int) or not 0<=seed<=2147483647:
            return False,'Seed must be an integer between 0 and 2147483647'
        self.owner='NONE'; self.publish_velocity(0.,0.)
        start,self.goal=sample_mission(self.grid,seed)
        self.seed=seed; self.started=time.monotonic(); self.events.clear(); self.pending=None
        self.nav_epoch+=1; self.supervision_requests.clear(); self.telemetry_history.clear()
        self.last_applied_review=0; self.request_times.clear(); self.last_veto=None
        self.call_count=0; self.supervision_calls=0; self.last_review_at=None
        self.nav_feedback={}; self.nav_feedback_at=None; self.supervision_status='INACTIVE'
        folder=Path(self.get_parameter('report_dir').value); folder.mkdir(parents=True,exist_ok=True)
        self.report=folder/('mission-{}-{}.jsonl'.format(seed,time.time_ns()))
        self.attempts={k:0 for k in self.attempts}; self.local_attempts=0; self.pause_count=0; self.nav_failures=0
        self.nav_started=False; self.ready_since=None; self.decision=None; self.injections.clear(); self.path=[]
        self.replan_attempts=0;self.replan_result='Not requested';self.blocked_review=False;self.blocked_decision=False
        self.motion_history=MotionHistory(); self.worsened_while_moving=False
        req=SetEntityState.Request(); req.state.name='turtlebot3_burger'; req.state.reference_frame='world'
        req.state.pose.position.x,req.state.pose.position.y=start[:2]; req.state.pose.position.z=.01
        req.state.pose.orientation.z=math.sin(start[2]/2); req.state.pose.orientation.w=math.cos(start[2]/2)
        self.spawn_future=self.set_entity.call_async(req)
        self.stage_started=time.monotonic()
        self.set_phase('SETTING_START','Random connected start and goal selected')
        # Separate evaluation artifact: never included in Jev state or initialpose.
        self.report.with_suffix('.spawn.json').write_text(json.dumps(dict(seed=seed,start=start,goal=self.goal)))
        self.emit('MISSION',seed=seed,goal=self.goal)
        return True,'Mission starting'

    def halt(self,next_phase,reason):
        self.nav_epoch+=1; self.supervision_status='INACTIVE'
        self.blocked_decision=False
        self.owner='NONE'; self.publish_velocity(0.,0.)
        self.owner_pub.publish(String(data='NONE'))
        self.actions.stop(reason)
        self.active_action='NONE'; self.reason=reason
        self.next_phase=next_phase; self.cancel_started=time.monotonic(); self.cancel_future=None
        if self.nav_send or self.nav_handle:
            self.set_phase('CANCELING',reason)
        else:
            self.set_phase(next_phase,reason)
        self.emit('MOTION_REVOKED',reason=reason)

    def nav_poll(self):
        if self.nav_send and self.nav_send.done():
            future=self.nav_send; self.nav_send=None
            try:
                handle=future.result()
                if not handle.accepted: raise RuntimeError('Navigation goal rejected')
                self.nav_handle=handle; self.nav_result=handle.get_result_async()
            except Exception:
                if self.phase!='CANCELING': self.halt('HELP','Nav2 rejected or failed to accept the goal')
        if self.phase=='CANCELING':
            if self.nav_handle and self.cancel_future is None:
                self.cancel_future=self.nav_handle.cancel_goal_async()
            if self.nav_result and self.nav_result.done():
                self.nav_handle=self.nav_result=None
            if not self.nav_send and not self.nav_handle:
                if time.monotonic()-self.cancel_started>.5:
                    self.ready_since=None; self.set_phase(self.next_phase,self.reason)
            elif time.monotonic()-self.cancel_started>5:
                self.reason='Waiting for Nav2 cancellation acknowledgment; motion stays disabled'

    def points(self,now):
        laser=self.scan_geometry(now)
        return [transform_point(r*math.cos(self.scan.angle_min+i*self.scan.angle_increment),
                                r*math.sin(self.scan.angle_min+i*self.scan.angle_increment),laser)
                for i,r in enumerate(self.scan.ranges) if math.isfinite(r) and self.scan.range_min<=r<self.scan.range_max]

    def can_replan(self,now):
        safety=self.safety_snapshot(now)
        return (self.nav_started and self.goal is not None and self.localization_ready and
                safety['status'] in ('CLEAR','SLOW','BLOCKED') and
                self.replan_attempts<self.replan_limit and self.planner.server_is_ready() and
                self.follower.server_is_ready())

    def navigation_choices(self,now):
        result=['CONTINUE_NAVIGATION','PAUSE_NAVIGATION','REQUEST_HELP','STOP']
        if self.can_replan(now):result.append('REPLAN_PATH')
        return result

    def begin_replan(self,now):
        self.replan_attempts+=1;self.replan_result='Planning with current obstacle costmap'
        self.blocked_decision=False
        self.halt('REPLAN_START','Jev requested a new route to the same destination')
        self.active_action='REPLAN_PATH'
        self.stage_started=now
        self.emit('REPLAN_REQUEST',attempt=self.replan_attempts,goal=self.goal)

    def step_replan(self,state,now):
        if not self.localization_ready or self.safety_snapshot(now)['status']=='SENSOR_FAULT':
            self.replan_result='Rejected: localization or sensors unavailable'
            self.halt('HELP',self.replan_result);return
        if self.phase=='REPLAN_START':
            goal=ComputePathToPose.Goal();goal.goal=self.goal_message();goal.planner_id='GridBased';goal.use_start=False
            self.nav_send=self.planner.send_goal_async(goal);self.stage_started=now
            self.set_phase('REPLANNING','Computing a route while motion is disabled')
        elif self.phase=='REPLANNING':
            if self.nav_result and self.nav_result.done():
                try:response=self.nav_result.result();path=response.result.path
                except Exception:
                    self.halt('HELP','Planner result unavailable');return
                self.nav_result=self.nav_handle=None
                points=[(p.pose.position.x,p.pose.position.y) for p in path.poses]
                valid=(response.status==4 and path.header.frame_id=='map' and len(points)>=2 and
                       all(math.isfinite(v) for xy in points for v in xy) and
                       all(p.header.frame_id in ('','map') and all(math.isfinite(v) for v in
                           (p.pose.position.z,p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w)) and
                           .9<sum(v*v for v in (p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w))<1.1
                           for p in path.poses) and
                       math.dist(points[-1],self.goal[:2])<=.3 and self.monitor.pose is not None and
                       math.dist(points[0],self.monitor.pose[:2])<=.5 and
                       all(math.dist(a,b)<=.5 for a,b in zip(points,points[1:])))
                if not valid:
                    self.replan_result='No valid replacement route';self.emit('REPLAN_RESULT',success=False,reason=self.replan_result)
                    self.halt('HELP',self.replan_result);return
                self.replanned_path=path;self.path=[list(xy) for xy in points]
                self.replan_result='Route found; waiting for independent safety clearance'
                self.emit('REPLAN_RESULT',success=True,attempt=self.replan_attempts,points=len(points))
                self.replan_reset.publish(String(data='CHECK_NEW_ROUTE'))
                self.stage_started=now;self.set_phase('REPLAN_READY',self.replan_result)
            elif now-self.stage_started>8:self.halt('HELP','Replanning timed out')
        elif self.phase=='REPLAN_READY':
            safety=self.safety_snapshot(now)
            if now-self.stage_started>1.2 and safety['status'] in ('CLEAR','SLOW') and not safety.get('latched'):
                goal=FollowPath.Goal();goal.path=self.replanned_path;goal.controller_id='FollowPath';goal.goal_checker_id='general_goal_checker'
                self.nav_epoch+=1;self.nav_feedback={};self.nav_feedback_at=None;self.telemetry_history.clear()
                self.last_review_at=None;self.next_supervision_at=now
                epoch=self.nav_epoch
                self.nav_send=self.follower.send_goal_async(goal,feedback_callback=lambda msg:self.on_path_feedback(msg,epoch))
                self.stage_started=now;self.replan_result='Following replacement route'
                self.set_phase('NAV_STARTING',self.replan_result)
            elif now-self.stage_started>5:self.halt('HELP','New route exists but safe departure is unavailable')

    def available(self,state,now):
        allowed=['REQUEST_HELP','STOP']; targets={}
        if self.blocked_decision or self.safety_snapshot(now)['status'] not in ('CLEAR','SLOW'):
            if self.can_replan(now):allowed+=['REPLAN_PATH','PAUSE_NAVIGATION']
            return allowed,targets
        if self.can_replan(now):allowed.append('REPLAN_PATH')
        if self.localization_ready:
            if self.nav.server_is_ready(): allowed.insert(0,'RESUME_NAVIGATION' if self.nav_started else 'NAVIGATE_TO_GOAL')
            if self.pause_count<2: allowed.append('PAUSE_NAVIGATION')
            return allowed,targets
        if sum(self.attempts.values())+self.local_attempts>=5: return allowed,targets
        if not self.actions.safety_reason(state,self.motion_sample,now):
            if self.attempts['spin_attempts']<1: allowed.append('SPIN')
            if (state.global_service_available and self.attempts['global_relocalization_attempts']<2 and
                    (self.nav_started or self.attempts['spin_attempts']>0)):
                allowed.append('GLOBAL_RELOCALIZE')
            if state.backtrack_available and state.worsened_while_moving and not self.attempts['backtrack_attempts']:
                allowed.append('BACKTRACK_AND_SPIN')
            if self.local_attempts<2 and self.attempts['spin_attempts']>0:
                try:
                    points=self.points(now)
                    targets={k:v for k,v in LOCAL_TARGETS.items() if path_clear(*v,points)}
                except Exception: targets={}
                if targets: allowed.append('MOVE_LOCAL')
        return allowed,targets

    def request_decision(self,state,now):
        # Let previous navigation requests drain; their results cannot drive recovery.
        if self.reviews_inflight():return
        for call_id in self.supervision_requests:
            self.emit('DISCARDED',purpose='NAVIGATION_SUPERVISION',call_id=call_id,reason='Navigation ended')
        self.supervision_requests.clear()
        allowed,targets=self.available(state,now)
        context=dict(robot=state.to_dict(),safety=self.safety_snapshot(now),mission=dict(goal=self.goal,navigation_started=self.nav_started,
            navigation_failures=self.nav_failures,local_move_attempts=self.local_attempts),
            localization_ready=self.localization_ready,replanning=dict(attempts=self.replan_attempts,limit=self.replan_limit,result=self.replan_result),last_outcome=self.reason,
            allowed_actions=allowed,local_targets=targets)
        payload=self.mission_selector.make_payload(context,allowed,targets)
        self.call_count+=1
        self.emit('JEV_REQUEST',purpose='MISSION_DECISION',call_id=self.call_count,request=payload)
        self.request_state=(state.status,self.localization_ready)
        self.request_started=now; future=Future(); self.pending=future
        def run():
            try: future.set_result(self.mission_selector.infer(payload))
            except Exception as exc: future.set_exception(exc)
        Thread(target=run,daemon=True,name='jev-mission').start()
        self.set_phase('WAITING_JEV','Robot stopped while Jev selects an action')

    def execute(self,decision,state,now):
        action=decision['action']; self.decision=decision
        allowed,targets=self.available(state,now)
        if action not in allowed:
            reason='{} is not allowed now; available actions: {}'.format(action, ', '.join(allowed))
            self.emit('VETO',action=action,reason=reason,allowed_actions=allowed)
            self.halt('HELP',reason);return
        if action not in ('STOP','REQUEST_HELP','PAUSE_NAVIGATION') and decision['confidence']<self.p['jev_min_confidence']:
            reason='Jev chose {} with confidence {:.2f}, below the {:.2f} threshold'.format(
                action,decision['confidence'],self.p['jev_min_confidence'])
            self.emit('VETO',action=action,reason=reason,confidence=decision['confidence'],threshold=self.p['jev_min_confidence'])
            self.halt('HELP',reason);return
        if action=='REPLAN_PATH':self.begin_replan(now);return
        if self.blocked_decision:
            self.blocked_decision=False
            if action=='PAUSE_NAVIGATION':self.set_phase('SAFETY_WAIT','Jev chose to wait for obstacle clearance');return
        if action=='MOVE_LOCAL':
            if decision.get('target_confidence',0)<self.p['jev_min_confidence']:
                reason='Jev local-target confidence {:.2f} is below {:.2f}'.format(
                    decision.get('target_confidence',0),self.p['jev_min_confidence'])
                self.emit('VETO',action=action,reason=reason);self.halt('HELP',reason);return
            if (decision['x_m'],decision['y_m']) not in targets.values():
                reason='Local target is no longer in the currently safe target set'
                self.emit('VETO',action=action,reason=reason);self.halt('HELP',reason);return
        self.emit('ACTION',action=action,decision=decision)
        self.active_action=action
        if action in ('STOP','REQUEST_HELP'):
            self.halt('STOPPED' if action=='STOP' else 'HELP',
                'Jev requested operator assistance; inspect obstacles/localization, optionally set a pose in RViz, then acknowledge'
                if action=='REQUEST_HELP' else 'Jev selected STOP'); return
        if action=='PAUSE_NAVIGATION':
            self.pause_count+=1; self.stage_started=now; self.set_phase('PAUSED','Jev requested a brief pause'); return
        if action in ('NAVIGATE_TO_GOAL','RESUME_NAVIGATION'):
            self.attempts={k:0 for k in self.attempts}; self.local_attempts=0
            if not all(c.service_is_ready() for c in self.clear_clients):
                self.halt('HELP','Navigation costmap services unavailable');return
            self.clear_futures=[c.call_async(ClearEntireCostmap.Request()) for c in self.clear_clients]
            self.stage_started=now; self.owner='NONE'
            self.set_phase('PREPARING_NAV','Clearing stale obstacle observations before planning');return
        self.owner='RECOVERY'; self.stage_started=now; self.spin_lock_since=None; self.ready_since=None
        if action=='MOVE_LOCAL':
            self.local_attempts+=1
            self.local=LocalMove(self.motion_sample,decision['x_m'],decision['y_m'],now)
        else:
            key={'SPIN':'spin_attempts','GLOBAL_RELOCALIZE':'global_relocalization_attempts','BACKTRACK_AND_SPIN':'backtrack_attempts'}[action]
            self.attempts[key]+=1
            self.actions.start(action,state,self.motion_sample,now)
        self.set_phase('EXECUTING','Executing '+action)

    def on_path_feedback(self,msg,epoch):
        if epoch!=self.nav_epoch:return
        feedback=msg.feedback
        values=dict(distance_remaining_m=float(feedback.distance_to_goal),speed_mps=float(feedback.speed))
        self.nav_feedback={k:v for k,v in values.items() if math.isfinite(v)}
        self.nav_feedback_at=time.monotonic()

    def on_nav_feedback(self,msg,epoch):
        if epoch!=self.nav_epoch or self.phase not in ('NAV_STARTING','NAVIGATING'):return
        feedback=msg.feedback
        def seconds(duration):return duration.sec+duration.nanosec*1e-9
        values=dict(distance_remaining_m=float(feedback.distance_remaining),
            navigation_time_seconds=seconds(feedback.navigation_time),
            estimated_time_remaining_seconds=seconds(feedback.estimated_time_remaining))
        self.nav_feedback={k:v for k,v in values.items() if math.isfinite(v)}
        self.nav_feedback_at=time.monotonic()

    def navigation_telemetry(self,state,now):
        pose=self.monitor.pose
        velocity=None
        if self.odom is not None and now-self.odom_at<.75:
            v=self.odom.twist.twist
            if all(math.isfinite(n) for n in (v.linear.x,v.angular.z)):
                velocity=dict(linear_mps=v.linear.x,angular_radps=v.angular.z)
        sample=dict(time_seconds=round(now-self.started,3),scan_map_score=state.scan_map_score,
            pose_uncertainty=state.pose_uncertainty,obstacle_clearance_m=state.min_obstacle_distance,
            goal_distance_m=math.dist(pose[:2],self.goal[:2]) if pose and self.goal else None,
            route_deviation_m=route_deviation(pose,self.path))
        if not self.telemetry_history or sample['time_seconds']-self.telemetry_history[-1]['time_seconds']>=.5:
            self.telemetry_history.append(sample)
        return dict(estimated_pose=pose,velocity=velocity,safety=self.safety_snapshot(now),**sample,
            nav2_feedback=self.nav_feedback,feedback_age_seconds=now-self.nav_feedback_at if self.nav_feedback_at else None,
            recent_samples=list(self.telemetry_history),owner=self.owner,active_action=self.active_action,
            elapsed_on_current_goal_seconds=now-self.stage_started)

    def reviews_inflight(self):
        return sum(not item['future'].done() for item in self.supervision_requests.values())

    def apply_navigation_review(self,decision,state,now):
        action=decision['action'];self.decision=decision;self.last_review_at=now
        allowed=self.navigation_choices(now)
        if action not in allowed:
            reason='{} is not allowed during navigation supervision'.format(action)
            self.emit('VETO',action=action,reason=reason,allowed_actions=allowed)
            self.halt('HELP',reason);return
        if action in ('CONTINUE_NAVIGATION','REPLAN_PATH') and decision['confidence']<self.p['jev_min_confidence']:
            reason='Jev continuation confidence {:.2f} is below {:.2f}'.format(decision['confidence'],self.p['jev_min_confidence'])
            self.emit('VETO',action=action,reason=reason)
            self.halt('HELP',reason);return
        if action=='REPLAN_PATH':self.begin_replan(now);return
        if action=='CONTINUE_NAVIGATION':
            if not state.data_ready or state.status!=Health.HEALTHY:
                self.halt('DECIDING','Localization changed before Jev continuation could be applied');return
            self.supervision_status='MONITORING'
            self.emit('SUPERVISION_RESULT',action=action,owner=self.owner)
        elif action=='PAUSE_NAVIGATION':
            self.halt('DECIDING','Jev paused navigation for reassessment')
        else:
            self.halt('HELP' if action=='REQUEST_HELP' else 'STOPPED',
                'Jev navigation supervisor requested '+action)

    def supervise_navigation(self,state,now):
        if self.phase!='NAVIGATING':return
        telemetry=self.navigation_telemetry(state,now)
        # Newest completed snapshot wins. Older responses cannot reverse it.
        for call_id,item in sorted(list(self.supervision_requests.items()),reverse=True):
            future=item['future']
            stale=item['epoch']!=self.nav_epoch or call_id<=self.last_applied_review
            expired=now-item['started']>=self.p['decision_timeout']
            if not future.done():
                if expired and not stale:
                    self.emit('JEV_ERROR',purpose='NAVIGATION_SUPERVISION',call_id=call_id,reason='Review deadline exceeded')
                    self.halt('HELP','Jev navigation supervision timed out');return
                continue
            del self.supervision_requests[call_id]
            if stale or expired:
                self.emit('DISCARDED',purpose='NAVIGATION_SUPERVISION',call_id=call_id,reason='Superseded, expired, or cancelled review')
                continue
            try:decision,body=future.result()
            except Exception:
                self.emit('JEV_ERROR',purpose='NAVIGATION_SUPERVISION',call_id=call_id,reason='Review request failed')
                self.halt('HELP','Jev navigation supervision failed');return
            self.emit('JEV_RESPONSE',purpose='NAVIGATION_SUPERVISION',call_id=call_id,
                response=body,decision=decision,latency_ms=round((now-item['started'])*1000))
            self.last_applied_review=call_id
            self.apply_navigation_review(decision,state,now)
            if self.phase!='NAVIGATING':return
        if now-(self.last_review_at or self.stage_started)>self.p['decision_timeout']:
            self.halt('HELP','No fresh Jev navigation review within the decision deadline');return
        if now+1e-6<self.next_supervision_at:return
        if self.reviews_inflight()>=self.supervision_max_inflight:
            self.supervision_status='BACKPRESSURE';return
        if self.pending is not None and not self.pending.done():return
        context=dict(mode='NAVIGATION_SUPERVISION',robot=state.to_dict(),navigation=telemetry,
            mission=dict(goal=self.goal,navigation_started=True,navigation_failures=self.nav_failures,
                         replan_attempts=self.replan_attempts,replan_limit=self.replan_limit,replan_result=self.replan_result),
            localization_ready=self.localization_ready)
        allowed=self.navigation_choices(now)
        payload=self.mission_selector.make_payload(context,allowed,{})
        self.call_count+=1;self.supervision_calls+=1;call_id=self.call_count
        self.last_supervision_started=now;self.supervision_status='REVIEW_PENDING'
        self.next_supervision_at=max(self.next_supervision_at+self.supervision_interval,now)
        if self.next_supervision_at<=now:self.next_supervision_at=now+self.supervision_interval
        self.request_times.append(now)
        self.emit('JEV_REQUEST',purpose='NAVIGATION_SUPERVISION',call_id=call_id,request=payload)
        future=Future()
        self.supervision_requests[call_id]=dict(future=future,started=now,epoch=self.nav_epoch)
        def infer():
            try:future.set_result(self.mission_selector.infer(payload))
            except Exception as exc:future.set_exception(exc)
        Thread(target=infer,daemon=True,name='jev-navigation-supervisor').start()

    def goal_message(self):
        msg=PoseStamped(); msg.header.frame_id='map'; msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=self.goal[:2]
        msg.pose.orientation.z=math.sin(self.goal[2]/2); msg.pose.orientation.w=math.cos(self.goal[2]/2)
        return msg

    def step_recovery(self,state,now):
        self.nav_poll()
        if self.tuning_future is not None and self.tuning_future.done():
            try:
                result=self.tuning_future.result().result
                self.tuning_result=dict(status='APPLIED' if result.successful else 'REJECTED',message=result.reason)
            except Exception:self.tuning_result=dict(status='REJECTED',message='Safety bridge update failed')
            self.tuning_future=None;self.emit('SAFETY_SETTINGS',**self.tuning_result)
        if self.phase!='NAVIGATING' and self.phase!='EXECUTING': self.owner='NONE'
        self.owner_pub.publish(String(data=self.owner))
        for at,msg in self.injections[:]:
            if now>=at:
                msg.header.stamp=self.get_clock().now().to_msg(); self.initial_pub.publish(msg)
                self.injections.remove((at,msg))
        if self.eval_future and self.eval_future.done():
            future=self.eval_future; self.eval_future=None
            try:
                result=future.result()
                if result.success:
                    p=result.state.pose.position
                    q=result.state.pose.orientation
                    true_yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
                    estimate=self.monitor.pose
                    self.emit('GROUND_TRUTH_EVALUATION',goal_error_m=math.hypot(p.x-self.goal[0],p.y-self.goal[1]),
                        localization_error_m=math.hypot(p.x-estimate[0],p.y-estimate[1]) if estimate else None,
                        yaw_error_rad=abs(angle_delta(true_yaw,estimate[2])) if estimate else None)
            except Exception: self.emit('EVALUATION_ERROR',reason='Ground truth unavailable')
        try:
            cmd=self.commands.get_nowait()
            if cmd['command']=='start':
                ok,reason=self.start_mission(cmd.get('seed',self.seed)); self.emit('COMMAND_RESULT',success=ok,reason=reason)
            elif cmd['command']=='stop': self.stop_service(None,Trigger.Response())
            elif cmd['command']=='tune_safety':
                try:self.tune_safety(cmd.get('parameters'))
                except ValueError as exc:self.tuning_result=dict(status='REJECTED',message=str(exc))
            elif cmd['command']=='ack':
                r=self.ack_service(None,Trigger.Response()); self.emit('COMMAND_RESULT',success=r.success,reason=r.message)
            else:
                req=InjectDelocalization.Request()
                for k,default in (('dx',4.),('dy',4.),('dyaw',1.5),('variance',.005)): setattr(req,k,float(cmd.get(k,default)))
                r=self.inject_service(req,InjectDelocalization.Response()); self.emit('COMMAND_RESULT',success=r.success,reason=r.message)
        except QueueEmpty: pass
        except (ValueError,TypeError,OverflowError): self.emit('COMMAND_RESULT',success=False,reason='Invalid command values')
        if self.auto and self.phase=='IDLE' and self.grid and self.set_entity.service_is_ready() and self.global_client:
            ok,_=self.start_mission(self.seed)
            if ok:self.auto=False
        healthy=(state.data_ready and state.status==Health.HEALTHY and state.pose_uncertainty is not None and state.pose_uncertainty<.35 and state.scan_map_score is not None and
                 state.scan_map_score>=float(self.get_parameter('navigation_min_score').value))
        if healthy:
            if self.ready_since is None:self.ready_since=now
        else:self.ready_since=None
        self.localization_ready=self.ready_since is not None and now-self.ready_since>=2.
        safety=self.safety_snapshot(now)
        if self.phase in ('NAVIGATING','EXECUTING','NAV_STARTING','PREPARING_NAV','WAITING_JEV','DECIDING') and safety['status'] not in ('CLEAR','SLOW') and not self.blocked_decision:
            self.safety_wait_started=now
            self.halt('SAFETY_WAIT','Obstacle safety: '+safety.get('reason','Motion blocked'))
            self.owner_pub.publish(String(data='NONE'))
            return
        if self.phase in ('REPLAN_START','REPLANNING','REPLAN_READY'):
            self.step_replan(state,now);return
        if self.phase=='SAFETY_WAIT':
            if self.safety_wait_started is None:self.safety_wait_started=now
            if not self.blocked_review and self.can_replan(now) and now-self.safety_wait_started<float(safety.get('parameters',{}).get('blocked_timeout',self.get_parameter('safety_blocked_timeout').value)):
                self.blocked_decision=True;self.request_decision(state,now)
                if self.phase=='WAITING_JEV':self.blocked_review=True
                return
            if safety['status'] in ('CLEAR','SLOW') and not safety.get('latched'):
                self.blocked_decision=False
                self.set_phase('DECIDING','Obstacle cleared; asking Jev to reassess')
            elif now-self.safety_wait_started>float(safety.get('parameters',{}).get('blocked_timeout',self.get_parameter('safety_blocked_timeout').value)):
                self.halt('HELP','Obstacle safety requires operator assistance: '+safety.get('reason','Blocked'))
            return
        if self.phase in ('IDLE','HELP','STOPPED','SUCCEEDED','CANCELING'):return
        if now-self.started>float(self.get_parameter('mission_timeout').value):
            self.halt('HELP','Mission time budget exceeded'); return
        if self.phase=='SETTING_START':
            if self.spawn_future.done():
                if not self.spawn_future.result().success:self.halt('HELP','Gazebo rejected random spawn');return
                self.monitor=LocalizationMonitor(self.thresholds); self.pose_message=None; self.motion_history=MotionHistory()
                self.init_future=self.request_global(); self.stage_started=now
                self.set_phase('INITIALIZING','AMCL initialized globally without the true spawn pose')
            elif now-self.stage_started>5:self.halt('HELP','Gazebo spawn service timed out')
        elif self.phase=='INITIALIZING':
            if self.init_future and self.init_future.done() and now-self.stage_started>3 and state.data_ready:
                try:self.init_future.result()
                except Exception:self.halt('HELP','AMCL global initialization failed');return
                self.set_phase('DECIDING','Initial localization observations ready')
            elif now-self.stage_started>20:self.halt('HELP','Initial sensor/localization evidence unavailable')
        elif self.phase=='ACK_RECHECK':
            if now-self.stage_started>=2.1 and (self.localization_ready or not healthy):
                self.set_phase('DECIDING','Fresh evidence checked after operator intervention')
        elif self.phase=='DECIDING':
            self.request_decision(state,now)
        elif self.phase=='WAITING_JEV':
            if now-self.request_started>self.p['decision_timeout']:
                self.halt('HELP','Jev response deadline exceeded');return
            if self.pending.done():
                try:decision,body=self.pending.result()
                except Exception as exc:
                    self.emit('JEV_ERROR',reason=str(exc));self.halt('HELP','Jev request failed');return
                self.pending=None
                self.emit('JEV_RESPONSE',response=body,latency_ms=round((now-self.request_started)*1000),decision=decision)
                if self.request_state!=(state.status,self.localization_ready):
                    self.emit('DISCARDED',reason='Localization changed while Jev was deciding')
                    self.blocked_decision=False
                    self.set_phase('SAFETY_WAIT' if safety['status']=='BLOCKED' else 'DECIDING','Reassessing changed evidence');return
                self.execute(decision,state,now)
        elif self.phase=='PREPARING_NAV':
            if all(f.done() for f in self.clear_futures):
                try:
                    for future in self.clear_futures:future.result()
                except Exception:self.halt('HELP','Costmap refresh failed');return
                if not self.localization_ready:self.set_phase('DECIDING','Localization changed before navigation');return
                self.emit('COSTMAPS_CLEARED')
                goal=NavigateToPose.Goal();goal.pose=self.goal_message()
                self.nav_epoch+=1; epoch=self.nav_epoch
                self.nav_feedback={};self.nav_feedback_at=None;self.telemetry_history.clear()
                self.last_supervision_started=-math.inf;self.next_supervision_at=now;self.last_review_at=None
                self.nav_send=self.nav.send_goal_async(goal,feedback_callback=lambda msg:self.on_nav_feedback(msg,epoch))
                self.nav_started=True;self.stage_started=now
                self.set_phase('NAV_STARTING','Waiting for Nav2 goal acceptance')
            elif now-self.stage_started>5:self.halt('HELP','Costmap refresh timed out')
        elif self.phase=='NAV_STARTING':
            if self.nav_handle:
                self.owner='NAV'; self.nav_bad_since=None; self.set_phase('NAVIGATING','Nav2 navigating to the assigned goal')
            elif now-self.stage_started>5:self.halt('HELP','Nav2 goal acceptance timed out')
        elif self.phase=='NAVIGATING':
            if not state.data_ready or self.motion_sample is None or now-self.motion_sample.received_at>.75:
                self.halt('HELP','Navigation lost fresh localization or odometry');return
            if state.status!=Health.HEALTHY:
                if self.nav_bad_since is None:self.nav_bad_since=now
                if state.status==Health.LOST or now-self.nav_bad_since>.5:
                    self.halt('DECIDING','Localization deteriorated during navigation');return
            else:self.nav_bad_since=None
            if self.nav_result and self.nav_result.done():
                status=self.nav_result.result().status
                self.nav_handle=self.nav_result=None;self.owner='NONE'
                if status==4 and healthy:
                    self.active_action='NONE';self.set_phase('SUCCEEDED','Goal reached with healthy localization')
                    self.emit('MISSION_SUCCESS',seed=self.seed,goal=self.goal,health=state.to_dict())
                    if self.get_entity.service_is_ready():
                        req=GetEntityState.Request();req.name='turtlebot3_burger';req.reference_frame='world'
                        self.eval_future=self.get_entity.call_async(req)
                else:
                    self.nav_failures+=1
                    self.halt('HELP' if self.nav_failures>=3 else 'DECIDING','Navigation failed or localization not healthy at arrival')
                return
            self.supervise_navigation(state,now)
        elif self.phase=='EXECUTING':
            problem=self.command_problem()
            if problem:self.halt('HELP',problem);return
            if self.active_action=='MOVE_LOCAL':
                try:points=self.points(now)
                except Exception:self.halt('HELP','Local move lost scan geometry');return
                linear,angular=self.local.step(self.motion_sample,points,now,state.min_obstacle_distance)
                self.publish_velocity(linear,angular); result,reason=self.local.result,self.local.reason
            else:
                # A single HEALTHY observation is not enough. End only the spin
                # stage after the stricter navigation lock stays reliable for 2 s.
                safety=self.actions.safety_reason(state,self.motion_sample,now)
                if self.actions.stage=='SPIN' and healthy and not safety:
                    if self.spin_lock_since is None:self.spin_lock_since=now
                    if now-self.spin_lock_since>=2.:
                        self.actions.stop('Stable localization lock for 2 seconds; spin finished early',success=True)
                else:self.spin_lock_since=None
                if self.actions.result is None:self.actions.tick(state,self.motion_sample,now)
                result,reason=self.actions.result,self.actions.reason
            if result is not None:
                self.owner='NONE';self.owner_pub.publish(String(data='NONE'));self.publish_velocity(0.,0.);self.stage_started=now;self.ready_since=None
                self.emit('ACTION_RESULT',action=self.active_action,success=result,reason=reason)
                self.set_phase('EVALUATING',reason)
        elif self.phase=='EVALUATING':
            if now-self.stage_started>4:
                self.active_action='NONE';self.set_phase('DECIDING','Localization ready' if self.localization_ready else 'Previous action did not restore localization')
        elif self.phase=='PAUSED' and now-self.stage_started>2:self.set_phase('DECIDING','Pause complete')

    def publish_state(self,state):
        super().publish_state(state)
        now=time.monotonic()
        if now-self.last_ui<.3:return
        self.last_ui=now
        data=dict(phase=self.phase,seed=self.seed,goal=self.goal,pose=self.monitor.pose,
            elapsed=now-self.started,health=state.to_dict(),safety=self.safety_snapshot(now),owner=self.owner,reason=self.reason,
            decision=self.decision,active_action=self.active_action,path=self.path,
            tuning=self.tuning_result,
            replanning=dict(attempts=self.replan_attempts,limit=self.replan_limit,result=self.replan_result),
            jev_calls=self.call_count,supervision=dict(enabled=True,interval_seconds=self.supervision_interval,
                status=self.supervision_status if self.phase=='NAVIGATING' else 'INACTIVE',
                calls=self.supervision_calls,pending=bool(self.reviews_inflight()),inflight=self.reviews_inflight(),
                target_hz=1/self.supervision_interval,observed_request_hz=sum(t>=now-5 for t in self.request_times)/5.,
                last_review_age_seconds=now-self.last_review_at if self.last_review_at else None))
        if self.phase=='NAVIGATING':
            allowed=self.navigation_choices(now)
        elif self.phase in ('DECIDING','WAITING_JEV','PAUSED','EVALUATING'):
            allowed,_=self.available(state,now)
        elif self.phase in ('HELP','ACK_RECHECK'):allowed=['STOP']
        else:allowed=['STOP','REQUEST_HELP']
        data['action_availability']={a:dict(available=a in allowed,description=description,
            reason='Available at this decision point' if a in allowed else
            ('Acknowledge intervention, then wait for fresh localization checks' if self.phase in ('HELP','ACK_RECHECK') else 'Requires navigation to stop first' if self.phase=='NAVIGATING' else 'Unavailable in this stage or current safety/attempt conditions'))
            for a,description in ACTIONS.items()}
        data['last_veto']=self.last_veto
        self.demo_pub.publish(String(data=json.dumps(data,allow_nan=False)))
        if self.goal:self.goal_pub.publish(self.goal_message())
        data['events']=list(self.events)
        if self.grid:
            data['map']=dict(width=self.grid.width,height=self.grid.height,resolution=self.grid.resolution,
                origin=self.grid.origin,blocked=[i for i,v in enumerate(self.grid.data) if v>=50 or v<0])
        self.dashboard.snapshot=json.dumps(data,allow_nan=False)

    def reset_stop(self,req,res):
        res.success=False;res.message='Use /demo/start or /demo/acknowledge_help for mission mode';return res

def main(args=None):
    rclpy.init(args=args);node=MissionNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.owner_pub.publish(String(data='NONE'));node.publish_velocity(0.,0.)
        node.dashboard.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
