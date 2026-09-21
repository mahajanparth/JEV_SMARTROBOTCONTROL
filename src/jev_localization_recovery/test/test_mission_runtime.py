"""Controller races and safety policies using isolated ROS, no live model calls."""
from concurrent.futures import Future
from types import SimpleNamespace
import time
import pytest
rclpy=pytest.importorskip('rclpy')
pytest.importorskip('jev_demo_interfaces.srv')
from std_srvs.srv import Trigger
from jev_demo_interfaces.srv import InjectDelocalization
from geometry_msgs.msg import PoseWithCovarianceStamped
from jev_localization_recovery.mission_node import MissionNode
from jev_localization_recovery.localization_monitor import LocalizationState, Health
from jev_localization_recovery.recovery_actions import MotionSample

@pytest.fixture
def mission(monkeypatch):
    import jev_localization_recovery.mission_node as module
    monkeypatch.setenv('TYPESAFE_API_KEY','test-only-placeholder')
    monkeypatch.delenv('TYPESAFE_API_KEY_FILE',raising=False)
    monkeypatch.setattr(module,'Dashboard',lambda *a:SimpleNamespace(snapshot='',close=lambda:None))
    rclpy.init(args=[],domain_id=196)
    node=MissionNode();node.emit=lambda *a,**k:None
    node.safety_status=dict(status="CLEAR",reason="Test fixture",latched=False);node.safety_at=time.monotonic()
    node.publish_velocity=lambda *a:None
    node.owner_pub=SimpleNamespace(publish=lambda m:None)
    yield node
    node.destroy_node();rclpy.shutdown()


def test_stop_during_pending_goal_cancels_late_acceptance(mission):
    node=mission;node.phase='NAV_STARTING';node.nav_send=Future()
    node.halt('STOPPED','Operator stop')
    assert node.phase=='CANCELING' and node.owner=='NONE'
    canceled=[];result=Future()
    handle=SimpleNamespace(accepted=True,get_result_async=lambda:result,
        cancel_goal_async=lambda:canceled.append(True) or Future())
    node.nav_send.set_result(handle);node.nav_poll()
    assert canceled and node.owner=='NONE'
    result.set_result(SimpleNamespace(status=5));node.cancel_started=time.monotonic()-1;node.nav_poll()
    assert node.phase=='STOPPED'


def test_help_ack_reassesses_without_enabling_motion(mission):
    mission.phase='HELP';mission.owner='NONE'
    mission.attempts['spin_attempts']=3
    response=mission.ack_service(None,Trigger.Response())
    assert response.success and mission.phase=='DECIDING' and mission.owner=='NONE'
    assert mission.attempts['spin_attempts']==0


def test_injection_does_not_trigger_recovery_directly(mission):
    mission.phase='NAVIGATING';mission.pose_message=PoseWithCovarianceStamped()
    mission.pose_message.pose.pose.orientation.w=1.
    req=InjectDelocalization.Request();req.dx=4.;req.dy=4.;req.dyaw=1.5;req.variance=.005
    response=mission.inject_service(req,InjectDelocalization.Response())
    assert response.success and mission.phase=='NAVIGATING'
    assert len(mission.injections)==3
    req.dx=float('nan')
    assert not mission.inject_service(req,InjectDelocalization.Response()).success


def test_exhausted_budget_and_unhealthy_navigation_veto(mission):
    node=mission;node.localization_ready=False;node.attempts['spin_attempts']=5
    state=LocalizationState(status=Health.LOST,data_ready=True,min_obstacle_distance=1.)
    allowed,_=node.available(state,time.monotonic())
    assert allowed==['REQUEST_HELP','STOP']
    node.execute(dict(action='NAVIGATE_TO_GOAL',confidence=1.),state,time.monotonic())
    assert node.phase=='HELP' and node.owner=='NONE'


def test_help_ack_waits_for_outstanding_request(mission):
    mission.phase='HELP';mission.pending=Future()
    response=mission.ack_service(None,Trigger.Response())
    assert not response.success and mission.phase=='HELP'


def test_expiring_velocity_owner_and_obstacle_gate():
    from jev_localization_recovery.mission_bridge import MissionBridge
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rclpy.init(args=[],domain_id=195)
    node=MissionBridge();sent=[]
    node.publisher=SimpleNamespace(publish=sent.append)
    try:
        now=time.monotonic();node.scan_at=node.odom_at=now
        node.scan_stamp=node.odom_stamp=node.get_clock().now().nanoseconds*1e-9
        node.points=[(3.,0.)];node.scan_fault=None
        nav=Twist();nav.linear.x=.1
        recovery=Twist();recovery.angular.z=.35
        node.on_owner(String(data='NAV'));node.store('NAV',nav);node.store('RECOVERY',recovery)
        node.relay();assert sent[-1].linear.x==.1 and sent[-1].angular.z==0
        node.on_owner(String(data='RECOVERY'));node.relay()
        assert sent[-1].linear.x==0 and sent[-1].angular.z==0  # old commands flushed
        node.store('RECOVERY',recovery);node.relay();assert sent[-1].angular.z==.35
        node.owner_at=now-1;node.relay();assert sent[-1].angular.z==0
        node.on_owner(String(data='NAV'));node.store('NAV',nav);node.points=[(.1,0.)]
        node.relay();assert sent[-1].linear.x==0
    finally:node.destroy_node();rclpy.shutdown()


def test_healthy_label_alone_cannot_complete_mission(mission):
    node=mission;now=time.monotonic()
    node.phase='NAVIGATING';node.owner='NAV';node.nav_started=True
    node.motion_sample=MotionSample(0,0,0,now)
    node.nav_result=Future();node.nav_result.set_result(SimpleNamespace(status=4))
    node.nav_handle=SimpleNamespace()
    state=LocalizationState(status=Health.HEALTHY,data_ready=True,pose_uncertainty=.01,
        scan_map_score=.8,min_obstacle_distance=1.)
    node.step_recovery(state,now)
    assert node.phase=='DECIDING' and node.owner=='NONE'
    assert not node.localization_ready


def test_stop_while_costmaps_refresh_cannot_start_navigation(mission):
    node=mission;node.phase='PREPARING_NAV';node.owner='NONE'
    node.clear_futures=[Future(),Future()]
    node.halt('STOPPED','Operator stop')
    for future in node.clear_futures:future.set_result(SimpleNamespace())
    node.step_recovery(LocalizationState(),time.monotonic())
    assert node.phase=='STOPPED' and node.nav_send is None


def review_fixture(node, now, action='CONTINUE_NAVIGATION', confidence=.95):
    node.phase='NAVIGATING';node.owner='NAV';node.stage_started=now-5
    node.localization_ready=True;node.goal=(1.,2.,0.)
    node.call_count=1;node.last_applied_review=0;node.next_supervision_at=now+2
    node.last_review_at=now-.2
    node.last_supervision_started=now-.2
    future=Future()
    future.set_result((dict(action=action,confidence=confidence,model='test'),{}))
    node.supervision_requests={1:dict(future=future,started=now-.2,epoch=node.nav_epoch)}
    return LocalizationState(status=Health.HEALTHY,data_ready=True,pose_uncertainty=.01,scan_map_score=1.,min_obstacle_distance=1.)


def test_supervision_continue_preserves_running_goal(mission):
    now=time.monotonic();state=review_fixture(mission,now)
    handle=object();mission.nav_handle=handle
    mission.supervise_navigation(state,now)
    assert mission.phase=='NAVIGATING' and mission.owner=='NAV'
    assert mission.nav_handle is handle and mission.nav_send is None
    assert mission.supervision_status=='MONITORING'

@pytest.mark.parametrize('action,phase',[('PAUSE_NAVIGATION','DECIDING'),('REQUEST_HELP','HELP'),('STOP','STOPPED')])
def test_supervision_interventions_cancel_before_next_action(mission,action,phase):
    now=time.monotonic();state=review_fixture(mission,now,action)
    mission.nav_handle=object()
    mission.supervise_navigation(state,now)
    assert mission.owner=='NONE' and mission.phase=='CANCELING' and mission.next_phase==phase


def test_review_cannot_undo_operator_stop(mission):
    now=time.monotonic();state=review_fixture(mission,now)
    mission.halt('STOPPED','Operator stop')
    mission.supervise_navigation(state,now)
    assert mission.phase=='STOPPED' and mission.owner=='NONE'


def test_review_from_previous_goal_discarded(mission):
    now=time.monotonic();state=review_fixture(mission,now,'STOP')
    mission.nav_epoch+=1
    mission.supervise_navigation(state,now)
    assert mission.phase=='NAVIGATING' and mission.owner=='NAV'

@pytest.mark.parametrize('case',['timeout','error','confidence','changed_health'])
def test_supervision_failure_stops_motion(mission,case):
    now=time.monotonic();state=review_fixture(mission,now,confidence=.1 if case=='confidence' else .95)
    if case=='timeout':mission.supervision_requests[1].update(started=now-20,future=Future())
    if case=='error':
        future=Future();future.set_exception(RuntimeError('test'));mission.supervision_requests[1]['future']=future
    if case=='changed_health':state.status=Health.LOST
    mission.supervise_navigation(state,now)
    assert mission.owner=='NONE' and mission.phase in ('HELP','DECIDING')


def test_navigation_requests_are_bounded_and_contain_telemetry(mission):
    from threading import Event
    release=Event();entered=Event();calls=[]
    def infer(payload):
        calls.append(payload);entered.set();release.wait(2)
        return dict(action='CONTINUE_NAVIGATION',confidence=1.),{}
    mission.mission_selector.infer=infer
    now=time.monotonic();state=review_fixture(mission,now)
    mission.supervision_requests.clear();mission.next_supervision_at=now;mission.supervision_interval=.2
    mission.monitor.pose=(0.,0.,0.);mission.path=[[0.,0.],[1.,0.]]
    try:
        mission.supervise_navigation(state,now)
        assert entered.wait(1)
        for i in range(1,10):mission.supervise_navigation(state,now+i*.2)
        assert mission.supervision_calls==4 and mission.owner=='NAV' and mission.phase=='NAVIGATING'
        import json
        context=json.loads(calls[0]['state'])
        assert context['mode']=='NAVIGATION_SUPERVISION'
        assert context['navigation']['estimated_pose']==[0.,0.,0.]
        assert context['navigation']['route_deviation_m']==0
        assert 'CONTINUE_NAVIGATION' in calls[0]['questions']['action']['criteria']
    finally:release.set()


def test_older_continue_cannot_override_newer_stop(mission):
    now=time.monotonic();state=review_fixture(mission,now)
    newer=Future();newer.set_result((dict(action='STOP',confidence=.9),{}))
    mission.supervision_requests[2]=dict(future=newer,started=now-.1,epoch=mission.nav_epoch)
    mission.supervise_navigation(state,now)
    assert mission.phase=='STOPPED' and mission.owner=='NONE'


def test_late_old_response_does_not_override_newer_continue(mission):
    now=time.monotonic();state=review_fixture(mission,now,'STOP')
    mission.last_applied_review=2
    mission.supervise_navigation(state,now)
    assert mission.phase=='NAVIGATING' and mission.owner=='NAV'


def test_low_confidence_reason_is_specific_and_safe_help_still_allowed(mission):
    now=time.monotonic();state=review_fixture(mission,now,confidence=.19)
    mission.supervise_navigation(state,now)
    assert '0.19' in mission.reason and '0.20' in mission.reason
    state=review_fixture(mission,now,'REQUEST_HELP',.1)
    mission.supervise_navigation(state,now)
    assert mission.phase=='HELP' and 'REQUEST_HELP' in mission.reason

@pytest.mark.parametrize('action',['SPIN','GLOBAL_RELOCALIZE','BACKTRACK_AND_SPIN'])
def test_recovery_spin_finishes_on_sustained_lock(mission,monkeypatch,action):
    node=mission;now=time.monotonic()
    node.phase='EXECUTING';node.active_action=action;node.owner='RECOVERY'
    node.stage_started=now;node.spin_lock_since=None
    monkeypatch.setattr(node,'command_problem',lambda:'')
    monkeypatch.setattr(node.actions,'publish_velocity',lambda *args:None)
    monkeypatch.setattr(node.actions,'tick',lambda *args:None)
    node.actions.stage='SPIN';node.actions.result=None
    state=LocalizationState(status=Health.HEALTHY,data_ready=True,pose_uncertainty=.01,scan_map_score=.95,min_obstacle_distance=1.)
    for elapsed in (0.,1.,2.1):
        node.motion_sample=MotionSample(0,0,0,now+elapsed)
        node.safety_at=now+elapsed  # fresh safety heartbeat for the simulated tick
        node.step_recovery(state,now+elapsed)
        if elapsed<2:assert node.phase=='EXECUTING'
    assert node.phase=='EVALUATING' and node.owner=='NONE'
    assert node.actions.result is True and 'finished early' in node.actions.reason


def test_brief_or_weak_lock_does_not_finish_spin(mission,monkeypatch):
    node=mission;now=time.monotonic();node.phase='EXECUTING';node.active_action='SPIN'
    monkeypatch.setattr(node,'command_problem',lambda:'')
    monkeypatch.setattr(node.actions,'tick',lambda *args:None)
    node.actions.stage='SPIN';node.actions.result=None
    state=LocalizationState(status=Health.HEALTHY,data_ready=True,pose_uncertainty=.01,scan_map_score=.95,min_obstacle_distance=1.)
    for elapsed,score in ((0,.95),(1,.7),(2.1,.95),(3.,.95)):
        state.scan_map_score=score;node.motion_sample=MotionSample(0,0,0,now+elapsed)
        node.safety_at=now+elapsed  # fresh safety heartbeat for the simulated tick
        node.step_recovery(state,now+elapsed)
        assert node.phase=='EXECUTING' and node.actions.result is None

@pytest.mark.parametrize('confidence,accepted',[(.19,False),(.2,True),(.31,True)])
def test_local_target_twenty_percent_gate_before_action_event(mission,monkeypatch,confidence,accepted):
    node=mission;now=time.monotonic();events=[]
    node.emit=lambda kind,**kw:events.append(kind)
    monkeypatch.setattr(node,'available',lambda *args:(['MOVE_LOCAL'],{'forward':(.4,0.)}))
    node.motion_sample=MotionSample(0,0,0,now)
    node.execute(dict(action='MOVE_LOCAL',confidence=.84,target_confidence=confidence,x_m=.4,y_m=0.),LocalizationState(),now)
    assert (node.phase=='EXECUTING') is accepted
    assert ('ACTION' in events) is accepted
    if not accepted:assert node.phase=='HELP' and 'local-target confidence' in node.reason


def test_obstacle_event_aborts_recovery_and_waits(mission):
    import json
    from std_msgs.msg import String
    mission.phase='EXECUTING';mission.owner='RECOVERY';mission.active_action='SPIN'
    mission.on_safety(String(data=json.dumps(dict(status='BLOCKED',reason='Obstacle in path',latched=True))))
    assert mission.phase=='SAFETY_WAIT' and mission.owner=='NONE'
    assert mission.active_action=='NONE'


def test_obstacle_event_cancels_nav_and_discards_review_epoch(mission):
    import json
    from std_msgs.msg import String
    mission.phase='NAVIGATING';mission.owner='NAV';mission.nav_send=Future()
    epoch=mission.nav_epoch
    mission.on_safety(String(data=json.dumps(dict(status='BLOCKED',reason='Obstacle',latched=True))))
    assert mission.phase=='CANCELING' and mission.next_phase=='SAFETY_WAIT'
    assert mission.owner=='NONE' and mission.nav_epoch>epoch


def test_safety_wait_has_no_requests_and_times_out(mission):
    mission.phase='SAFETY_WAIT';now=time.monotonic()
    mission.safety_wait_started=now-16
    mission.safety_status=dict(status='BLOCKED',reason='Obstacle',latched=True)
    before=mission.call_count
    mission.step_recovery(LocalizationState(),now)
    assert mission.phase=='HELP' and mission.call_count==before


def test_clearance_returns_to_decision_without_motion(mission):
    mission.phase='SAFETY_WAIT';mission.owner='NONE'
    before=mission.call_count
    mission.step_recovery(LocalizationState(),time.monotonic())
    assert mission.phase=='DECIDING' and mission.owner=='NONE' and mission.call_count==before


def test_stale_safety_telemetry_removes_motion_choices(mission):
    mission.safety_at=0.;mission.localization_ready=True
    allowed,targets=mission.available(LocalizationState(),time.monotonic())
    assert allowed==['REQUEST_HELP','STOP'] and not targets


def test_bridge_obstacle_stop_flushes_commands_before_release():
    from jev_localization_recovery.mission_bridge import MissionBridge
    from jev_localization_recovery.obstacle_safety import ObstacleSafety, SafetyConfig
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rclpy.init(args=[],domain_id=195)
    node=MissionBridge();sent=[];node.publisher=SimpleNamespace(publish=sent.append)
    node.safety=ObstacleSafety(SafetyConfig(release_delay=.001))
    try:
        node.scan_at=node.odom_at=time.monotonic()
        node.scan_stamp=node.odom_stamp=node.get_clock().now().nanoseconds*1e-9
        node.scan_fault=None;node.points=[(.3,0.)]
        msg=Twist();msg.linear.x=.15
        node.on_owner(String(data='NAV'));node.store('NAV',msg);node.relay()
        assert sent[-1].linear.x==0 and not node.commands
        node.on_owner(String(data='NONE'));node.points=[(3.,0.)];node.relay()
        time.sleep(.005);node.relay()
        assert not node.safety.latched
        node.on_owner(String(data='NAV'));node.relay()
        assert sent[-1].linear.x==0  # no replay of the previously blocked request
        node.store('NAV',msg);node.relay();assert sent[-1].linear.x==.15
        node.odom_at=0.;node.relay()
        assert sent[-1].linear.x==0 and node.safety.latched
    finally:node.destroy_node();rclpy.shutdown()


def replan_fixture(node):
    node.nav_started=True;node.goal=(1.,0.,0.);node.localization_ready=True
    node.monitor.pose=(0.,0.,0.)
    node.planner=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=lambda goal:Future())
    node.follower=SimpleNamespace(server_is_ready=lambda:True,send_goal_async=lambda goal:Future())
    node.replan_reset=SimpleNamespace(publish=lambda msg:None)
    return time.monotonic()


def test_replan_offered_for_blockage_but_not_sensor_fault_or_exhaustion(mission):
    now=replan_fixture(mission)
    mission.safety_status=dict(status='BLOCKED',latched=True)
    assert 'REPLAN_PATH' in mission.available(LocalizationState(),now)[0]
    mission.safety_status['status']='SENSOR_FAULT'
    assert 'REPLAN_PATH' not in mission.available(LocalizationState(),now)[0]
    mission.safety_status['status']='CLEAR';mission.replan_attempts=2
    assert 'REPLAN_PATH' not in mission.navigation_choices(now)


def test_replan_revokes_owner_and_cancels_old_goal_first(mission):
    now=replan_fixture(mission);mission.phase='NAVIGATING';mission.owner='NAV';mission.nav_send=Future()
    mission.apply_navigation_review(dict(action='REPLAN_PATH',confidence=.8),LocalizationState(),now)
    assert mission.owner=='NONE' and mission.phase=='CANCELING'
    assert mission.next_phase=='REPLAN_START' and mission.replan_attempts==1


def test_replan_confidence_gate(mission):
    now=replan_fixture(mission)
    mission.execute(dict(action='REPLAN_PATH',confidence=.19),LocalizationState(),now)
    assert mission.phase=='HELP' and mission.replan_attempts==0


def test_replan_starts_planning_without_clearing_costmaps(mission):
    now=replan_fixture(mission);mission.phase='REPLAN_START';calls=[]
    mission.clear_clients=[SimpleNamespace(call_async=lambda *a:pytest.fail('Cleared obstacles'))]
    mission.planner.send_goal_async=lambda goal:calls.append(goal) or Future()
    mission.step_replan(LocalizationState(),now)
    assert mission.phase=='REPLANNING' and calls[0].goal.pose.position.x==1.
    assert mission.owner=='NONE'


def replan_response(node,valid=True):
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    path=Path();path.header.frame_id='map'
    if valid:
        for x in (0.,.25,.5,.75,1.):
            p=PoseStamped();p.header.frame_id='map';p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.append(p)
    node.phase='REPLANNING';node.nav_result=Future()
    node.nav_result.set_result(SimpleNamespace(status=4,result=SimpleNamespace(path=path)))
    return path


def test_valid_replan_waits_for_safety_then_executes_exact_path(mission):
    now=replan_fixture(mission);path=replan_response(mission);sent=[]
    mission.step_replan(LocalizationState(),now)
    assert mission.phase=='REPLAN_READY' and mission.owner=='NONE'
    mission.follower.send_goal_async=lambda goal,**kw:sent.append(goal) or Future()
    mission.safety_status=dict(status='BLOCKED',latched=True);mission.safety_at=now+2
    mission.step_replan(LocalizationState(),now+2)
    assert not sent
    mission.safety_status=dict(status='CLEAR',latched=False);mission.safety_at=now+3
    mission.step_replan(LocalizationState(),now+3)
    assert mission.phase=='NAV_STARTING' and sent[0].path==path and mission.owner=='NONE'


def test_invalid_replan_requests_help(mission):
    now=replan_fixture(mission);replan_response(mission,False)
    mission.step_replan(LocalizationState(),now)
    assert mission.phase=='HELP' and mission.owner=='NONE'


def test_stop_cancels_late_planner_acceptance(mission):
    replan_fixture(mission);mission.phase='REPLANNING';mission.nav_send=Future()
    mission.halt('STOPPED','Test stop');canceled=[];result=Future()
    mission.nav_send.set_result(SimpleNamespace(accepted=True,get_result_async=lambda:result,
        cancel_goal_async=lambda:canceled.append(True) or Future()))
    mission.nav_poll()
    assert canceled and mission.phase=='CANCELING' and mission.owner=='NONE'


def test_bridge_replan_does_not_override_footprint_collision():
    from jev_localization_recovery.mission_bridge import MissionBridge
    from std_msgs.msg import String
    rclpy.init(args=[],domain_id=195);node=MissionBridge()
    try:
        node.on_owner(String(data='NONE'));node.odom_at=node.scan_at=time.monotonic()
        node.odom_stamp=node.scan_stamp=node.get_clock().now().nanoseconds*1e-9
        node.scan_fault=None;node.points=[(.1,0.)]
        node.safety.latched=True;node.safety.probe=(.15,0.)
        node.on_replan_ready(String(data='CHECK_NEW_ROUTE'));node.relay()
        assert node.safety.latched and node.safety.clear_since is None
        node.safety.probe=(.15,0.);node.measured=(.1,0.)
        node.on_replan_ready(String(data='CHECK_NEW_ROUTE'))
        assert node.safety.probe==(.15,0.)
    finally:node.destroy_node();rclpy.shutdown()


def test_one_blocked_assessment_then_wait_without_periodic_calls(mission):
    now=replan_fixture(mission);mission.phase='SAFETY_WAIT';mission.safety_wait_started=now
    mission.safety_status=dict(status='BLOCKED',latched=True,reason='Obstacle')
    mission.ready_since=now-3
    calls=[]
    def request(*args):
        calls.append(True);mission.set_phase('WAITING_JEV')
    mission.request_decision=request
    state=LocalizationState(status=Health.HEALTHY,data_ready=True,pose_uncertainty=.01,scan_map_score=.95)
    mission.step_recovery(state,now)
    assert len(calls)==1 and mission.blocked_review and mission.blocked_decision
    mission.execute(dict(action='PAUSE_NAVIGATION',confidence=.1),state,now)
    assert mission.phase=='SAFETY_WAIT' and not mission.blocked_decision
    mission.step_recovery(state,now+.1)
    assert len(calls)==1


def test_stale_follow_path_feedback_is_discarded(mission):
    epoch=mission.nav_epoch
    feedback=SimpleNamespace(feedback=SimpleNamespace(distance_to_goal=2.,speed=.1))
    mission.on_path_feedback(feedback,epoch)
    assert mission.nav_feedback['distance_remaining_m']==2.
    mission.nav_epoch+=1;mission.nav_feedback={}
    mission.on_path_feedback(feedback,epoch)
    assert not mission.nav_feedback


def test_planner_result_exception_stays_stopped(mission):
    now=replan_fixture(mission);mission.phase='REPLANNING';mission.nav_result=Future()
    mission.nav_result.set_exception(RuntimeError('Planner transport failed'))
    mission.step_replan(LocalizationState(),now)
    assert mission.phase=='HELP' and mission.owner=='NONE'
