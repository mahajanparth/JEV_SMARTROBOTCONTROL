"""Recovery transitions with real ROS nodes and deterministic sensor/clock inputs.

No robot or simulator is used. ROS domain 198 is separate from the transport
smoke test and optional AMCL demo. Pure tests still run when ROS is unavailable.
"""
import json
import time
from threading import Event
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy', reason='Recovery node tests require ROS 2')
pytest.importorskip('tf2_ros', reason='Recovery node tests require tf2_ros')

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger

from jev_localization_recovery.localization_monitor import Health, LocalizationState
from jev_localization_recovery.recovery_actions import MotionSample
from jev_localization_recovery.recovery_node import RecoveryNode
from jev_localization_recovery.recovery_selector import RecoveryAction, RecoveryDecision


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init(args=[], domain_id=198)
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def rig(monkeypatch):
    node = RecoveryNode()
    commands, events = [], []
    record_velocity = lambda linear, angular: commands.append((linear, angular))
    monkeypatch.setattr(node, 'publish_velocity', record_velocity)
    monkeypatch.setattr(node.actions, 'publish_velocity', record_velocity)
    monkeypatch.setattr(node, 'event_publisher', SimpleNamespace(
        publish=lambda message: events.append(json.loads(message.data))))
    node.p['enable_recovery'] = True
    node.actions.config.spin_angle = 0.2
    node.bindings['cmd_vel'] = '/state_machine_test/cmd_vel'
    monkeypatch.setattr(node, 'count_subscribers', lambda topic: 1)
    monkeypatch.setattr(node, 'get_publishers_info_by_topic', lambda topic: [
        SimpleNamespace(node_name=node.get_name(), node_namespace=node.get_namespace())])
    node.pose_message = PoseWithCovarianceStamped()
    node.pose_message.header.stamp.sec = 100
    try:
        yield SimpleNamespace(node=node, commands=commands, events=events)
    finally:
        node.destroy_node()


def evidence(node, **changes):
    fields = dict(status=Health.DEGRADED, data_ready=True,
                  pose_uncertainty=0.7, scan_map_score=0.4,
                  min_obstacle_distance=1.0, rear_clearance=1.0,
                  **node.attempts)
    fields.update(changes)
    return LocalizationState(**fields)


def begin_spin(rig, before=None):
    node = rig.node
    before = before or evidence(node)
    node.step_recovery(before, 10.0)
    assert node.machine == 'LOCALIZATION_BAD'
    selected_at = 10.0 + node.p['bad_hold_time'] + 0.01
    node.step_recovery(before, selected_at)
    assert node.machine == 'SELECT_RECOVERY'
    started_at = selected_at + 0.01
    node.motion_sample = MotionSample(0.0, 0.0, 0.0, started_at)
    node.step_recovery(before, started_at)
    assert node.machine == 'EXECUTING_RECOVERY'
    assert node.active_action == 'SPIN'
    assert node.attempts['spin_attempts'] == 1
    return started_at


def finish_spin(rig, before=None):
    node = rig.node
    started_at = begin_spin(rig, before)
    node.motion_sample = MotionSample(0.0, 0.0, 0.05, started_at + 0.1)
    node.step_recovery(evidence(node), started_at + 0.1)
    assert rig.commands[-1][1] > 0
    finished_at = started_at + 0.2
    node.motion_sample = MotionSample(0.0, 0.0, 0.2, finished_at)
    node.step_recovery(evidence(node), finished_at)
    assert node.machine == 'EVALUATING'
    assert rig.commands[-1] == (0.0, 0.0)
    return finished_at


def fresh_after(node, now):
    node.scan_at = now
    node.pose_message.header.stamp.sec = 101


@pytest.mark.parametrize('metric', ['scan_score', 'uncertainty'])
def test_success_requires_sustained_health_and_accepts_either_improved_metric(rig, metric):
    node = rig.node
    before = evidence(node, pose_uncertainty=0.1 if metric == 'scan_score' else 0.9,
                      scan_map_score=0.4 if metric == 'scan_score' else 0.8)
    evaluated_at = finish_spin(rig, before)
    after = evidence(node, status=Health.HEALTHY, pose_uncertainty=0.1, scan_map_score=0.8)
    first = evaluated_at + 0.1
    fresh_after(node, first)
    node.step_recovery(after, first)
    assert node.machine == 'EVALUATING'
    node.step_recovery(after, first + node.p['healthy_hold_time'] - 0.01)
    assert node.machine == 'EVALUATING'
    done = first + node.p['healthy_hold_time'] + 0.01
    fresh_after(node, done)
    node.step_recovery(after, done)
    assert node.machine == 'MONITORING'
    assert node.active_action == 'NONE'
    assert not any(node.attempts.values())
    assert node.cooldown_until == pytest.approx(done + node.p['cooldown'])
    event = rig.events[-1]
    assert event['event'] == 'RECOVERY SUCCESS'
    assert event['before']['scan_map_score'] == before.scan_map_score
    assert event['after']['status'] == 'HEALTHY'
    assert rig.commands[-1] == (0.0, 0.0)


@pytest.mark.parametrize('missing', [
    'healthy_status', 'fresh_data', 'new_scan', 'new_pose', 'metric_improvement',
])
def test_evaluation_rejects_each_missing_success_requirement(rig, missing):
    node = rig.node
    before = evidence(node)
    if missing == 'metric_improvement':
        # A recent pose jump can make good unchanged metrics DEGRADED; merely
        # letting that jump expire must not count as a successful recovery.
        before = evidence(node, pose_uncertainty=0.1, scan_map_score=0.8,
                          pose_jump_distance=1.0)
    evaluated_at = finish_spin(rig, before)
    after = evidence(node, status=Health.HEALTHY, pose_uncertainty=0.1, scan_map_score=0.8)
    first = evaluated_at + 0.1
    fresh_after(node, first)
    if missing == 'healthy_status':
        after.status = Health.DEGRADED
    elif missing == 'fresh_data':
        after.data_ready = False
    elif missing == 'new_scan':
        node.scan_at = evaluated_at
    elif missing == 'new_pose':
        node.pose_message.header.stamp.sec = 100
    node.step_recovery(after, first)
    node.step_recovery(after, first + node.p['healthy_hold_time'] + 0.1)
    assert node.machine == 'EVALUATING'
    assert not any(event['event'] == 'RECOVERY SUCCESS' for event in rig.events)


def test_unhealthy_observation_restarts_healthy_hold(rig):
    node = rig.node
    evaluated_at = finish_spin(rig)
    healthy = evidence(node, status=Health.HEALTHY, pose_uncertainty=0.1, scan_map_score=0.8)
    first = evaluated_at + 0.1
    fresh_after(node, first)
    node.step_recovery(healthy, first)
    node.step_recovery(evidence(node), first + 0.6)
    node.step_recovery(healthy, first + 0.8)
    node.step_recovery(healthy, first + 1.2)
    assert node.machine == 'EVALUATING'
    fresh_after(node, first + 1.9)
    node.step_recovery(healthy, first + 1.9)
    assert rig.events[-1]['event'] == 'RECOVERY SUCCESS'


def test_failed_evaluation_obeys_cooldown_and_attempt_limit(rig):
    node = rig.node
    node.p['max_attempts'] = node.selector.max_attempts = 1
    evaluated_at = finish_spin(rig)
    failed_at = evaluated_at + node.p['evaluation_time'] + 0.01
    node.step_recovery(evidence(node), failed_at)
    assert node.machine == 'SELECT_RECOVERY'
    assert rig.events[-1]['event'] == 'RECOVERY FAILED'
    assert node.attempts['spin_attempts'] == 1
    decision_count = sum(event['event'] == 'DECISION' for event in rig.events)
    node.step_recovery(evidence(node), node.cooldown_until - 0.01)
    assert node.machine == 'SELECT_RECOVERY'
    assert sum(event['event'] == 'DECISION' for event in rig.events) == decision_count
    node.step_recovery(evidence(node), node.cooldown_until + 0.01)
    assert node.machine == 'STOPPED'
    assert rig.events[-1]['event'] == 'STOP'
    assert rig.commands[-1] == (0, 0)


def test_stop_is_latched_until_explicit_reset_and_reset_has_cooldown(rig, monkeypatch):
    node = rig.node
    node.latch_stop('test safety stop')
    marker = len(rig.commands)
    for status in (Health.HEALTHY, Health.LOST, Health.DEGRADED):
        node.step_recovery(evidence(node, status=status), 20.0)
    assert node.machine == 'STOPPED'
    assert all(command == (0, 0) for command in rig.commands[marker:])
    node.attempts['spin_attempts'] = 2
    monkeypatch.setattr('jev_localization_recovery.recovery_node.time.monotonic', lambda: 50.0)
    response = node.reset_stop(Trigger.Request(), Trigger.Response())
    assert response.success
    assert node.machine == 'MONITORING' and node.active_action == 'NONE'
    assert not any(node.attempts.values())
    assert node.cooldown_until == pytest.approx(50.0 + node.p['cooldown'])
    node.step_recovery(evidence(node), 50.0)
    node.step_recovery(evidence(node), node.cooldown_until - 0.01)
    assert node.machine == 'LOCALIZATION_BAD'
    assert all(command == (0, 0) for command in rig.commands[marker:])


def test_reset_cannot_interrupt_pending_global_localization(rig):
    node = rig.node
    future = Future()
    requests = []

    def call_async(request):
        requests.append(request)
        return future

    node.global_client = SimpleNamespace(service_is_ready=lambda: True, call_async=call_async)
    node.global_service = '/state_machine_test/reinitialize_global_localization'
    node.machine = 'SELECT_RECOVERY'
    node.motion_sample = MotionSample(0.0, 0.0, 0.0, 20.0)
    node.step_recovery(evidence(node, status=Health.LOST, global_service_available=True), 20.0)
    assert node.machine == 'EXECUTING_RECOVERY'
    assert node.actions.stage == 'WAIT_SERVICE'
    assert len(requests) == 1 and not future.done()
    response = node.reset_stop(Trigger.Request(), Trigger.Response())
    assert not response.success
    assert node.machine == 'EXECUTING_RECOVERY'
    assert node.active_action == 'GLOBAL_RELOCALIZE'
    assert node.actions.stage == 'WAIT_SERVICE'
    assert node.attempts['global_relocalization_attempts'] == 1


@pytest.mark.parametrize('problem', ['missing_subscriber', 'competing_publisher'])
@pytest.mark.parametrize('during_motion', [False, True])
def test_command_ownership_failure_stops_before_or_during_motion(rig, monkeypatch, problem, during_motion):
    node = rig.node
    if during_motion:
        now = begin_spin(rig) + 0.1
        node.motion_sample = MotionSample(0.0, 0.0, 0.02, now)
        node.step_recovery(evidence(node), now)
        assert rig.commands[-1][1] > 0
    else:
        node.machine = 'SELECT_RECOVERY'
        now = 20.0
        node.motion_sample = MotionSample(0.0, 0.0, 0.0, now)
    if problem == 'missing_subscriber':
        monkeypatch.setattr(node, 'count_subscribers', lambda topic: 0)
    else:
        monkeypatch.setattr(node, 'get_publishers_info_by_topic', lambda topic: [
            SimpleNamespace(node_name=node.get_name(), node_namespace=node.get_namespace()),
            SimpleNamespace(node_name='navigation_controller', node_namespace='/')])
    marker = len(rig.commands)
    node.step_recovery(evidence(node), now + 0.1)
    assert node.machine == 'STOPPED'
    assert rig.events[-1]['event'] == 'STOP'
    assert all(command == (0, 0) for command in rig.commands[marker:])


@pytest.mark.parametrize('malformed', [False, True])
def test_decision_provider_failure_latches_stop(rig, malformed):
    node = rig.node
    node.machine = 'SELECT_RECOVERY'
    node.motion_sample = MotionSample(0.0, 0.0, 0.0, 20.0)

    def choose(_state):
        if malformed:
            return {'selected_action': 'DRIVE_FAST', 'confidence': float('nan')}
        raise ValueError('Invalid decision probabilities')

    node.selector = SimpleNamespace(choose=choose)
    node.step_recovery(evidence(node), 20.0)
    assert node.machine == 'STOPPED'
    assert rig.events[-1]['event'] == 'STOP'
    assert all(command == (0, 0) for command in rig.commands)


def jev_spin(confidence=.8):
    return RecoveryDecision(RecoveryAction.SPIN, confidence,
        {'SPIN': .85, 'GLOBAL_RELOCALIZE': .1, 'BACKTRACK_AND_SPIN': .01, 'STOP': .04},
        'Test response', 'jev-test')


def pending_jev(rig):
    node = rig.node
    node.p['selector_provider'] = 'jev'
    node.machine = 'WAITING_DECISION'
    node.decision_started = 20.0
    node.decision_state = evidence(node)
    node.decision_future = Future()
    node.motion_sample = MotionSample(0, 0, 0, 20.1)
    return node.decision_future


def test_jev_inference_does_not_block_callbacks_or_issue_motion(rig):
    node = rig.node
    entered, release = Event(), Event()
    calls = []
    def choose(state):
        calls.append(state)
        entered.set()
        release.wait(2)
        return jev_spin()
    node.selector = SimpleNamespace(choose=choose)
    node.p['selector_provider'] = 'jev'
    node.machine = 'SELECT_RECOVERY'
    node.motion_sample = MotionSample(0, 0, 0, 20)
    start = time.monotonic()
    try:
        node.step_recovery(evidence(node), 20)
        assert time.monotonic() - start < .5
        assert entered.wait(.5)
        assert node.machine == 'WAITING_DECISION'
        node.step_recovery(evidence(node), 20.1)
        assert len(calls) == 1
        assert all(c == (0, 0) for c in rig.commands)
        release.set()
        node.decision_future.result(timeout=1)
        node.step_recovery(evidence(node), 20.2)
        assert node.machine == 'EXECUTING_RECOVERY'
        assert rig.events[-1]['provider'] == 'jev'
        assert rig.events[-1]['model'] == 'jev-test'
    finally:
        release.set()


def test_timed_out_jev_response_cannot_execute_after_stop_or_reset(rig):
    node = rig.node
    future = pending_jev(rig)
    now = 20 + node.p['decision_timeout'] + .1
    node.motion_sample = MotionSample(0, 0, 0, now)
    node.step_recovery(evidence(node), now)
    assert node.machine == 'STOPPED'
    assert not node.reset_stop(Trigger.Request(), Trigger.Response()).success
    future.set_result(jev_spin())
    node.step_recovery(evidence(node), now + .1)
    assert node.machine == 'STOPPED'
    assert all(c == (0, 0) for c in rig.commands)
    assert node.reset_stop(Trigger.Request(), Trigger.Response()).success
    assert node.decision_future is None


@pytest.mark.parametrize('problem', ['scan', 'odometry', 'request_error', 'confidence', 'stale_state'])
def test_jev_result_is_rechecked_against_current_evidence(rig, problem):
    node = rig.node
    future = pending_jev(rig)
    current = evidence(node)
    if problem == 'request_error':
        future.set_exception(RuntimeError('sensitive upstream detail'))
    else:
        future.set_result(jev_spin(.1 if problem == 'confidence' else .8))
    if problem == 'scan':
        current.min_obstacle_distance = None
    elif problem == 'odometry':
        node.motion_sample = None
    elif problem == 'stale_state':
        current.scan_map_score += .2
    node.step_recovery(current, 20.1)
    assert node.machine == ('MONITORING' if problem == 'stale_state' else 'STOPPED')
    assert all(c == (0, 0) for c in rig.commands)
    assert 'sensitive upstream detail' not in json.dumps(rig.events)


def test_attempt_cap_cannot_be_bypassed_by_jev(rig):
    node = rig.node
    node.p['selector_provider'] = 'jev'
    node.attempts['spin_attempts'] = node.p['max_attempts']
    node.apply_decision(jev_spin(), evidence(node), 20)
    assert node.machine == 'STOPPED'
    assert all(c == (0, 0) for c in rig.commands)
