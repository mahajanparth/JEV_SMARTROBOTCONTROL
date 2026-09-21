import math
from concurrent.futures import Future

from jev_localization_recovery.localization_monitor import LocalizationState
from jev_localization_recovery.recovery_actions import ActionConfig, MotionHistory, MotionSample, RecoveryActions
from jev_localization_recovery.recovery_selector import RecoveryAction as A


def test_spin_uses_odometry_and_stops_on_obstacles():
    commands = []
    actions = RecoveryActions(lambda v, w: commands.append((v, w)), lambda: None, ActionConfig())
    state = LocalizationState(min_obstacle_distance=1.)
    actions.start(A.SPIN, state, MotionSample(0, 0, 0, 0), 0)
    for i in range(1, 65):
        yaw = math.atan2(math.sin(i * .1), math.cos(i * .1))
        actions.tick(state, MotionSample(0, 0, yaw, i * .3), i * .3)
    assert actions.result is True and commands[-1] == (0., 0.)
    actions.start(A.SPIN, state, MotionSample(0, 0, 0, 20), 20)
    state.min_obstacle_distance = .1
    actions.tick(state, MotionSample(0, 0, 0, 20.1), 20.1)
    assert actions.safety_failure and commands[-1] == (0., 0.)


def test_missing_scan_stale_odom_and_service_timeout_stop():
    commands = []
    future = Future()
    actions = RecoveryActions(lambda v, w: commands.append((v, w)), lambda: future, ActionConfig())
    state = LocalizationState(min_obstacle_distance=1.)
    actions.start(A.GLOBAL_RELOCALIZE, state, MotionSample(0, 0, 0, 0), 0)
    actions.tick(state, MotionSample(0, 0, 0, 4), 4)
    assert actions.result is False and actions.safety_failure
    assert all(cmd == (0., 0.) for cmd in commands)
    actions.start(A.SPIN, state, MotionSample(0, 0, 0, 5), 5)
    actions.tick(state, MotionSample(0, 0, 0, 5), 6)
    assert 'stale odometry' in actions.reason
    state.min_obstacle_distance = None
    actions.start(A.SPIN, state, MotionSample(0, 0, 0, 7), 7)
    assert actions.safety_failure


def test_global_service_completes_before_spin():
    future = Future()
    commands = []
    actions = RecoveryActions(lambda v, w: commands.append((v, w)), lambda: future, ActionConfig())
    state = LocalizationState(min_obstacle_distance=1.)
    actions.start(A.GLOBAL_RELOCALIZE, state, MotionSample(0, 0, 0, 0), 0)
    assert actions.stage == 'WAIT_SERVICE'
    future.set_result(None)
    actions.tick(state, MotionSample(0, 0, 0, .1), .1)
    actions.tick(state, MotionSample(0, 0, 0, 1.2), 1.2)
    actions.tick(state, MotionSample(0, 0, 0, 1.3), 1.3)
    assert commands[-1] == (0., .35)


def test_backtrack_uses_local_distance_then_spin_and_checks_rear():
    commands = []
    actions = RecoveryActions(lambda v, w: commands.append((v, w)), lambda: None, ActionConfig())
    state = LocalizationState(min_obstacle_distance=1., rear_clearance=1., backtrack_available=True)
    actions.start(A.BACKTRACK_AND_SPIN, state, MotionSample(3, 2, 0, 0), 0)
    actions.tick(state, MotionSample(2.9, 2, 0, 1), 1)
    assert commands[-1] == (-.08, 0.)
    for i in range(2, 7):
        actions.tick(state, MotionSample(3 - .1 * i, 2, 0, i), i)
    assert actions.stage == 'SPIN' and commands[-1] == (0., 0.)
    actions.tick(state, MotionSample(2.4, 2, 0, 6.1), 6.1)
    assert commands[-1] == (0., .35)
    actions.start(A.BACKTRACK_AND_SPIN, state, MotionSample(3, 2, 0, 10), 10)
    state.rear_clearance = .2
    actions.tick(state, MotionSample(3, 2, 0, 10.1), 10.1)
    assert actions.safety_failure and commands[-1] == (0., 0.)


def test_history_rejects_curves_stale_samples_and_odometry_resets():
    history = MotionHistory()
    for i in range(10):
        history.add(MotionSample(i * .08, 0, 0, i * .2))
    assert history.forward_distance(1.8) >= .6
    assert history.moving_forward(1.8)
    assert history.forward_distance(3) == 0
    history.add(MotionSample(.72, 0, .4, 1.9))
    assert history.forward_distance(1.9) == 0
    history.add(MotionSample(9, 9, .4, 2))
    assert history.forward_distance(2) == 0


def test_backtrack_requires_history_and_stops_if_path_diverges():
    actions = RecoveryActions(lambda v, w: None, lambda: None, ActionConfig())
    state = LocalizationState(min_obstacle_distance=1., rear_clearance=1.)
    actions.start(A.BACKTRACK_AND_SPIN, state, MotionSample(0, 0, 0, 0), 0)
    assert actions.safety_failure
    state.backtrack_available = True
    actions.start(A.BACKTRACK_AND_SPIN, state, MotionSample(0, 0, 0, 1), 1)
    actions.tick(state, MotionSample(-.05, .1, 0, 1.1), 1.1)
    assert actions.safety_failure
