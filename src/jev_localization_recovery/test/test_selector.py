import pytest
from jev_localization_recovery.localization_monitor import Health, LocalizationState
from jev_localization_recovery.recovery_selector import (
    MockRecoverySelector, RecoveryAction as A, RecoveryDecision)


def test_selector_priorities_and_probabilities():
    selector = MockRecoverySelector()
    s = LocalizationState(data_ready=True, status=Health.DEGRADED, min_obstacle_distance=1.)
    assert selector.choose(s).selected_action == A.SPIN
    s.status, s.global_service_available = Health.LOST, True
    decision = selector.choose(s)
    assert decision.selected_action == A.GLOBAL_RELOCALIZE
    assert set(decision.probabilities) == set(A)
    assert abs(sum(decision.probabilities.values()) - 1) < 1e-9
    s.global_relocalization_attempts = 1
    s.worsened_while_moving = s.backtrack_available = True
    s.rear_clearance = 1.
    assert selector.choose(s).selected_action == A.BACKTRACK_AND_SPIN
    s.rear_clearance = .1
    assert selector.choose(s).selected_action == A.SPIN
    s.min_obstacle_distance = None
    assert selector.choose(s).selected_action == A.STOP
    s.min_obstacle_distance, s.spin_attempts = 1., 2
    assert selector.choose(s).selected_action == A.STOP


def test_unavailable_global_service_and_bad_provider_output():
    s = LocalizationState(data_ready=True, status=Health.LOST, min_obstacle_distance=1.)
    assert MockRecoverySelector().choose(s).selected_action == A.SPIN
    with pytest.raises(ValueError):
        RecoveryDecision(A.SPIN, .5, {A.SPIN: 1.})
    with pytest.raises(ValueError):
        RecoveryDecision(A.SPIN, .5, {a: float('nan') for a in A})
