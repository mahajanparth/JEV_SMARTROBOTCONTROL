"""Validated recovery decisions from the mock policy or TypeSafe Jev API."""
from dataclasses import dataclass
from enum import Enum
import json
import math
import os
from pathlib import Path
from urllib import error, request

from .localization_monitor import Health


class RecoveryAction(str, Enum):
    SPIN = 'SPIN'
    GLOBAL_RELOCALIZE = 'GLOBAL_RELOCALIZE'
    BACKTRACK_AND_SPIN = 'BACKTRACK_AND_SPIN'
    STOP = 'STOP'


@dataclass
class RecoveryDecision:
    selected_action: RecoveryAction
    confidence: float
    probabilities: dict
    explanation: str = ''
    model: str = ''

    def __post_init__(self):
        self.selected_action = RecoveryAction(self.selected_action)
        self.probabilities = {RecoveryAction(k): v for k, v in self.probabilities.items()}
        if set(self.probabilities) != set(RecoveryAction):
            raise ValueError('Decision must include exactly the four recovery actions')
        if not all(math.isfinite(p) and 0 <= p <= 1 for p in self.probabilities.values()):
            raise ValueError('Probabilities must be finite and in [0, 1]')
        if abs(sum(self.probabilities.values()) - 1.0) > 1e-6:
            raise ValueError('Probabilities must sum to one')
        if not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError('Confidence must be in [0, 1]')


class RecoverySelector:
    def choose(self, state):
        raise NotImplementedError


class MockRecoverySelector(RecoverySelector):
    def __init__(self, spin_clearance=0.35, rear_clearance=0.45, max_attempts=3):
        self.spin_clearance = spin_clearance
        self.rear_clearance = rear_clearance
        self.max_attempts = max_attempts

    def choose(self, state):
        A = RecoveryAction
        attempts = state.spin_attempts + state.global_relocalization_attempts + state.backtrack_attempts
        if (not state.data_ready or state.min_obstacle_distance is None or
                state.min_obstacle_distance < self.spin_clearance or attempts >= self.max_attempts):
            action, reason = A.STOP, 'Unsafe/unknown clearance or recovery attempt limit'
        elif state.status == Health.HEALTHY:
            action, reason = A.STOP, 'No recovery needed'
        elif state.status == Health.LOST and state.global_relocalization_attempts == 0 and state.global_service_available:
            action, reason = A.GLOBAL_RELOCALIZE, 'Lost localization; global initialization has not been tried'
        elif (state.worsened_while_moving and state.backtrack_available and state.backtrack_attempts == 0 and
              state.rear_clearance is not None and state.rear_clearance >= self.rear_clearance):
            action, reason = A.BACKTRACK_AND_SPIN, 'Recent forward path and safe rear clearance'
        elif state.spin_attempts < 1:
            action, reason = A.SPIN, 'Gather more observations with a slow rotation'
        elif state.global_relocalization_attempts == 0 and state.global_service_available:
            action, reason = A.GLOBAL_RELOCALIZE, 'Spin failed; try global initialization'
        else:
            action, reason = A.STOP, 'Available recoveries exhausted'
        probabilities = {A.SPIN: .14, A.GLOBAL_RELOCALIZE: .14, A.BACKTRACK_AND_SPIN: .08, A.STOP: .04}
        probabilities[action] += 1.0 - sum(probabilities.values())
        return RecoveryDecision(action, probabilities[action], probabilities, reason)


class JevRecoverySelector(RecoverySelector):
    """Official TypeSafe Choice API; run choose() outside the ROS executor thread."""
    ENDPOINT = 'https://api.typesafe.ai/v1/systemone'

    def __init__(self, model='jev-latest', timeout=3.0, key_file='', limits=None):
        self.model, self.timeout = model, timeout
        self.limits = limits or dict(spin_clearance=0.35, rear_clearance=0.45, max_attempts=3)
        key_file = key_file or os.environ.get('TYPESAFE_API_KEY_FILE', '')
        try:
            self._key = (Path(key_file).read_text().strip() if key_file else
                         os.environ.get('TYPESAFE_API_KEY', '').strip())
        except OSError:
            raise ValueError('Cannot read the configured TypeSafe key file') from None
        if not self._key or any(c.isspace() for c in self._key):
            raise ValueError('Set TYPESAFE_API_KEY or a key file containing one raw API key')
        # Do not forward Authorization through redirects to another endpoint.
        class NoRedirect(request.HTTPRedirectHandler):
            def redirect_request(self, req, fp, code, msg, headers, newurl):
                return None
        self._opener = request.build_opener(NoRedirect())

    def payload(self, state):
        return {
            'model': self.model,
            'state': json.dumps(dict(robot=state.to_dict(), limits=self.limits), allow_nan=False),
            'questions': {'recovery': {
                'type': 'choice',
                'instructions': (
                    'Which single recovery action is most appropriate for this mobile robot now? '
                    'Use the localization evidence, available capabilities, clearance in meters, '
                    'and previous attempts. Covariance alone does not prove delocalization. '
                    'Prefer a first global relocalization for LOST when available. '
                    'For DEGRADED with no previous spin attempt prefer spin, or retracing a verified path if worsening while moving. '
                    'If spin_attempts is at least one and localization remains bad, prefer an available '
                    'GLOBAL_RELOCALIZE when global_relocalization_attempts is zero, even for DEGRADED. '
                    'Attempt counts describe recoveries already tried; do not keep spinning after failed spins. '
                    'Unknown safety evidence, '
                    'unsafe clearance, or exhausted attempt limits require STOP.'),
                'criteria': {
                    'SPIN': 'First slow full rotation for additional laser observations when clearance is safe and spin_attempts is zero.',
                    'GLOBAL_RELOCALIZE': 'Call the available AMCL global reset service then rotate; appropriate for LOST or continued DEGRADED after a spin failed, when not already tried.',
                    'BACKTRACK_AND_SPIN': 'Reverse along verified recent odometry history then rotate; requires backtrack_available, worsened_while_moving, and safe rear clearance.',
                    'STOP': 'Remain stopped when evidence, clearance, available actions, or attempt limits do not allow safe recovery.',
                },
            }},
        }

    @staticmethod
    def parse_response(body):
        try:
            answer = body['answers']['recovery']
            if answer['type'] != 'choice':
                raise ValueError
            decision = RecoveryDecision(
                answer['choice'], answer['confidence'], answer['probabilities'],
                'TypeSafe Jev Choice result; safety checks are applied locally',
                str(body.get('model', 'unknown')))
            if decision.probabilities[decision.selected_action] < max(decision.probabilities.values()):
                raise ValueError
            return decision
        except (KeyError, ValueError, TypeError, AttributeError):
            raise ValueError('Invalid TypeSafe Choice response schema') from None

    def choose(self, state):
        body = json.dumps(self.payload(state), allow_nan=False).encode('utf-8')
        req = request.Request(self.ENDPOINT, data=body, method='POST', headers={
            'Authorization': 'Bearer ' + self._key, 'Content-Type': 'application/json',
            'User-Agent': 'jev-localization-recovery/0.1'})
        try:
            with self._opener.open(req, timeout=self.timeout) as response:
                data = response.read(65537)
            if len(data) > 65536:
                raise ValueError('TypeSafe response exceeded the size limit')
            return self.parse_response(json.loads(data))
        except error.HTTPError as exc:
            # No response bodies/headers in exceptions: they may contain secrets.
            raise RuntimeError('TypeSafe HTTP error {}'.format(exc.code)) from None
        except (error.URLError, OSError, TimeoutError):
            raise RuntimeError('TypeSafe request failed or timed out') from None
        except (ValueError, TypeError):
            raise ValueError('Invalid TypeSafe response') from None
