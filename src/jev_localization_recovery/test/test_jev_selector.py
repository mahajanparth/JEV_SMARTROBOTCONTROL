import json
from types import SimpleNamespace
from urllib.error import HTTPError

import pytest

from jev_localization_recovery.localization_monitor import LocalizationState
from jev_localization_recovery.recovery_selector import JevRecoverySelector, RecoveryAction


def response():
    return {'model': 'jev-test', 'answers': {'recovery': {
        'type': 'choice', 'choice': 'GLOBAL_RELOCALIZE', 'confidence': .8,
        'probabilities': {'GLOBAL_RELOCALIZE': .85, 'SPIN': .1, 'BACKTRACK_AND_SPIN': .01, 'STOP': .04}}}}


def test_choice_mapping_preserves_distinct_confidence():
    decision = JevRecoverySelector.parse_response(response())
    assert decision.selected_action == RecoveryAction.GLOBAL_RELOCALIZE
    assert decision.confidence == .8
    assert decision.probabilities[decision.selected_action] == .85
    assert decision.model == 'jev-test'


@pytest.mark.parametrize('corruption', ['missing', 'wrong_type', 'unknown_action', 'nan', 'bad_sum', 'wrong_winner'])
def test_invalid_responses_are_rejected(corruption):
    body = response()
    answer = body['answers']['recovery']
    if corruption == 'missing':
        del answer['probabilities']['STOP']
    elif corruption == 'wrong_type':
        answer['type'] = 'noul'
    elif corruption == 'unknown_action':
        answer['choice'] = 'DRIVE_FORWARD'
    elif corruption == 'nan':
        answer['confidence'] = float('nan')
    elif corruption == 'bad_sum':
        answer['probabilities']['STOP'] = .5
    else:
        answer['choice'] = 'STOP'
    with pytest.raises(ValueError, match='schema'):
        JevRecoverySelector.parse_response(body)


def test_payload_and_transport_exclude_key_from_state_and_errors(monkeypatch, tmp_path):
    secret = 'test-only-not-a-real-key'
    path = tmp_path / 'key'
    path.write_text(secret)
    selector = JevRecoverySelector(key_file=str(path))
    payload = selector.payload(LocalizationState())
    assert secret not in json.dumps(payload)
    assert set(payload['questions']['recovery']['criteria']) == {a.value for a in RecoveryAction}
    assert json.loads(payload['state'])['robot']['scan_map_score'] is None

    def fail(req, timeout):
        assert req.full_url == 'https://api.typesafe.ai/v1/systemone'
        assert req.get_header('Authorization') == 'Bearer ' + secret
        assert timeout == 3.0
        raise HTTPError(req.full_url, 401, secret, {}, None)
    selector._opener = SimpleNamespace(open=fail)
    with pytest.raises(RuntimeError) as error:
        selector.choose(LocalizationState())
    assert str(error.value) == 'TypeSafe HTTP error 401'
    assert secret not in str(error.value)


def test_missing_key_fails_clearly(monkeypatch):
    monkeypatch.delenv('TYPESAFE_API_KEY', raising=False)
    monkeypatch.delenv('TYPESAFE_API_KEY_FILE', raising=False)
    with pytest.raises(ValueError, match='TYPESAFE_API_KEY'):
        JevRecoverySelector()
