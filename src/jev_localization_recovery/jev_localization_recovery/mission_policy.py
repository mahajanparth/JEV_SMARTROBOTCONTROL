"""Pure mission generation, bounded local targets, and validated Jev decisions."""
import json
import math
import random
from collections import deque
from urllib import request, error
from .recovery_selector import JevRecoverySelector

ACTIONS = {
    'CONTINUE_NAVIGATION': 'Keep the current Nav2 goal running when localization and navigation evidence are safe; turning in place can be normal progress.',
    'SPIN': 'First safe full rotation to acquire localization observations.',
    'GLOBAL_RELOCALIZE': 'Reset global AMCL hypotheses and rotate. Prefer when lost or a spin failed.',
    'BACKTRACK_AND_SPIN': 'Retrace verified recent straight forward history then rotate.',
    'MOVE_LOCAL': 'Move to one of the safe robot-relative targets to disambiguate localization; maximum 0.5 m.',
    'NAVIGATE_TO_GOAL': 'Start Nav2 navigation when localization_ready is true and navigation has not started.',
    'PAUSE_NAVIGATION': 'Cancel active navigation and stop for reassessment when progress or localization is concerning; if already stopped, remain paused briefly.',
    'RESUME_NAVIGATION': 'Resume the original destination when localization_ready is true after interruption.',
    'REQUEST_HELP': 'Stop and request operator assistance if actions failed, limits reached, or evidence is unsafe.',
    'STOP': 'End the mission stopped when continuing is inappropriate.',
}
LOCAL_TARGETS = {'forward': (.4, 0.), 'forward_left': (.3, .3),
                 'forward_right': (.3, -.3), 'left': (0., .4),
                 'right': (0., -.4), 'backward': (-.4, 0.)}

def validate_target(x, y):
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (x, y)):
        raise ValueError('Coordinates must be finite numbers')
    if not .05 <= math.hypot(x, y) <= .5:
        raise ValueError('Local displacement must be between 0.05 and 0.5 m')
    return x, y

def sample_mission(grid, seed, clearance=.5, min_distance=3.):
    """Largest connected component of conservatively inflated known-free cells."""
    width, height = grid.width, grid.height
    radius = math.ceil(clearance / grid.resolution)
    blocked = {(i % width, i // width) for i, v in enumerate(grid.data) if v < 0 or v >= 50}
    safe = set()
    # Square inflation is conservative and inexpensive on this small demo map.
    for y in range(radius, height-radius):
        for x in range(radius, width-radius):
            if all((xx, yy) not in blocked for yy in range(y-radius, y+radius+1)
                   for xx in range(x-radius, x+radius+1)):
                safe.add((x, y))
    largest = set()
    while safe:
        first = min(safe)
        component, queue = {first}, deque([first])
        safe.remove(first)
        while queue:
            x, y = queue.popleft()
            for p in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
                if p in safe:
                    safe.remove(p); component.add(p); queue.append(p)
        if len(component) > len(largest):
            largest = component
    rng, cells = random.Random(seed), sorted(largest)
    if not cells:
        raise ValueError('No connected space with requested clearance')
    def world(cell):
        x, y = (cell[0]+.5)*grid.resolution, (cell[1]+.5)*grid.resolution
        c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
        return (grid.origin[0]+c*x-s*y, grid.origin[1]+s*x+c*y)
    for _ in range(500):
        start, goal = world(rng.choice(cells)), world(rng.choice(cells))
        if math.dist(start, goal) >= min_distance:
            return (*start, rng.uniform(-math.pi, math.pi)), (*goal, 0.)
    raise ValueError('Cannot find sufficiently separated connected start and goal')

class MissionSelector(JevRecoverySelector):
    def make_payload(self, state, allowed, targets):
        questions = {'action': {'type': 'choice', 'instructions': (
            'Choose the next safe robot mission action from the available choices. '
            'If localization_ready, navigate or resume immediately. At initial startup AMCL has '
            'already been globally initialized, so choose SPIN to gather observations. '
            'If navigation_started is true and robot status is LOST, prefer an available '
            'GLOBAL_RELOCALIZE to recover the map pose rather than a first spin. '
            'For DEGRADED prefer a first SPIN, then GLOBAL_RELOCALIZE if it failed. '
            'Use MOVE_LOCAL to obtain a different viewpoint '
            'after unsuccessful localization. Do not repeat failed actions indefinitely. '
            'Request help when evidence is unsafe or recovery budget exhausted. '
            'A pause means no progress; prefer a feasible productive action.'),
            'criteria': {a: ACTIONS[a] for a in allowed}}}
        if state.get('mode')=='NAVIGATION_SUPERVISION':
            questions['action']['instructions']=(
                'Review the running Nav2 navigation using current evidence and recent trends. '
                'Choose CONTINUE_NAVIGATION when localization and sensor evidence are safe and progress '
                'is reasonable. Turning in place at the start or near a goal is normal; brief lack '
                'of translation or missing early feedback alone does not require stopping. '
                'Choose PAUSE_NAVIGATION for sustained lack of progress, concerning route deviation, '
                'or deteriorating localization. Choose REQUEST_HELP when an operator is needed, '
                'or STOP to end an unsafe mission. Nav2 is already driving; do not restart its goal.')
        if targets:
            questions['target'] = {'type': 'choice', 'instructions':
                'If moving locally, choose a safe target providing a different viewpoint. '
                'X is forward and Y is left in the robot frame at action start.',
                'criteria': {k: {'x_m': v[0], 'y_m': v[1]} for k,v in targets.items()}}
        return {'model': self.model, 'state': json.dumps(state, allow_nan=False), 'questions': questions}

    @staticmethod
    def parse_choice(body, name, options):
        answer = body['answers'][name]
        probs = answer['probabilities']
        confidence = answer['confidence']
        if (answer['type'] != 'choice' or set(probs) != set(options) or
                answer['choice'] not in options or
                not isinstance(confidence, (int, float)) or not math.isfinite(confidence) or
                not 0 <= confidence <= 1 or
                not all(isinstance(p, (int,float)) and math.isfinite(p) and 0 <= p <= 1 for p in probs.values()) or
                abs(sum(probs.values())-1) > .005*len(probs)+1e-6 or
                probs[answer['choice']] < max(probs.values())):
            raise ValueError('Invalid Jev choice')
        return answer

    def infer(self, payload):
        req = request.Request(self.ENDPOINT, data=json.dumps(payload, allow_nan=False).encode(),
            headers={'Authorization': 'Bearer '+self._key, 'Content-Type': 'application/json'}, method='POST')
        try:
            with self._opener.open(req, timeout=self.timeout) as response:
                data = response.read(65537)
            if len(data) > 65536:
                raise ValueError('Response too large')
            body = json.loads(data)
            action = self.parse_choice(body, 'action', payload['questions']['action']['criteria'])
            decision = dict(action=action['choice'], confidence=action['confidence'],
                            probabilities=action['probabilities'], model=str(body.get('model','unknown')))
            if decision['action'] == 'MOVE_LOCAL':
                target = self.parse_choice(body, 'target', payload['questions']['target']['criteria'])
                xy = payload['questions']['target']['criteria'][target['choice']]
                validate_target(xy['x_m'], xy['y_m'])
                decision.update(x_m=xy['x_m'], y_m=xy['y_m'], target_confidence=target['confidence'])
            # Only structured noncredential response fields are exposed to the UI.
            return decision, {k: body[k] for k in ('model','answers','usage') if k in body}
        except error.HTTPError as exc:
            raise RuntimeError('TypeSafe HTTP {}'.format(exc.code)) from None
        except (error.URLError, OSError):
            raise RuntimeError('TypeSafe network error or timeout') from None
        except (ValueError, KeyError, TypeError, AttributeError):
            raise ValueError('Invalid TypeSafe mission response') from None
