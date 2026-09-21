"""Nonblocking, bounded motion executor; every tick rechecks sensor safety."""
from dataclasses import dataclass
from collections import deque
import math

from .localization_monitor import angle_delta
from .recovery_selector import RecoveryAction as Action


@dataclass
class MotionSample:
    x: float
    y: float
    yaw: float
    received_at: float


@dataclass
class ActionConfig:
    spin_speed: float = 0.35
    spin_angle: float = 2 * math.pi
    spin_clearance: float = 0.35
    rear_clearance: float = 0.45
    max_odom_age: float = 0.75
    action_timeout: float = 45.0
    service_timeout: float = 3.0
    global_settle_time: float = 1.0
    backtrack_speed: float = 0.08
    backtrack_distance: float = 0.6


class MotionHistory:
    """Conservative short reverse path: only a recent, straight forward segment."""
    def __init__(self):
        self.samples = deque(maxlen=600)

    def add(self, sample):
        if sample is None:
            self.samples.clear()
            return
        if self.samples:
            previous = self.samples[-1]
            if (sample.received_at - previous.received_at > 0.75 or
                    math.hypot(sample.x - previous.x, sample.y - previous.y) > 0.25 or
                    abs(angle_delta(sample.yaw, previous.yaw)) > 0.5):
                self.samples.clear()
        self.samples.append(sample)

    def forward_distance(self, now):
        if len(self.samples) < 2 or now - self.samples[-1].received_at > 0.75:
            return 0.0
        end, distance = self.samples[-1], 0.0
        c, s = math.cos(end.yaw), math.sin(end.yaw)
        previous = end
        for sample in reversed(list(self.samples)[:-1]):
            if now - sample.received_at > 20.0 or abs(angle_delta(sample.yaw, end.yaw)) > 0.15:
                break
            dx, dy = previous.x - sample.x, previous.y - sample.y
            forward, lateral = c * dx + s * dy, -s * dx + c * dy
            total_lateral = -s * (end.x - sample.x) + c * (end.y - sample.y)
            if forward < -0.003 or abs(lateral) > 0.02 or abs(total_lateral) > 0.05:
                break
            distance = max(0.0, c * (end.x - sample.x) + s * (end.y - sample.y))
            previous = sample
        return distance

    def moving_forward(self, now):
        if len(self.samples) < 2:
            return False
        recent = [s for s in self.samples if now - s.received_at <= 0.5]
        if len(recent) < 2:
            return False
        first, last = recent[0], recent[-1]
        return ((last.x - first.x) * math.cos(last.yaw) +
                (last.y - first.y) * math.sin(last.yaw)) > 0.01


class RecoveryActions:
    def __init__(self, publish_velocity, request_global, config):
        self.publish_velocity = publish_velocity
        self.request_global = request_global
        self.config = config
        self.stage = 'IDLE'
        self.result = None
        self.safety_failure = False
        self.reason = ''
        self.previous_odom = None
        self.rotation = 0.0
        self.action = Action.STOP

    def stop(self, reason, safety=False, success=False):
        self.publish_velocity(0.0, 0.0)
        self.stage = 'DONE'
        self.result, self.reason, self.safety_failure = success, reason, safety

    def safety_reason(self, state, odom, now):
        if state.min_obstacle_distance is None:
            return 'Missing fresh scan/TF or incomplete 360-degree clearance'
        if state.min_obstacle_distance < self.config.spin_clearance:
            return 'Obstacle inside spin clearance'
        if odom is None or now - odom.received_at > self.config.max_odom_age:
            return 'Missing or stale odometry'
        if self.stage == 'BACKTRACK' and (
                state.rear_clearance is None or state.rear_clearance < self.config.rear_clearance):
            return 'Rear clearance unsafe or unknown'
        return ''

    def start(self, action, state, odom, now):
        self.action = Action(action)
        self.result, self.safety_failure, self.reason = None, False, ''
        self.started_at, self.stage_at = now, now
        self.previous_odom = odom
        self.rotation = 0.0
        self.stage = 'SPIN'
        self.publish_velocity(0.0, 0.0)
        if action == Action.STOP:
            self.stop('STOP selected', safety=True)
            return
        reason = self.safety_reason(state, odom, now)
        if reason:
            self.stop(reason, safety=True)
        elif action == Action.GLOBAL_RELOCALIZE:
            self.stage = 'WAIT_SERVICE'
            try:
                self.future = self.request_global()
                if self.future is None:
                    self.stop('AMCL global localization service unavailable')
            except Exception as exc:
                self.stop('Global localization request failed: ' + str(exc))
        elif action == Action.BACKTRACK_AND_SPIN:
            self.stage = 'BACKTRACK'
            self.backtrack_origin = odom
            reason = self.safety_reason(state, odom, now)
            if not state.backtrack_available or reason:
                self.stop(reason or 'No recent straight forward path to retrace', safety=True)

    def tick(self, state, odom, now):
        if self.result is not None or self.stage == 'IDLE':
            return
        reason = self.safety_reason(state, odom, now)
        if reason:
            self.stop(reason, safety=True)
            return
        if now - self.started_at > self.config.action_timeout:
            self.stop('Action timed out before odometry confirmed completion', safety=True)
            return
        if self.stage == 'WAIT_SERVICE':
            self.publish_velocity(0.0, 0.0)
            if self.future.done():
                try:
                    self.future.result()
                except Exception as exc:
                    self.stop('AMCL service failed: ' + str(exc))
                    return
                self.stage, self.stage_at = 'SETTLING', now
            elif now - self.stage_at > self.config.service_timeout:
                self.stop('AMCL service timed out', safety=True)
            return
        if self.stage == 'SETTLING':
            self.publish_velocity(0.0, 0.0)
            if now - self.stage_at >= self.config.global_settle_time:
                self.stage, self.previous_odom = 'SPIN', odom
            return
        if odom.received_at != self.previous_odom.received_at:
            delta = angle_delta(odom.yaw, self.previous_odom.yaw)
            distance = math.hypot(odom.x - self.previous_odom.x, odom.y - self.previous_odom.y)
            if abs(delta) > 0.5 or distance > 0.25:
                self.stop('Odometry discontinuity; refusing to infer motion', safety=True)
                return
            if self.stage == 'SPIN':
                self.rotation += delta
            self.previous_odom = odom
        if self.stage == 'BACKTRACK':
            origin = self.backtrack_origin
            dx, dy = odom.x - origin.x, odom.y - origin.y
            distance = -(math.cos(origin.yaw) * dx + math.sin(origin.yaw) * dy)
            lateral = -math.sin(origin.yaw) * dx + math.cos(origin.yaw) * dy
            if distance < -0.03 or abs(lateral) > 0.08 or abs(angle_delta(odom.yaw, origin.yaw)) > 0.2:
                self.stop('Backtrack deviated from the recorded straight path', safety=True)
            elif distance >= self.config.backtrack_distance - 0.01:
                self.publish_velocity(0.0, 0.0)
                self.stage, self.rotation = 'SPIN', 0.0
            else:
                self.publish_velocity(-self.config.backtrack_speed, 0.0)
            return
        if self.rotation >= self.config.spin_angle - 0.03:
            self.stop('Spin completed using odometry', success=True)
        else:
            self.publish_velocity(0.0, self.config.spin_speed)
