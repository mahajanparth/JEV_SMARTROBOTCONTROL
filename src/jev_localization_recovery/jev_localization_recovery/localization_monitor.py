"""ROS-independent metrics. Covariance is evidence, never proof of delocalization."""
from dataclasses import asdict, dataclass
from enum import Enum
import math
from typing import Optional


class Health(str, Enum):
    HEALTHY = 'HEALTHY'
    DEGRADED = 'DEGRADED'
    LOST = 'LOST'


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


@dataclass
class Thresholds:
    yaw_weight: float = 0.5
    uncertainty_degraded: float = 0.5
    uncertainty_lost: float = 2.0
    score_healthy: float = 0.65
    score_lost: float = 0.25
    jump_distance: float = 0.75
    jump_yaw: float = 0.8
    jump_hold_sec: float = 2.0


@dataclass
class LocalizationState:
    pose_uncertainty: Optional[float] = None
    scan_map_score: Optional[float] = None
    pose_jump_distance: float = 0.0
    pose_jump_yaw: float = 0.0
    min_obstacle_distance: Optional[float] = None
    rear_clearance: Optional[float] = None
    spin_attempts: int = 0
    global_relocalization_attempts: int = 0
    backtrack_attempts: int = 0
    status: Health = Health.DEGRADED
    data_ready: bool = False
    worsened_while_moving: bool = False
    backtrack_available: bool = False
    global_service_available: bool = False
    reason: str = 'Waiting for fresh pose, map, scan, and TF'

    def to_dict(self):
        result = asdict(self)
        result['status'] = self.status.value
        return result


def classify(state, thresholds):
    if not state.data_ready or state.pose_uncertainty is None or state.scan_map_score is None:
        return Health.DEGRADED, 'UNKNOWN: insufficient fresh localization evidence'
    if state.pose_uncertainty >= thresholds.uncertainty_lost:
        return Health.LOST, 'Severe covariance (one signal only)'
    if state.scan_map_score < thresholds.score_lost:
        return Health.LOST, 'Very poor scan-map agreement'
    jumped = (state.pose_jump_distance >= thresholds.jump_distance or
              state.pose_jump_yaw >= thresholds.jump_yaw)
    if jumped and state.scan_map_score < thresholds.score_healthy:
        return Health.LOST, 'Pose jump and poor scan-map agreement'
    if (jumped or state.pose_uncertainty >= thresholds.uncertainty_degraded or
            state.scan_map_score < thresholds.score_healthy):
        return Health.DEGRADED, 'Elevated covariance, poor agreement, or recent jump'
    return Health.HEALTHY, 'Covariance and scan-map agreement are reasonable'


class LocalizationMonitor:
    def __init__(self, thresholds):
        self.thresholds = thresholds
        self.pose = None
        self.uncertainty = None
        self.received_at = None
        self.jump_at = -math.inf
        self.jump_distance = self.jump_yaw = 0.0

    def update(self, x, y, yaw, covariance, now):
        variances = [covariance[i] for i in (0, 7, 35)]
        # AMCL can emit tiny negative variances after particle collapse because
        # covariance subtracts nearly equal floating-point second moments.
        if not all(math.isfinite(v) for v in (x, y, yaw, *variances)) or min(variances) < -1e-9:
            self.uncertainty = None
            return
        variances = [max(0.0, value) for value in variances]
        if self.pose is not None:
            distance = math.hypot(x - self.pose[0], y - self.pose[1])
            rotation = abs(angle_delta(yaw, self.pose[2]))
            if distance >= self.thresholds.jump_distance or rotation >= self.thresholds.jump_yaw:
                self.jump_at, self.jump_distance, self.jump_yaw = now, distance, rotation
        self.pose, self.received_at = (x, y, yaw), now
        self.uncertainty = variances[0] + variances[1] + self.thresholds.yaw_weight * variances[2]

    def jumps(self, now):
        if now - self.jump_at > self.thresholds.jump_hold_sec:
            return 0.0, 0.0
        return self.jump_distance, self.jump_yaw
