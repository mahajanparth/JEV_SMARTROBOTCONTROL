"""Small endpoint matcher with rotated map origins and laser extrinsics."""
from dataclasses import dataclass
import math
from typing import Optional


def transform_point(x, y, pose):
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * x - s * y, py + s * x + c * y


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin: tuple
    data: object

    def cell(self, x, y):
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return (math.floor((c * (x - ox) + s * (y - oy)) / self.resolution),
                math.floor((-s * (x - ox) + c * (y - oy)) / self.resolution))

    def occupancy(self, ix, iy):
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return self.data[iy * self.width + ix]
        return None

    def near_occupied(self, ix, iy, threshold, tolerance):
        for dy in range(-tolerance, tolerance + 1):
            for dx in range(-tolerance, tolerance + 1):
                cell = self.occupancy(ix + dx, iy + dy)
                if cell is not None and cell >= threshold:
                    return True
        return False


@dataclass
class ScoreResult:
    score: Optional[float]
    valid_beams: int
    reason: str = ''


def scan_map_score(grid, ranges, angle_min, angle_increment, range_min, range_max,
                   pose, laser_pose=(0.0, 0.0, 0.0), stride=8,
                   occupied_threshold=65, tolerance=2, minimum_beams=12):
    if stride < 1 or minimum_beams < 1 or not 1 <= occupied_threshold <= 100 or tolerance < 0:
        raise ValueError('Invalid scan-map scoring parameters')
    if (grid is None or pose is None or grid.width <= 0 or grid.height <= 0 or grid.resolution <= 0 or
            len(grid.data) != grid.width * grid.height):
        return ScoreResult(None, 0, 'Invalid or missing map/pose')
    valid, matches = 0, 0
    for i in range(0, len(ranges), max(1, stride)):
        distance = ranges[i]
        # No-return/max-range beams are not observed occupied endpoints.
        if not math.isfinite(distance) or distance <= 0 or not range_min <= distance < range_max:
            continue
        angle = angle_min + i * angle_increment
        bx, by = transform_point(distance * math.cos(angle), distance * math.sin(angle), laser_pose)
        mx, my = transform_point(bx, by, pose)
        ix, iy = grid.cell(mx, my)
        occupancy = grid.occupancy(ix, iy)
        matched = grid.near_occupied(ix, iy, occupied_threshold, tolerance)
        if occupancy == -1 and not matched:
            continue  # Unknown cells are not evidence of disagreement.
        valid += 1  # Out-of-map endpoints count as mismatches.
        matches += int(matched)
    if valid < minimum_beams:
        return ScoreResult(None, valid, 'Too few usable hit endpoints')
    return ScoreResult(matches / valid, valid)


def scan_clearance(ranges, angle_min, angle_increment, range_min, range_max,
                   laser_pose=(0.0, 0.0, 0.0), rear=False):
    """Return None when required angular coverage is missing/invalid.

    +inf means no return up to range_max; NaN/zero/-inf are unknown.
    Distances are from base origin, so thresholds must include robot radius.
    """
    covered = set()
    distances = []
    required = set(range(12, 24)) if rear else set(range(36))
    for i, distance in enumerate(ranges):
        direction = (angle_min + i * angle_increment + laser_pose[2]) % (2 * math.pi)
        sector = int(direction / (2 * math.pi) * 36) % 36
        if sector not in required:
            continue
        if distance == math.inf and math.isfinite(range_max):
            clearance = range_max - math.hypot(*laser_pose[:2])
        elif math.isfinite(distance) and range_min <= distance <= range_max and distance > 0:
            angle = angle_min + i * angle_increment
            clearance = math.hypot(*transform_point(distance * math.cos(angle),
                                                  distance * math.sin(angle), laser_pose))
        else:
            # A blind beam can conceal a nearby obstacle. Fail closed.
            return None
        covered.add(sector)
        distances.append(clearance)
    if not required.issubset(covered) or not distances:
        return None
    return min(distances)
