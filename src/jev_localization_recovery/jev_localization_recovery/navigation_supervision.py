"""Compact estimated navigation telemetry; no simulation ground truth."""
import math


def route_deviation(pose, path):
    if pose is None or not path:
        return None
    if len(path) == 1:
        return math.dist(pose[:2], path[0])
    distances = []
    for a, b in zip(path, path[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = max(0., min(1., ((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/max(dx*dx+dy*dy,1e-12)))
        distances.append(math.hypot(pose[0]-a[0]-t*dx, pose[1]-a[1]-t*dy))
    return min(distances)
