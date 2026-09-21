"""Bounded odometry-relative differential-drive movement; no AMCL dependency."""
import math
from .mission_policy import validate_target
from .localization_monitor import angle_delta

def path_clear(x, y, points, radius=.25):
    length2 = x*x+y*y
    for px, py in points:
        t = max(0., min(1., (px*x+py*y)/max(length2, 1e-9)))
        if math.hypot(px-t*x, py-t*y) < radius:
            return False
    return True

class LocalMove:
    def __init__(self, origin, x, y, now):
        validate_target(x, y)
        self.origin = self.previous = origin
        c, s = math.cos(origin.yaw), math.sin(origin.yaw)
        self.target = (origin.x+c*x-s*y, origin.y+s*x+c*y)
        self.started = now
        self.travel = 0.
        self.result = None
        self.reason = ''

    def step(self, odom, points, now, clearance):
        def end(success, reason):
            self.result, self.reason = success, reason
            return 0., 0.
        if self.result is not None:
            return 0., 0.
        if odom is None or now-odom.received_at > .75 or clearance is None or clearance < .25:
            return end(False, 'Local move lost fresh sensor clearance or odometry')
        delta = math.hypot(odom.x-self.previous.x, odom.y-self.previous.y)
        if delta > .2 or abs(angle_delta(odom.yaw,self.previous.yaw)) > .5:
            return end(False, 'Odometry discontinuity')
        self.travel += delta
        self.previous = odom
        if now-self.started > 30 or self.travel > .55:
            return end(False, 'Local move timeout or distance budget exceeded')
        dx, dy = self.target[0]-odom.x, self.target[1]-odom.y
        if math.hypot(dx,dy) < .025:
            return end(True, 'Local target reached using odometry')
        c,s = math.cos(odom.yaw),math.sin(odom.yaw)
        rx,ry = c*dx+s*dy,-s*dx+c*dy
        if not path_clear(rx,ry,points):
            return end(False, 'Local target path blocked')
        heading = math.atan2(ry,rx)
        if abs(heading) > .10:
            return 0., max(-.35,min(.35,heading))
        return min(.08,math.hypot(dx,dy)*.5), max(-.2,min(.2,heading))
