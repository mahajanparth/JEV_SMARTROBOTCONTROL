"""Pure obstacle filter. Distances use a conservative circular base footprint."""
from dataclasses import dataclass
import math

# Bounds are shared by ROS validation and the dashboard. Radius cannot undercut Nav2.
SAFETY_LIMITS = dict(radius=(.18,.35), margin=(.02,.3), braking=(.05,2.),
                     reaction=(.05,1.), slowdown=(.05,1.), sensor_timeout=(.1,.75),
                     release_delay=(.2,5.), blocked_timeout=(2.,120.))

@dataclass(frozen=True)
class SafetyConfig:
    radius: float = .18
    margin: float = .05
    braking: float = .3
    reaction: float = .15
    slowdown: float = .25
    sensor_timeout: float = .5
    blocked_timeout: float = 15.
    release_delay: float = 1.

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('Safety parameters must be finite and positive')
        for key,(low,high) in SAFETY_LIMITS.items():
            if not low<=getattr(self,key)<=high:raise ValueError(f'{key} must be {low}..{high}')


def scan_points(ranges, angle_min, increment, range_min, range_max, transform):
    """Require full valid coverage; +inf means clear only to finite sensor range."""
    n=len(ranges)
    if (n<16 or not all(math.isfinite(x) for x in
            (angle_min,increment,range_min,range_max,*transform)) or
            increment<=0 or increment>.1 or range_min<0 or range_max<=range_min or
            abs(n*increment-2*math.pi) > 2*increment):
        raise ValueError('Invalid scan geometry or incomplete 360-degree coverage')
    x,y,yaw=transform; c,s=math.cos(yaw),math.sin(yaw); points=[]
    for i,r in enumerate(ranges):
        if r == math.inf:r=range_max
        elif not math.isfinite(r) or not range_min<=r<=range_max or r<=0:
            raise ValueError('Invalid lidar sector')
        a=angle_min+i*increment; px,py=r*math.cos(a),r*math.sin(a)
        points.append((x+c*px-s*py,y+s*px+c*py))
    return points


def envelope_clearance(points,v,w,age,config):
    # Sample swept circular footprint through latency and conservative braking.
    # Bound endpoint sampling error by inflating the footprint another 5 mm.
    duration=config.reaction+age+abs(v)/config.braking
    steps=max(1,math.ceil(abs(v)*duration/.01))
    clearance=math.inf
    for i in range(steps+1):
        t=duration*i/steps
        if abs(w)<1e-6:x,y=v*t,0.
        else:x,y=v/w*math.sin(w*t),v/w*(1-math.cos(w*t))
        clearance=min(clearance,min(math.hypot(px-x,py-y) for px,py in points)-config.radius-config.margin-.005)
    return clearance


class ObstacleSafety:
    def __init__(self,config=None):
        self.config=config or SafetyConfig()
        self.latched=False; self.probe=(0.,0.); self.clear_since=None

    def evaluate(self,requested,measured,points,age,now,fault=None,owner='NONE'):
        c=self.config; v,w=requested
        status='CLEAR';reason='Swept footprint clear';scale=1.;clearance=None
        if fault or not points or not math.isfinite(age) or age>c.sensor_timeout:
            status='SENSOR_FAULT';reason=fault or 'Fresh obstacle evidence unavailable';scale=0.
        else:
            # Keep testing the blocked trajectory while ownership is revoked.
            command=self.probe if self.latched else requested
            clearance=min(envelope_clearance(points,*motion,age,c) for motion in (command,measured))
            if clearance<=0:
                status='BLOCKED';reason='Obstacle intersects stopping envelope';scale=0.
            elif clearance<c.slowdown and any(command):
                status='SLOW';reason='Obstacle near stopping envelope';scale=max(.05,clearance/c.slowdown)
        if status in ('BLOCKED','SENSOR_FAULT'):
            if not self.latched:self.probe=requested
            self.latched=True;self.clear_since=None
        elif self.latched:
            if self.clear_since is None:self.clear_since=now
            if now-self.clear_since<c.release_delay:
                status='BLOCKED';reason='Waiting for sustained clearance';scale=0.
        ready=self.latched and self.clear_since is not None and now-self.clear_since>=c.release_delay
        # An owner NONE handshake flushes prior commands before a new decision.
        if ready and owner=='NONE':
            self.latched=False;self.probe=(0.,0.)
        elif self.latched:
            scale=0.
            if ready:status='BLOCKED';reason='Waiting for motion ownership to be revoked'
        return dict(status=status,reason=reason,clearance_m=clearance,scale=scale,
                    requested=list(requested),applied=[v*scale,w*scale],latched=self.latched)
