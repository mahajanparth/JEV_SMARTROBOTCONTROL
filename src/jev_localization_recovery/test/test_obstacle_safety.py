"""Geometry, evidence validity, and explicit stop-release behavior."""
import math
import pytest
from jev_localization_recovery.obstacle_safety import ObstacleSafety, SafetyConfig, scan_points

@pytest.mark.parametrize('command,point',[
    ((.15,0.),(.30,0.)), ((-.15,0.),(-.30,0.)),
    ((0.,.5),(.2,0.)), ((.15,.6),(.30,.08)),
])
def test_stopping_envelope_blocks_each_motion(command,point):
    result=ObstacleSafety().evaluate(command,(0.,0.),[point],.1,1.,owner='NAV')
    assert result['status']=='BLOCKED' and result['applied']==[0.,0.]


def test_side_obstacle_outside_swept_path_does_not_stop():
    result=ObstacleSafety().evaluate((.15,0.),(0.,0.),[(0.,.6)],.1,1.)
    assert result['status']=='CLEAR' and result['applied']==[.15,0.]


def test_slowdown_preserves_curvature():
    result=ObstacleSafety().evaluate((.1,.2),(0.,0.),[(.48,0.)],.1,1.)
    assert result['status']=='SLOW' and 0<result['scale']<1
    assert result['applied'][1]/result['applied'][0]==pytest.approx(2)


def test_measured_velocity_protects_after_zero_request():
    result=ObstacleSafety().evaluate((0.,0.),(.15,0.),[(.3,0.)],.1,1.)
    assert result['status']=='BLOCKED'


def test_release_requires_clearance_and_owner_revocation():
    safety=ObstacleSafety()
    assert safety.evaluate((.15,0.),(0.,0.),[(.3,0.)],.1,1.,owner='NAV')['latched']
    assert safety.evaluate((0.,0.),(0.,0.),[(.3,0.)],.1,2.,owner='NONE')['latched']
    assert safety.evaluate((.15,0.),(0.,0.),[(2.,0.)],.1,3.,owner='NAV')['latched']
    result=safety.evaluate((.15,0.),(0.,0.),[(2.,0.)],.1,4.1,owner='NAV')
    assert result['latched'] and result['applied']==[0.,0.]
    assert not safety.evaluate((0.,0.),(0.,0.),[(2.,0.)],.1,4.2,owner='NONE')['latched']

@pytest.mark.parametrize('fault,age', [('Missing TF',.1),(None,.6)])
def test_bad_evidence_stops(fault,age):
    result=ObstacleSafety().evaluate((.1,0.),(0.,0.),[(3.,0.)],age,1.,fault)
    assert result['status']=='SENSOR_FAULT' and result['applied']==[0.,0.]

@pytest.mark.parametrize('value',[float('nan'),-math.inf,0.,-.1,4.])
def test_invalid_sector_rejected(value):
    scan=[1.]*360;scan[100]=value
    with pytest.raises(ValueError):scan_points(scan,0.,math.pi/180,.1,3.,(0.,0.,0.))


def test_infinite_returns_bounded_and_transformed():
    points=scan_points([math.inf]*360,0.,math.pi/180,.1,3.,(.1,0.,math.pi/2))
    assert points[0]==pytest.approx((.1,3.))
    with pytest.raises(ValueError):scan_points([1.]*180,0.,math.pi/180,.1,3.,(0.,0.,0.))


def test_invalid_parameters_rejected():
    with pytest.raises(ValueError):SafetyConfig(braking=0.)
