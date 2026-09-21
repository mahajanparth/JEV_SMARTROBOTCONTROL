import math
import pytest
from jev_localization_recovery.mission_policy import sample_mission, validate_target, MissionSelector
from jev_localization_recovery.local_move import LocalMove, path_clear
from jev_localization_recovery.recovery_actions import MotionSample
from jev_localization_recovery.scan_map_score import Grid


def test_random_mission_connected_reproducible_and_clear():
    data=[0]*400
    for y in range(20):data[y*20+10]=100
    grid=Grid(20,20,.5,(2.,3.,0.),data)
    start,goal=sample_mission(grid,42,clearance=.5,min_distance=2.)
    assert (start,goal)==sample_mission(grid,42,clearance=.5,min_distance=2.)
    assert (start[0]<7)==(goal[0]<7)
    assert math.dist(start[:2],goal[:2])>=2
    assert grid.occupancy(*grid.cell(*start[:2]))==0

@pytest.mark.parametrize('xy',[(.51,0),(.4,.4),(float('nan'),0),(0,0),(True,0)])
def test_local_invalid_targets(xy):
    with pytest.raises(ValueError):validate_target(*xy)


def test_local_target_is_frozen_in_start_frame():
    move=LocalMove(MotionSample(1,2,math.pi/2,0),.4,0,0)
    assert move.target==pytest.approx((1,2.4))
    assert move.step(MotionSample(1,2,math.pi/2,.1),[],.1,1.)[0]>0
    # Move in small increments: a teleport must not count as completion.
    for i in range(1,5):move.step(MotionSample(1,2+i*.1,math.pi/2,i*.2),[],i*.2,1.)
    assert move.result is True


def test_local_collision_and_stale_odom_stop():
    assert not path_clear(.4,0,[(.3,.1)])
    move=LocalMove(MotionSample(0,0,0,0),.4,0,0)
    assert move.step(MotionSample(0,0,0,.1),[(.3,0)],.1,1.)==(0,0)
    assert move.result is False
    move=LocalMove(MotionSample(0,0,0,0),.4,0,0)
    assert move.step(MotionSample(0,0,0,0),[],1.,1.)==(0,0)
    assert move.result is False


def test_local_odometry_discontinuity():
    move=LocalMove(MotionSample(0,0,0,0),.4,0,0)
    move.step(MotionSample(.4,0,0,.1),[],.1,1.)
    assert move.result is False


def test_mission_choice_validation():
    body={'answers':{'action':{'type':'choice','choice':'STOP','confidence':.7,'probabilities':{'STOP':.8,'SPIN':.2}}}}
    assert MissionSelector.parse_choice(body,'action',['STOP','SPIN'])['confidence']==.7
    with pytest.raises(ValueError):MissionSelector.parse_choice(body,'action',['SPIN'])
    body['answers']['action']['probabilities']['STOP']=float('nan')
    with pytest.raises(ValueError):MissionSelector.parse_choice(body,'action',['STOP','SPIN'])


def test_choice_allows_decimal_rounding_but_not_invalid_mass():
    body={'answers':{'action':{'type':'choice','choice':'SPIN','confidence':.6,
        'probabilities':{'SPIN':.67,'STOP':.32}}}}
    assert MissionSelector.parse_choice(body,'action',['SPIN','STOP'])['choice']=='SPIN'
    body['answers']['action']['probabilities']['STOP']=.2
    with pytest.raises(ValueError):MissionSelector.parse_choice(body,'action',['SPIN','STOP'])


def test_route_deviation_uses_segments_not_waypoint_spacing():
    from jev_localization_recovery.navigation_supervision import route_deviation
    assert route_deviation((5.,2.,0.),[[0.,0.],[10.,0.]])==2.
    assert route_deviation((12.,0.,0.),[[0.,0.],[10.,0.]])==2.
    assert route_deviation((0.,0.,0.),[]) is None
