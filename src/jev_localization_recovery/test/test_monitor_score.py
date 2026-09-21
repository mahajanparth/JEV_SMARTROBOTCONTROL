import math

from jev_localization_recovery.localization_monitor import (
    Health, LocalizationMonitor, LocalizationState, Thresholds, classify)
from jev_localization_recovery.scan_map_score import Grid, scan_map_score, scan_clearance, transform_point


def test_rotated_grid_and_floor():
    grid = Grid(5, 5, 0.5, (2, 3, math.pi / 2), [0] * 25)
    assert grid.cell(1.75, 3.75) == (1, 0)
    assert Grid(1, 1, 1, (0, 0, 0), [0]).cell(-0.01, 0) == (-1, 0)
    x, y = transform_point(1, 0, (2, 3, math.pi / 2))
    assert abs(x - 2) < 1e-8 and abs(y - 4) < 1e-8


def test_score_extrinsics_unknown_and_outside():
    grid = Grid(5, 5, 1, (0, 0, 0), [0] * 25)
    grid.data[2 * 5 + 3] = 100
    args = (grid, [1.0], 0, 1, 0.1, 10, (1, 2, 0))
    assert scan_map_score(*args, laser_pose=(1, 0, 0), tolerance=0, minimum_beams=1).score == 1
    assert scan_map_score(*args, tolerance=0, minimum_beams=1).score == 0
    assert scan_map_score(grid, [8.0], 0, 1, .1, 10, (1, 2, 0), minimum_beams=1).score == 0
    grid.data = [-1] * 25
    assert scan_map_score(*args, minimum_beams=1).score is None
    assert scan_map_score(grid, [math.inf, math.nan, 0], 0, 1, .1, 10,
                          (0, 0, 0), minimum_beams=1).score is None


def test_health_unknown_jump_and_covariance():
    t = Thresholds()
    s = LocalizationState(pose_uncertainty=.1, scan_map_score=.9, data_ready=True)
    assert classify(s, t)[0] == Health.HEALTHY
    s.scan_map_score = .5
    assert classify(s, t)[0] == Health.DEGRADED
    s.pose_jump_distance = 2
    assert classify(s, t)[0] == Health.LOST
    s.scan_map_score = None
    assert classify(s, t)[0] == Health.DEGRADED
    s.scan_map_score, s.pose_uncertainty = .9, 3
    assert classify(s, t)[0] == Health.LOST


def test_pose_jump_wrap_hold_and_invalid_covariance():
    m = LocalizationMonitor(Thresholds())
    cov = [0.] * 36
    m.update(0, 0, math.pi - .01, cov, 0)
    m.update(0, 0, -math.pi + .01, cov, 1)
    assert m.jumps(1) == (0, 0)
    m.update(2, 0, 0, cov, 2)
    assert m.jumps(3)[0] == 2
    assert m.jumps(5) == (0, 0)
    cov[0] = -1
    m.update(2, 0, 0, cov, 6)
    assert m.uncertainty is None
    cov[0], cov[7], cov[35] = -2.6e-13, -1.5e-13, -1.3e-14
    m.update(2, 0, 0, cov, 7)
    assert m.uncertainty == 0.0


def test_clearance_fails_closed():
    ranges = [math.inf] * 360
    assert scan_clearance(ranges, 0, math.pi / 180, .1, 10) == 10
    ranges[180] = .2
    assert scan_clearance(ranges, 0, math.pi / 180, .1, 10, rear=True) == .2
    ranges[180] = math.nan
    assert scan_clearance(ranges, 0, math.pi / 180, .1, 10) is None
    assert scan_clearance([1.] * 180, 0, math.pi / 180, .1, 10) is None
