import math

import pytest

from bld120_sim.experiments import (MotionGuard, PROFILES, mark_moves_to_pose, marks_to_pose,
                                    profile_at, profile_distance, profile_duration)


def test_steps_profile_fits_a_short_corridor():
    assert profile_distance(PROFILES['steps']) == pytest.approx(0.98)
    assert profile_duration(PROFILES['steps']) == pytest.approx(20.0)


def test_staircase_returns_to_zero_between_levels_and_stays_below_max_rpm():
    speeds = [v for _, v, _ in PROFILES['staircase']]
    assert all(b == 0.0 for b in speeds[1::2])
    max_mps = 180.0 / 60.0 * math.pi * 0.043
    assert max(abs(v) for v in speeds) < max_mps


def test_profile_at_walks_the_segments_and_ends():
    profile = ((1.0, 0.1, 0.0), (2.0, 0.0, 0.5))
    assert profile_at(profile, 0.5) == (0.1, 0.0)
    assert profile_at(profile, 1.5) == (0.0, 0.5)
    assert profile_at(profile, 3.0) is None


def test_guard_quiet_while_inside_limits():
    guard = MotionGuard(max_distance=1.0, max_duration=10.0)
    guard.start(0.0)
    guard.update_pose(5.0, 5.0, 0.0)      # origin is wherever odometry starts
    guard.update_pose(5.5, 5.5, 1.0)
    assert guard.check(1.1) is None


def test_guard_stops_on_distance_duration_and_lost_odometry():
    guard = MotionGuard(max_distance=1.0, max_duration=10.0, stale_timeout=0.5)
    guard.start(0.0)
    guard.update_pose(0.0, 0.0, 0.0)
    guard.update_pose(1.2, 0.0, 1.0)
    assert 'from its start' in guard.check(1.0)

    guard = MotionGuard(max_duration=10.0)
    guard.start(0.0)
    guard.update_pose(0.0, 0.0, 10.4)
    assert 'longer than' in guard.check(10.5)

    guard = MotionGuard()
    guard.start(0.0)
    assert guard.check(0.3) is None       # grace period at the start
    assert 'no odometry' in guard.check(0.6)
    guard.update_pose(0.0, 0.0, 1.0)
    assert 'no odometry' in guard.check(1.6)


def test_marks_to_pose():
    # Robot facing +y, rear mark at (1, 0), axle 4 cm ahead of the rear mark.
    x, y, yaw = marks_to_pose(front=(1.0, 0.10), rear=(1.0, 0.0), axle_from_rear=0.04)
    assert (x, y, yaw) == pytest.approx((1.0, 0.04, math.pi / 2))


def test_mark_moves_to_pose():
    # Pure translation by 1 m: both marks move the same, heading unchanged.
    x, y, yaw = mark_moves_to_pose((1.0, 0.0), (1.0, 0.0), mark_spacing=0.12,
                                   axle_from_rear=0.06)
    assert (x, y, yaw) == pytest.approx((1.06, 0.0, 0.0))
