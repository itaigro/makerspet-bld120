import math

import numpy as np
import pytest

from bld120_sim.odometry_model import EncoderModel, FirmwareOdometry

TPR = 4140
R = 0.0215
TRACK = 0.105043


def drive(odom, enc_l, enc_r, wheel_l, wheel_r, steps=200, dt=0.05):
    """Spin the wheels at constant angular rates [rad/s] and integrate."""
    prev = (enc_l.update(0.0), enc_r.update(0.0))
    for k in range(1, steps + 1):
        ticks = (enc_l.update(wheel_l * k * dt), enc_r.update(wheel_r * k * dt))
        odom.step(ticks[0] - prev[0], ticks[1] - prev[1], dt)
        prev = ticks
    return odom.pose


def test_encoder_quantizes_to_ticks():
    enc = EncoderModel(TPR)
    assert enc.update(0.0) == 0
    assert enc.update(2 * math.pi) == TPR
    assert enc.update(2 * math.pi + 0.5 * 2 * math.pi / TPR) == TPR  # half a tick: no count
    assert enc.update(-2 * math.pi / TPR) == -1                      # floor, not round


def test_straight_line_distance():
    odom = FirmwareOdometry(R, R, TRACK, TPR)
    x, y, yaw = drive(odom, EncoderModel(TPR), EncoderModel(TPR), 5.0, 5.0)
    assert x == pytest.approx(5.0 * 10.0 * R, abs=2 * R * 2 * math.pi / TPR)
    assert y == pytest.approx(0.0, abs=1e-6)
    assert yaw == pytest.approx(0.0, abs=1e-6)


def test_turn_in_place_is_counter_clockwise_when_right_wheel_forward():
    odom = FirmwareOdometry(R, R, TRACK, TPR)
    w = 0.5 * TRACK / R  # wheel rate for 1 rad/s body rotation
    x, y, yaw = drive(odom, EncoderModel(TPR), EncoderModel(TPR), -w, w, steps=20)
    assert yaw == pytest.approx(1.0, abs=1e-3)
    # floor() quantizes a backward wheel to one tick more than a forward one,
    # so allow one tick of wheel travel (2*pi*R/TPR ~ 33 um) of translation.
    assert math.hypot(x, y) < 2 * math.pi * R / TPR


def test_yaw_wraps_into_pi():
    odom = FirmwareOdometry(R, R, TRACK, TPR)
    w = 0.5 * TRACK / R
    _, _, yaw = drive(odom, EncoderModel(TPR), EncoderModel(TPR), -w, w, steps=80)  # 4 rad
    assert yaw == pytest.approx(4.0 - 2 * math.pi, abs=2e-3)


def test_wheel_diameter_error_curves_a_straight_path():
    # Firmware believes the left wheel is 1 % larger: a truly straight run
    # looks like a right turn (clockwise, negative yaw).
    odom = FirmwareOdometry(R * 1.01, R, TRACK, TPR)
    _, _, yaw = drive(odom, EncoderModel(TPR), EncoderModel(TPR), 5.0, 5.0)
    expected = (R - 1.01 * R) * 5.0 * 10.0 / TRACK
    assert yaw == pytest.approx(expected, rel=0.01)


def test_slip_is_reproducible_with_seed():
    def run(seed):
        enc_l = EncoderModel(TPR, 0.05, np.random.default_rng(seed))
        enc_r = EncoderModel(TPR, 0.05, np.random.default_rng(seed + 1))
        return drive(FirmwareOdometry(R, R, TRACK, TPR), enc_l, enc_r, 5.0, 5.0)

    assert run(3) == run(3)
    assert run(3) != run(4)
