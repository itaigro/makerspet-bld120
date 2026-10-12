import math

import pytest

from bld120_sim.firmware_model import (MotorModel, RPM_PER_RAD_S, SampledPI, SpeedRamp,
                                       WheelController)


def run_motor(motor, duty, seconds, dt=1e-3):
    for _ in range(round(seconds / dt)):
        motor.step(duty, dt)
    return motor.omega * RPM_PER_RAD_S


def test_motor_reaches_no_load_speed_at_full_duty():
    assert run_motor(MotorModel(200.0, 0.05, 0.1), 1.0, 1.0) == pytest.approx(200.0, rel=1e-3)


def test_motor_time_constant():
    # First-order response: 63.2 % of the final speed after one time constant.
    rpm = run_motor(MotorModel(200.0, 0.05, 0.1), 1.0, 0.05)
    assert rpm / 200.0 == pytest.approx(1 - math.exp(-1), abs=0.01)


def test_motor_speed_is_linear_in_duty_above_dead_zone():
    m = MotorModel(200.0, 0.05, 0.1)
    half = run_motor(m, 0.55, 1.0)
    assert half == pytest.approx(200.0 * (0.55 - 0.1) / 0.9, rel=1e-3)


def test_stiction_holds_a_stopped_wheel():
    m = MotorModel(200.0, 0.05, 0.1)
    assert run_motor(m, 0.09, 1.0) == 0.0
    assert run_motor(m, -0.09, 1.0) == 0.0


def test_friction_stops_but_never_reverses_a_coasting_wheel():
    m = MotorModel(200.0, 0.05, 0.1)
    run_motor(m, 0.5, 0.5)
    assert run_motor(m, 0.05, 1.0) == 0.0  # below dead zone: friction brings it to rest


def test_pid_integral_scales_with_sample_time():
    pid = SampledPI(kp=0.0, ki=0.003, reference_period=0.03)
    out = pid.update(target=100.0, measured=0.0, h=0.06)  # two reference periods
    assert out == pytest.approx(0.003 * 2 * 100.0, rel=1e-6)


def test_pid_output_and_integral_are_clamped():
    pid = SampledPI(kp=0.001, ki=0.003, reference_period=0.03)
    for _ in range(100):
        out = pid.update(1000.0, 0.0, 0.03)
    assert out == 1.0
    assert float(pid.integral) == 1.0  # anti-windup: integral never exceeds the limit


def test_pid_integral_rate_is_independent_of_sample_rate():
    # Same total time, different sampling: the integral must end up the same.
    slow, fast = (SampledPI(kp=0.0, ki=0.003, reference_period=0.03) for _ in range(2))
    slow.update(10.0, 0.0, 0.03)
    for _ in range(15):
        fast.update(10.0, 0.0, 0.002)
    assert float(fast.integral) == pytest.approx(float(slow.integral), rel=1e-5)


def test_pid_derivative_acts_on_measurement_not_target():
    pid = SampledPI(kp=0.0, ki=0.0, reference_period=0.03, kd=0.01)
    assert pid.update(0.0, 5.0, 0.03) == 0.0      # first sample: no derivative
    assert pid.update(100.0, 5.0, 0.03) == 0.0    # target jump alone: no kick
    assert pid.update(100.0, 15.0, 0.03) == pytest.approx(-0.01 * 10.0)


def test_pid_reset_clears_integral():
    pid = SampledPI(kp=0.0, ki=0.003, reference_period=0.03)
    pid.update(50.0, 0.0, 0.03)
    pid.reset(0.0)
    assert float(pid.integral) == 0.0


def test_ramp_duration_follows_max_acceleration():
    ramp = SpeedRamp(accel_max=2.0)
    ramp.command(0.2, 0.0, (0.0, 0.0))
    assert ramp.duration == pytest.approx(0.1)
    halfway = ramp.update(0.05)
    assert halfway[0] == pytest.approx(0.5 * ramp.target[0])


def test_ramp_scales_both_wheels_when_one_exceeds_max_rpm():
    ramp = SpeedRamp(max_rpm=180.0)
    ramp.command(0.4, 2.0, (0.0, 0.0))   # right wheel: 0.505 m/s = 224 rpm > 180
    left, right = ramp.target
    assert max(abs(left), abs(right)) == pytest.approx(180.0)
    unscaled_ratio = (0.4 - 0.105043) / (0.4 + 0.105043)
    assert left / right == pytest.approx(unscaled_ratio)


def test_closed_loop_tracks_target_speed():
    pid = SampledPI(0.001, 0.003, 0.03)
    wheel = WheelController(pid)
    motor = MotorModel(200.0, 0.05, 0.1)
    wheel.set_target_rpm(100.0)
    dt, loop, t_loop = 1e-3, 2e-3, 0.0
    for _ in range(3000):  # 3 s
        t_loop += dt
        if t_loop >= loop - 1e-9:
            wheel.update(math.floor(motor.angle * 4140 / (2 * math.pi)), t_loop)
            t_loop = 0.0
        motor.step(wheel.duty, dt)
    assert motor.omega * RPM_PER_RAD_S == pytest.approx(100.0, abs=3.0)
