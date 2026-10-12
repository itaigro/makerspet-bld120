"""Wheel drive chain of the BLD-120MM firmware, plus a DC gear-motor model.

Re-implements the behaviour of Kaia.ai firmware v0.8.4 (Apache-2.0):

* ``kaiaai-esp32.ino``: ``cmd_vel`` -> per-wheel target RPM, scaled down
  together when either exceeds ``motor.rpm.max``, then a linear speed ramp
  limited by ``base.wheel.accel.max``.
* ``motor_ctl_kaia`` (``MotorController::update``): wheel RPM measured from
  encoder ticks, PID every ``pid.period`` seconds, *or immediately whenever the
  target RPM changed* (which, during a ramp, is every main-loop pass).
* The wheel speed law itself: a discrete PI(D) controller with gains per
  reference period, integral clamped to the output range (anti-windup) and the
  derivative taken on the measurement; output is a PWM duty in [-1, 1]. It is
  implemented here from its equations in ``docs/WHEEL_CONTROLLER.md``.
* PWM duty 0 on the IN1/IN2 driver is a *hard brake* (both motor terminals
  driven high), i.e. a shorted winding, which the motor model below gets for
  free from applying zero voltage.

The motor model (not from the firmware) is a first-order DC gear motor with
the robot's inertia lumped onto each wheel, plus Coulomb friction expressed as
a PWM dead zone::

    tau * dw/dt = k * (u - u_c * sgn) - w        k = w_no_load / (1 - u_c)

with stiction: a stopped wheel stays stopped while ``|u| <= u_c``. Its three
parameters (no-load speed, time constant ``tau``, dead zone ``u_c``) are what
the real-robot experiments in M3/M4 identify.
"""
from dataclasses import dataclass, field
import math

import numpy as np

F32 = np.float32
RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)


@dataclass
class SampledPI:
    """Wheel speed law of the firmware, written from docs/WHEEL_CONTROLLER.md.

    Gains are per reference period: a sample lasting ``h`` counts as
    ``rho = h / reference_period`` periods. Single-precision arithmetic, as on
    the ESP32.
    """

    kp: float
    ki: float
    reference_period: float
    kd: float = 0.0
    limit: float = 1.0                  # |duty| bound for the integral and the output
    integral: np.float32 = F32(0.0)
    prev_measured: np.float32 = None    # None until the first sample

    def _sat(self, x):
        return F32(min(max(x, F32(-self.limit)), F32(self.limit)))

    def update(self, target, measured, h):
        """One sample: return the duty for target/measured speed after ``h`` seconds."""
        y = F32(measured)
        rho = F32(h) / F32(self.reference_period)
        error = F32(target) - y
        self.integral = self._sat(self.integral + F32(self.ki) * rho * error)
        if self.prev_measured is None:
            derivative = F32(0.0)
        else:
            derivative = -F32(self.kd) * (y - self.prev_measured) / rho
        self.prev_measured = y
        return float(self._sat(F32(self.kp) * error + self.integral + derivative))

    def reset(self, measured):
        """Forget the integral, e.g. when the wheel is stopped on purpose."""
        self.integral = F32(0.0)
        self.prev_measured = F32(measured)


@dataclass
class MotorModel:
    """First-order DC gear motor with a PWM dead zone (see module docstring)."""

    no_load_rpm: float = 200.0     # wheel RPM at duty 1 (12 V N20 150:1, nominal)
    tau: float = 0.05              # s, mechanical time constant incl. robot inertia
    dead_zone: float = 0.10        # duty needed to break away / overcome friction
    omega: float = 0.0             # wheel speed, rad/s
    angle: float = 0.0             # wheel angle, rad

    def step(self, duty: float, dt: float):
        duty = max(-1.0, min(1.0, duty))
        k = self.no_load_rpm / RPM_PER_RAD_S / (1.0 - self.dead_zone)
        if self.omega == 0.0:
            if abs(duty) <= self.dead_zone:
                return  # stiction
            friction = math.copysign(self.dead_zone, duty)
        else:
            friction = math.copysign(self.dead_zone, self.omega)
        target = k * (duty - friction)
        new_omega = self.omega + (target - self.omega) * (dt / self.tau)
        if self.omega != 0.0 and new_omega * self.omega < 0.0 and abs(duty) <= self.dead_zone:
            new_omega = 0.0  # friction stops the wheel, it cannot reverse it
        self.angle += 0.5 * (self.omega + new_omega) * dt
        self.omega = new_omega


@dataclass
class WheelController:
    """``MotorController`` (motor_ctl_kaia) for one wheel with a quadrature encoder."""

    pid: SampledPI
    ticks_per_rev: int = 4140
    max_rpm: float = 180.0
    pid_period: float = 0.03
    target_rpm: float = 0.0
    measured_rpm: float = 0.0
    duty: float = 0.0
    _setpoint_changed: bool = False
    _time_since_update: float = 0.0
    _ticks_prev: int = 0

    def set_target_rpm(self, rpm):
        if rpm == self.target_rpm:
            return
        self.target_rpm = max(-self.max_rpm, min(self.max_rpm, rpm))
        self._setpoint_changed = True

    def update(self, ticks: int, elapsed: float):
        """Call once per firmware loop pass; ``elapsed`` = time since last pass."""
        self._time_since_update += elapsed
        if self._time_since_update < self.pid_period and not self._setpoint_changed:
            return
        dt = self._time_since_update
        self._time_since_update = 0.0
        self.measured_rpm = (ticks - self._ticks_prev) / self.ticks_per_rev / dt * 60.0
        self._ticks_prev = ticks
        self._setpoint_changed = False
        if self.target_rpm == 0.0 and self.measured_rpm == 0.0:
            self.pid.reset(self.measured_rpm)  # no creep after a stop
        self.duty = self.pid.update(self.target_rpm, self.measured_rpm, dt)


@dataclass
class SpeedRamp:
    """The firmware's ``cmd_vel`` handling and ``updateSpeedRamp``."""

    wheel_dia: float = 0.043
    wheel_track: float = 0.105043
    max_rpm: float = 180.0
    accel_max: float = 2.0          # m/s^2, base.wheel.accel.max
    start: tuple = (0.0, 0.0)       # (left, right) RPM at ramp start
    target: tuple = (0.0, 0.0)
    duration: float = 0.0
    elapsed: float = 0.0
    _rpm_per_mps: float = field(init=False)

    def __post_init__(self):
        self._rpm_per_mps = 60.0 / (math.pi * self.wheel_dia)

    def command(self, linear_x, angular_z, current_targets):
        half = 0.5 * angular_z * self.wheel_track
        rpm = [(linear_x - half) * self._rpm_per_mps, (linear_x + half) * self._rpm_per_mps]
        scale = min([1.0] + [self.max_rpm / abs(r) for r in rpm if abs(r) > self.max_rpm])
        self.target = (rpm[0] * scale, rpm[1] * scale)
        self.start = tuple(current_targets)
        max_speed_diff = max(abs(t - s) for t, s in zip(self.target, self.start)) \
            / self._rpm_per_mps
        self.duration = max_speed_diff / self.accel_max
        self.elapsed = 0.0

    def update(self, elapsed):
        """Return the (left, right) target RPM for this loop pass."""
        self.elapsed += elapsed
        if self.elapsed >= self.duration:
            return self.target
        r = self.elapsed / self.duration
        return tuple(s + (t - s) * r for s, t in zip(self.start, self.target))
