"""Encoder and odometry models of the BLD-120MM firmware, free of ROS.

The real robot computes odometry on the ESP32 (Kaia.ai firmware v0.8.4,
``calcOdometry`` in ``kaiaai-esp32.ino``) and the PC only republishes it. To make
simulated odometry behave like the real one, this module reproduces that
pipeline:

1. ``EncoderModel``: true wheel angle -> integer quadrature ticks
   (1035 PPR x 4 edges = 4140 ticks/rev), with optional random slip.
2. ``FirmwareOdometry``: tick deltas -> pose, integrated once per telemetry
   period (50 ms) with the firmware's midpoint rule, in float32, using the
   wheel geometry the firmware *believes* (which may differ from the true one).

Sign convention: positive wheel angle = that wheel rolling forward. In the
firmware, ``publishTelem`` passes (left, right) deltas into ``calcOdometry``,
whose parameters are named (right, left), and the mirrored *left* motor reads
negative when rolling forward; the two swaps cancel, so the firmware's net
result is d_yaw = (right - left) / track, counter-clockwise positive. The
simulated joints are both forward-positive already, so no negation is needed.
"""
from dataclasses import dataclass
import math

import numpy as np

F32 = np.float32
F32_PI = F32(math.pi)
F32_TWO_PI = F32(2.0 * math.pi)


@dataclass
class EncoderModel:
    """Quantizes a wheel's true rotation into encoder ticks.

    ``slip_std`` adds zero-mean multiplicative noise to each angle increment,
    modelling random wheel slip and floor irregularities: an increment of
    ``d`` rad is measured as ``d * (1 + N(0, slip_std))``.
    """

    ticks_per_rev: int = 4140
    slip_std: float = 0.0
    rng: np.random.Generator | None = None

    def __post_init__(self):
        self._last_true_angle = None
        self._measured_angle = 0.0  # continuous angle the encoder "saw"
        if self.rng is None:
            self.rng = np.random.default_rng()

    def update(self, true_angle: float) -> int:
        """Feed the current true joint angle; return the cumulative tick count."""
        if self._last_true_angle is None:
            self._last_true_angle = true_angle
        delta = true_angle - self._last_true_angle
        self._last_true_angle = true_angle
        if self.slip_std > 0.0 and delta != 0.0:
            delta *= 1.0 + self.rng.normal(0.0, self.slip_std)
        self._measured_angle += delta
        return math.floor(self._measured_angle * self.ticks_per_rev / (2.0 * math.pi))


def _wrap_pi_f32(angle):
    """Single wrap into [-pi, pi], exactly as the firmware does it."""
    if angle > F32_PI:
        angle = F32(angle - F32_TWO_PI)
    elif angle < -F32_PI:
        angle = F32(angle + F32_TWO_PI)
    return angle


class FirmwareOdometry:
    """Port of the firmware's ``calcOdometry``: midpoint Euler step in float32."""

    def __init__(self, wheel_radius_left: float, wheel_radius_right: float,
                 wheel_track: float, ticks_per_rev: int = 4140):
        self.rad_per_tick = F32(2.0 * math.pi / ticks_per_rev)
        self.radius_left = F32(wheel_radius_left)
        self.radius_right = F32(wheel_radius_right)
        self.track_recip = F32(1.0 / wheel_track)
        self.x = F32(0.0)
        self.y = F32(0.0)
        self.yaw = F32(0.0)
        self.vel_x = F32(0.0)
        self.vel_yaw = F32(0.0)

    def step(self, d_ticks_left: int, d_ticks_right: int, dt: float):
        """Integrate one telemetry period of tick deltas over ``dt`` seconds."""
        if dt <= 0.0:
            return
        dist_left = F32(d_ticks_left * self.rad_per_tick * self.radius_left)
        dist_right = F32(d_ticks_right * self.rad_per_tick * self.radius_right)

        avg_dist = F32((dist_right + dist_left) * F32(0.5))
        # Net firmware result in forward-positive terms (see module docstring).
        d_yaw = F32((dist_right - dist_left) * self.track_recip)

        avg_angle = _wrap_pi_f32(F32(d_yaw * F32(0.5) + self.yaw))
        self.x = F32(self.x + F32(np.cos(avg_angle)) * avg_dist)
        self.y = F32(self.y + F32(np.sin(avg_angle)) * avg_dist)
        self.yaw = _wrap_pi_f32(F32(self.yaw + d_yaw))

        self.vel_x = F32(avg_dist / F32(dt))
        self.vel_yaw = F32(d_yaw / F32(dt))

    @property
    def pose(self):
        return float(self.x), float(self.y), float(self.yaw)
