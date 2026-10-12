# The wheel speed controller, as equations

The virtual firmware (M2) has to behave like the ESP32's wheel speed loop. This
page specifies that loop mathematically; `SampledPI` in
[`bld120_sim/firmware_model.py`](../bld120_sim/bld120_sim/firmware_model.py) is
written from this page.

The behaviour was established by reading the firmware and observing what it
computes; no code from the firmware's PID library (`PID_Timed`, GPLv3) is used
in this repository. The control law itself is the standard discrete PI(D)
controller found in any control textbook.

## Signals

| Symbol | Meaning | Unit |
|---|---|---|
| `r_k` | target wheel speed at sample `k` (after the speed ramp) | RPM |
| `y_k` | wheel speed measured from encoder ticks over the last sample | RPM |
| `h_k` | actual time since the previous sample | s |
| `T` | reference period, `motor.driver.pid.period` = 0.03 s | s |
| `u_k` | PWM duty sent to the motor driver, sign = direction | – |

## The law

The firmware does not sample at a fixed rate. The loop runs every `T` normally,
but also on every main-loop pass (~2 ms) while the ramp is changing the target.
So the gains are defined *per reference period*. A sample of length `h_k`
counts as `ρ_k = h_k / T` reference periods:

```
ρ_k = h_k / T
e_k = r_k − y_k                                          tracking error

I_k = sat( I_{k−1} + K_i · ρ_k · e_k )                   integral, in duty units
D_k = −K_d · (y_k − y_{k−1}) / ρ_k                       derivative of the measurement
u_k = sat( K_p · e_k + I_k + D_k )

sat(x) = min(max(x, −1), 1)
```

Gains on this robot (BDC-30P configuration): `K_p = 0.001`, `K_i = 0.003`,
`K_d = 0`. In practice it is a **PI controller**.

### Why each piece is there

* **Scaling by `ρ_k`.** Gains per reference period make the integral grow at the
  same rate *per second* however often the loop runs. The continuous-time
  integral gain is `K_i / T = 0.1` duty per (RPM·s). Without the scaling, the
  fast samples during a ramp would wind the integral up about 15 times faster.
* **Clamping the integral to the output range (anti-windup).** While the motor
  is saturated (`|u| = 1`), an unclamped integral keeps growing and later causes
  a long overshoot. Clamping `I` to ±1 means it never asks for more than the
  driver can deliver.
* **Derivative of the measurement, not of the error.** When the target jumps,
  the derivative of the error is a spike ("derivative kick"); the derivative of
  the measurement is not. It is unused here (`K_d = 0`) but kept for
  completeness. On the very first sample there is no previous measurement, so
  `D` is taken as 0.
* **The integral carries the steady-state duty.** At steady state `e = 0`, so
  `u = I`. The integral settles at whatever duty holds the wheel at speed
  (≈ 0.30 at 0.1 m/s with the nominal motor, see M2). The proportional term
  only acts on the transient. With such a small `K_p`, the response is mostly
  shaped by `K_i`, which is why the loop overshoots by ~20 %. A PI loop on a
  first-order motor is a second-order system, and this pair of gains makes it
  underdamped.

### Reset at standstill

The motor controller (`motor_ctl_kaia`, Apache-2.0, not the PID library) resets
the loop when the target is zero *and* the wheel is measured stopped:

```
I_k = 0,   y_{k−1} = y_k
```

Otherwise a small leftover integral keeps a PWM duty on a stopped robot. Below
the dead zone it does not move the wheel, but it would make it creep away the
moment the friction is overcome.

### Arithmetic

The ESP32 computes in single precision (`float`). `SampledPI` does the same
(`numpy.float32`), so that rounding matches the robot.

## How the analogue looks in continuous time

For intuition, with `K_d = 0` and ignoring saturation, the law is a sampled
version of

```
u(t) = K_p · e(t) + (K_i / T) · ∫ e dt
```

Closing it around the first-order motor model `τ ω̇ = k u − ω` gives the
characteristic polynomial

```
τ s² + (1 + k K_p') s + k K_i' = 0      with K_p' = K_p, K_i' = K_i / T
```

(gains converted to the motor's speed units). The damping ratio is

```
ζ = (1 + k K_p') / (2 √(τ k K_i'))
```

With the nominal motor (`k` = 23.3 rad/s per unit duty, `τ` = 50 ms) this gives
`ζ ≈ 0.58`. The step response of this continuous, linear loop overshoots by
about 11 %.

The M2 simulation overshoots more (18–22 %). The difference comes from what the
linear model leaves out:

| Model of the 0.1 → 0.2 m/s step (`firmware_model.py`, no Gazebo) | Overshoot |
|---|---|
| Full virtual firmware: PI every 30 ms, dead zone 0.10, 2 m/s² ramp | 18.6 % |
| Without the dead zone | 17.1 % |
| PI every 2 ms instead of 30 ms | 10.9 % |
| PI every 2 ms, no dead zone | 9.7 % |
| PI every 2 ms, no dead zone, no ramp | 10.1 % |

**The 30 ms sample period causes most of the difference.** It is more than
half the motor's time constant (50 ms). The measured speed is an average over
the last sample, so the controller acts on information ~15–30 ms old, and that
delay erodes the damping. With fast sampling the model returns to the ~10 %
the continuous analysis predicts. The dead zone adds ~1.5 points: the integral
also has to carry the friction offset. On the real robot this is an argument
for a shorter `pid.period`, which M4 can test once the motor is identified.

M4 will repeat this comparison with the motor values identified on the real
robot.
