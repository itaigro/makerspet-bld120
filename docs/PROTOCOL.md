# Real-robot measurement protocol (M3)

This is the procedure for collecting the data that M4 uses to identify the
robot (motor model, wheel geometry, odometry errors) and to check the
simulation against reality. Every experiment uses the same tools that were
rehearsed in simulation (`scripts/rehearse_experiments.sh`), so the
simulation's prediction exists for each one before the robot runs it.

**Time:** about 1.5 hours, including breaks to charge the battery.
**You need:** a kitchen scale, a tape measure, a ruler, masking tape, a pencil,
a box or mug to put the robot on, and about 1.2 m × 1.2 m of clear floor.

## 0. Safety: read this first

* **The firmware has no command timeout.** The robot keeps driving its last
  `cmd_vel` until a new one arrives. If the laptop, the WiFi or the micro-ROS
  agent fails while the robot moves, *it will not stop by itself*. Stay next
  to it, and stop it by lifting it or with its power switch.
* The experiment nodes stop the robot on Ctrl-C and when they finish, and a
  guard stops a run that drives further or longer than planned, or whose
  odometry stops arriving. None of that helps when the link is down.
* Start every new experiment type with `dry_run:=true`: it prints the plan
  (duration, distance, guard limits) without moving.

## 1. Before you start

1. Charge the battery fully. Motor speed depends on battery voltage, so the
   voltage is recorded in every run (`/battery_state` in the bag). A full pack
   reads about 8.4 V; recharge when it drops below about 7.4 V (2 × 3.7 V).
   Ignore the `percentage` field: upstream computes it for a 9.0 V full /
   6.8 V empty pack, so a full battery shows about 72 %.
2. Start the robot as in README Part 1 (micro-ROS agent and
   `kaiaai_bringup physical.launch.py robot_model:=makerspet_mini`).
3. **In every new terminal**, source this repo's overlay workspace first.
   Without it, `ros2 launch bld120_bringup ...` fails with
   `Package 'bld120_bringup' not found`. Then check the connection:

   ```bash
   source ~/ros2_workspaces/bld120_ws/install/setup.bash
   ros2 topic hz /joint_states                # 20-40 Hz
   ros2 param get /pet motor.driver.pid.period # 0.03
   ros2 param get /pet motor.left.pwm.now      # a number, 0.0 at rest
   ```

   If the `ros2 param` commands time out, the firmware's parameter server is
   not reachable. The experiments still run, but without PWM data
   (`firmware.csv` stays empty). Note it and carry on.
4. Each run goes into its own folder under `~/bld120_runs/`, named after the
   experiment and the time. At the end, send me the whole `~/bld120_runs`
   folder (or its path, if I can reach it).

## 2. Static measurements

Fill in [`templates/robot_measurements.yaml`](templates/robot_measurements.yaml)
(copy it to `~/bld120_runs/robot_measurements.yaml`):

| What | How |
|---|---|
| Mass | Kitchen scale, with the battery in. |
| Wheel diameter | Ruler across each tyre, through the axle. Only a rough check (±0.5 mm is ±1 %); the straight-line test measures it properly. |
| Track width | Ruler from the centre of the left tyre's contact patch to the centre of the right one. |
| Reference marks | Put a small piece of masking tape on the top edge of the robot at the **front** and at the **back**, on its centre line, and draw a line on each. These are the points you transfer to the floor in sections 5 and 6. |
| Mark spacing | Distance between the front and rear marks (`mark_spacing_cm`). |
| Axle position | Distance from the rear mark to the wheel axle, along the centre line (`axle_from_rear_cm`). Look at the robot from the side and measure from the rear mark to the centre of the wheel. |

## 3. Wheels-up staircase (motor curve without load)

The robot sits on a box or a mug with **both wheels in the air**. It steps
through 16 speeds (5 s each, 1.5 s stop between them, forward and backward),
about 1 min 45 s in total. The firmware reports the PWM duty needed for each
speed, which gives the motor's dead zone and gain with no floor friction.

```bash
ros2 launch bld120_bringup experiment.launch.py experiment:=staircase dry_run:=true
ros2 launch bld120_bringup experiment.launch.py experiment:=staircase
```

Do it **twice**. Make sure the wheels touch nothing. The odometry will
"drive" about 10 m while the robot stands still; that is expected, and the
guard's distance limit is off for this experiment.

## 4. Step tests on the floor (speed response, PID period)

You need about **1.5 m of straight, clear floor** in front of the robot. The
robot drives about 1 m forward in steps (0.1 m/s, 0.2 m/s, stop, creep at
0.02 m/s), then spins in place for 3 s. Total 20 s.

```bash
ros2 launch bld120_bringup experiment.launch.py experiment:=steps dry_run:=true
ros2 launch bld120_bringup experiment.launch.py experiment:=steps    # run this line 3 times
```

Each launch is **one run**: start it again for the next one, carrying the
robot back to the start in between. Then two other PID settings, 3 runs each. `ros2 param set` changes the firmware live. The values return to
normal when the robot reboots, but restore them anyway at the end:

```bash
# A: faster sampling, same controller (ki scaled with the period; see docs/WHEEL_CONTROLLER.md)
ros2 param set /pet motor.driver.pid.period 0.01
ros2 param set /pet motor.driver.pid.ki 0.001
ros2 launch bld120_bringup experiment.launch.py experiment:=steps    # run this line 3 times

# B: faster sampling with the stock per-period gains (3x the integral gain)
ros2 param set /pet motor.driver.pid.ki 0.003
ros2 launch bld120_bringup experiment.launch.py experiment:=steps    # run this line 3 times

# restore
ros2 param set /pet motor.driver.pid.period 0.03
ros2 param get /pet motor.driver.pid.ki      # 0.003
```

The run folder records the firmware settings each run actually used
(`firmware.json`), so the runs sort themselves out later.

## 5. Straight line (mean wheel diameter)

The robot drives 1.0 m straight, as measured by its own odometry. You measure
how far it really went. The ratio calibrates the mean wheel diameter, and the
sideways drift hints at the left/right difference.

1. Lay a strip of masking tape on the floor along the direction of travel,
   about 1.3 m long. Align it with the parquet seams if you can: they give you
   a straight line and right angles for free.
2. Put the robot on the tape, its front and rear marks over the tape's line.
   Draw a short pencil line on the floor tape under each mark, and label them
   **F0** and **R0**. Directions: **x** runs along the tape, forward;
   **y** runs to the left.
3. Run:

   ```bash
   ros2 launch bld120_bringup experiment.launch.py experiment:=line
   ```

4. When it stops, mark the floor under the front and rear marks again (small
   pieces of tape, pencil): **F1**, **R1**. Measure how far each mark moved
   from **its own** start mark: F1 from F0, R1 from R0.
   * **dx:** along the tape, forward positive.
   * **dy:** perpendicular to the tape line, + left, − right.

   Write them into
   `~/bld120_runs/floor_marks.csv` (template:
   [`templates/floor_marks.csv`](templates/floor_marks.csv)), with the run
   folder's name.
5. Repeat for **3 runs**.

## 6. UMBmark square (odometry errors)

The robot drives a 1 m square on its own odometry and should end where it
started. It does not, and the end error in the clockwise and
counter-clockwise directions separates the two systematic errors: unequal
wheel diameters, and the wrong track width (Borenstein & Feng 1996).

1. Use the same start-frame method as the straight line, with the start in
   one corner of the 1.2 m × 1.2 m area:
   * **CCW** squares turn left, so they need the free area ahead and to the left.
   * **CW** squares turn right, so they need it ahead and to the right.
2. Run **5 CCW and 5 CW**, alternating or in blocks, whichever is easier:

   ```bash
   ros2 launch bld120_bringup experiment.launch.py experiment:=square direction:=ccw
   ros2 launch bld120_bringup experiment.launch.py experiment:=square direction:=cw
   ```

   Each square takes about 1 min (4 × 1 m at 0.1 m/s plus 4 turns).
3. After each run, mark and measure F1 and R1 as in the straight-line test
   (each from its own start mark). The robot ends near the start, so dx is
   small and can be negative (behind the start mark).
   Put the robot back exactly on F0/R0 for the next run.

The robot ends within a few centimetres of the start, so only short
measurements are needed. A few millimetres of measuring error is fine:
the effects we are measuring are centimetres.

## 7. Notes to keep

In `floor_marks.csv`, and in the `notes:` line of a run's `run.yaml` if
something was unusual: floor type (PVC or wood parquet), battery voltage if
you looked, anything that went wrong (wheel slipped on a seam, a cable caught
on something, run stopped by the guard). Failed runs are fine. Keep their
folders and write down what happened.

## What M4 does with this

| Data | Result |
|---|---|
| Staircase: PWM duty vs speed | Motor gain and dead zone (no load) |
| Step tests: wheel speed at 20-40 Hz | Time constant with the robot's inertia, floor dead zone; sim vs real overshoot |
| Step tests at 3 PID settings | Whether faster sampling helps on the real robot as the analysis predicts |
| Straight line | Mean wheel diameter |
| UMBmark CW/CCW | Wheel diameter ratio and effective track width (UMBmark correction factors) |
| Mass | Sanity check of the identified time constant |

With those values, the simulation gets the robot's own motor and odometry
errors, and M4 compares the calibrated simulation against these same runs.
