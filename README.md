# makerspet-bld120

Simulation realism, benchmarked autonomy and a patrol application for the
**Maker's Pet BLD-120MM** LiDAR robot, built on top of the open-source
[Kaia.ai](https://github.com/kaiaai) / [Maker's Pet](https://github.com/makerspet)
ROS 2 platform.

> **Credit.** The robot hardware, firmware and the base ROS 2 stack (bringup,
> telemetry, teleop, Gazebo world, Nav2 configuration) are the work of the
> Kaia.ai / Maker's Pet authors. This repository does **not** modify or vendor
> any of it; it adds new packages that run alongside the upstream ones. The
> build itself is documented in the official
> [BLD-120MM build guide](https://makerspet.com/blog/bld-120mm-pack/).
> See [NOTICE](NOTICE) for attribution details.

**Status:** work in progress. See the [roadmap](#roadmap).

---

## The robot

The BLD-120MM kit is a ~12 cm differential-drive robot. In the Kaia.ai software
it is described by the **`makerspet_mini`** robot-description package, which is
why every upstream command below uses `robot_model:=makerspet_mini`.

| Component  | Part |
|------------|------|
| MCU        | ESP32 DevKit v1 on a BDC-30P driver board, Kaia.ai firmware (Arduino) |
| LiDAR      | LDROBOT LD14P (360°, 2D) |
| Drive      | 2 × 12 V N20 gear motors with quadrature encoders (1035 PPR) |
| Power      | 2 × 18650 Li-ion (upstream default is 6 × AA; the 18650 pack is a mod the build guide flags as at-your-own-risk) |
| Link to PC | Wi-Fi (2.4 GHz) → micro-ROS agent on the ROS 2 PC |
| IMU        | none: odometry comes from the wheel encoders only |

---

## Part 1: Upstream baseline

This is the setup the rest of the repo builds on. It is a short summary in my
own words of what I ran; the authoritative, illustrated instructions
(assembly, firmware flashing, Wi-Fi provisioning) are in the
[build guide](https://makerspet.com/blog/bld-120mm-pack/) and the
[`kaiaai`](https://github.com/kaiaai) repositories.

**Platform:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, native install (no Docker).

1. **Build the robot and flash the firmware.** Follow the build guide. The
   firmware is the Kaia.ai ESP32 firmware (tested here with release `v0.8.4`,
   board config `config_mini_bdc_30p.yaml`), flashed from the Arduino IDE.
   Robot and network settings are entered through the robot's Wi-Fi setup page.
2. **Build the upstream ROS 2 packages** into an underlay workspace. The exact
   commits this repo is tested against are pinned in [`upstream.repos`](upstream.repos):
   ```bash
   mkdir -p ~/ros_ws/src && cd ~/ros_ws
   vcs import src < /path/to/makerspet-bld120/upstream.repos
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths src --ignore-src -y
   colcon build --symlink-install
   source install/setup.bash
   ```
3. **Connect the robot and drive it manually** (one terminal each):
   ```bash
   ros2 launch kaiaai_bringup physical.launch.py robot_model:=makerspet_mini
   ros2 launch kaiaai_bringup monitor_robot.launch.py robot_model:=makerspet_mini
   ros2 run kaiaai_teleop teleop_keyboard robot_model:=makerspet_mini
   ```
4. **Map, then navigate** (upstream autonomy, unchanged):
   ```bash
   ros2 launch kaiaai_bringup cartographer.launch.py robot_model:=makerspet_mini
   ros2 run nav2_map_server map_saver_cli -f ~/maps/home
   ros2 launch kaiaai_bringup navigation.launch.py robot_model:=makerspet_mini map:=$HOME/maps/home.yaml
   ```
5. **Simulation** (upstream Gazebo world; see Part 2 for this repo's version):
   ```bash
   ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_mini
   ```

Steps 3 and 4 were verified end to end on my robot (manual driving, a
Cartographer map of my apartment, Nav2 goals reached) on 2026-10-12. Practical
notes I ran into are collected in [docs/UPSTREAM.md](docs/UPSTREAM.md).

---

## Part 2: What this repo adds

| Area | Upstream | This repo |
|------|----------|-----------|
| Sim odometry | Ground-truth pose published as odometry: no drift | Odometry integrated from simulated 1035-PPR encoders with calibration error and slip |
| Sim actuation | Ideal velocity drive with an acceleration cap | "Virtual firmware": N20 motor model + the ESP32 wheel PID loop |
| Sim parameters | Estimated masses and nominal geometry | Fitted to logged data from the real robot (system identification) |
| Navigation | Default Nav2 (NavFn + DWB), untuned for this size | Benchmarked controllers/planners; tuned config backed by data |
| Controller | Nav2 built-ins | Custom Lyapunov-based path-tracking controller plugin |
| Application | Manual goals in RViz | Waypoint patrol with retries and return-to-home |

Each row is filled in by a milestone below; results and plots are added as they land.

### Simulated odometry that drifts like the real one (`bld120_sim`, M1)

The real robot computes odometry on the ESP32 from its wheel encoders and the
PC only republishes it. Upstream's simulation instead publishes the robot's
true pose as `/odom`, so in simulation odometry never drifts and anything
tested there (mapping, localization, navigation) looks better than it will on
the robot.

`bld120_sim` keeps upstream's robot model and worlds, but at launch it
reroutes Gazebo's true pose to `/ground_truth/odom` and produces `/odom` the
way the firmware does:

| Firmware (v0.8.4, `calcOdometry`) | `encoder_odometry` node |
|---|---|
| Quadrature encoders, 1035 PPR × 4 edges | True wheel angle quantized to 4140 ticks/rev |
| Telemetry every 50 ms | 20 Hz integration on simulation time |
| Midpoint (2nd-order) Euler step, `float32` | Same step, ported line by line, in `float32` |
| Wheel diameter / track from `config.yaml` | Same values, plus optional calibration errors and random slip |

```bash
ros2 launch bld120_sim sim.launch.py                       # GUI, upstream living room
ros2 launch bld120_sim sim.launch.py err_left_pct:=1.0     # firmware thinks the left wheel is 1 % larger
ros2 launch bld120_sim square_test.launch.py direction:=cw output:=~/runs/cw.csv
ros2 run bld120_sim analyze_square ~/runs/*.csv --out ~/runs
scripts/odometry_experiment.sh                             # the full experiment below
```

**Experiment.** A 1 m square driven in both directions, steering only on
`/odom` (leg length, heading and turns), as in the UMBmark test (Borenstein &
Feng, 1996). If odometry were perfect, the robot would return to its start.
Flat world, 0.1 m/s, 0.5 rad/s; the upstream simulation scores exactly 0 on
every row by construction.

| Error source | Direction | True return error | Odometry yaw error at end |
|---|---|---|---|
| None (quantization + simulated physics only) | CCW / CW | 0.1 / 0.5 cm | +1.0° / −1.0° |
| Firmware left wheel diameter +1 % | CCW / CW | 22.0 / 30.3 cm | −19.2° / −24.8° |
| Firmware wheel track +3 % | CCW / CW | 12.7 / 13.0 cm | −9.9° / +9.9° |
| Random slip, 2 % per step (seed 7) | CCW / CW | 6.9 / 7.6 cm | −4.5° / −6.6° |

The two systematic errors leave the classic UMBmark signatures, which is what
lets them be told apart (and later, in M4, measured on the real robot):

* **Unequal wheel diameters** curve every straight leg the same way, so the
  yaw error has the *same sign* in both directions.
* **Wrong wheel track** mis-scales every turn (3 % of 360° ≈ 10.8°, measured
  9.9°), so the yaw error *flips sign* with direction.
* Even with no injected error, the simulated wheels skid slightly during
  in-place turns, an effective-track error of about ±1° per square.

| Wheel diameter error | Track error |
|---|---|
| ![](docs/results/m1_odometry/wheel_dia_left_+1pct/square_paths.png) | ![](docs/results/m1_odometry/wheel_track_+3pct/square_paths.png) |

All plots and per-run numbers: [`docs/results/m1_odometry/`](docs/results/m1_odometry/).

---

## Roadmap

| Phase | Milestone | Status |
|-------|-----------|--------|
| 0 | M0: Repository skeleton, pinned upstream, workspace | ✅ |
| 1 | M1: Encoder-based odometry in simulation | ✅ |
| 1 | M2: Virtual firmware (motor model + wheel PID) | ⏳ |
| 1 | M3: Real-robot experiment tooling and protocol | ⏳ |
| 1 | M4: System identification, calibrated sim | ⏳ |
| 2 | M5: Navigation benchmark harness | ⏳ |
| 2 | M6: Controller/planner comparison, tuned config | ⏳ |
| 2 | M7: Custom Lyapunov path-tracking controller | ⏳ |
| 3 | M8: Patrol application (simulation) | ⏳ |
| 3 | M9: Patrol on the real robot | ⏳ |

---

## Building this repo

The packages here are built in an overlay workspace on top of the upstream one:

```bash
cd /path/to/makerspet-bld120
scripts/setup_workspace.sh --build      # creates ~/ros2_workspaces/bld120_ws, symlinks this repo, builds
source ~/ros2_workspaces/bld120_ws/install/setup.bash
```

`UPSTREAM_WS` and `BLD120_WS` override the default workspace locations.

## License

Apache-2.0, same as upstream. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
