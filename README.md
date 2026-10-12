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
| Power      | 6 × AA |
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
   firmware is the Kaia.ai ESP32 firmware (tested here with `0.8.3`), flashed
   from the Arduino IDE. Robot and network settings are entered through the
   robot's Wi-Fi setup page.
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
5. **Simulation** (upstream Gazebo world):
   ```bash
   ros2 launch kaiaai_gazebo world.launch.py robot_model:=makerspet_mini
   ```

Practical notes I ran into are collected in [docs/UPSTREAM.md](docs/UPSTREAM.md).

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

---

## Roadmap

| Phase | Milestone | Status |
|-------|-----------|--------|
| 0 | M0: Repository skeleton, pinned upstream, workspace | ✅ |
| 1 | M1: Encoder-based odometry in simulation | ⏳ |
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
