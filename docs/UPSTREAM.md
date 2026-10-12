# Upstream reference

Notes on the Kaia.ai / Maker's Pet packages this repo depends on: what is
pinned, how the BLD-120MM maps onto them, and practical notes. Written in my own
words; the authoritative source is always the upstream repos and the
[build guide](https://makerspet.com/blog/bld-120mm-pack/).

## Pinned versions

All upstream ROS 2 packages are on their `jazzy` branches, pinned in
[`../upstream.repos`](../upstream.repos):

| Package | Repo | Commit |
|---------|------|--------|
| kaiaai | kaiaai/kaiaai | `ef5fb2a` |
| kaiaai_bringup | kaiaai/kaiaai_bringup | `321c1b2` |
| kaiaai_gazebo | kaiaai/kaiaai_gazebo | `a6812df` |
| kaiaai_msgs | kaiaai/kaiaai_msgs | `e17a601` |
| kaiaai_telemetry | kaiaai/kaiaai_telemetry | `7ea0d66` |
| kaiaai_teleop | kaiaai/kaiaai_teleop | `0158918` |
| makerspet | makerspet/makerspet | `e15deb0` |
| makerspet_mini | makerspet/makerspet_mini | `77d196b` |
| m-explore-ros2 | kaiaai/m-explore-ros2 | `9279196` |
| makerspet_loki | makerspet/makerspet_loki | `e778ecd` |
| makerspet_snoopy | makerspet/makerspet_snoopy | `412b9d7` |

`makerspet_loki` and `makerspet_snoopy` are other robot models; they are pinned
only because `kaiaai_bringup` declares them as runtime dependencies.

Firmware: [kaiaai/firmware](https://github.com/kaiaai/firmware) release
**`v0.8.4`**, board config `config_mini_bdc_30p.yaml`, built with the Arduino IDE.
Note that the v0.8.4 source still reports `FW_VERSION = "0.8.3-iron"` on the
serial console; v0.8.4 only fixed the Ubuntu build and did not bump the string.
The newer `v0.8.6` (pre-release, tested with Jazzy) leaves the motor/PID code
unchanged; it adds LD19 LiDAR support and an optional distance/yaw stop limit
carried in `cmd_vel.linear.z` / `cmd_vel.angular.x` (zero = disabled, which is
what Nav2 and teleop send). As of v0.8.6 that limit check
(`isOdomTargetReached`) appears inverted: it returns true while the travelled
distance is still *below* the target, i.e. immediately after the command. Do not
rely on it; it is irrelevant here because this repo always sends zeros.

## BLD-120MM ↔ `makerspet_mini`

The kit is not listed under its own name upstream. It is the `makerspet_mini`
robot model, which supplies the URDF, the Gazebo plugins and bridge config, and
the Nav2 / Cartographer / teleop configuration used by `kaiaai_bringup`.

Key geometry from `makerspet_mini/urdf/params.xacro`:

| Parameter | Value |
|-----------|-------|
| Base diameter | 0.124 m |
| Wheel diameter | 0.043 m |
| Wheel separation | 0.105 m |
| Nav2 `robot_radius` | 0.0625 m |
| Nav2 `inflation_radius` | 0.0725 m |

## How upstream's simulation works (and what this repo changes)

- `kaiaai_gazebo/launch/world.launch.py` spawns the robot from its URDF into a
  Gazebo Harmonic world and starts a `ros_gz_bridge` from
  `makerspet_mini/config/gz_bridge.yaml`.
- Motion: `gz-sim-diff-drive-system` turns `cmd_vel` directly into wheel
  velocities (only an acceleration limit).
- Odometry: `gz-sim-odometry-publisher-system` publishes the **true** model pose
  as `/odom` and the `odom → base_footprint` transform, so simulated odometry
  never drifts.
- LiDAR: `gpu_lidar`, 360 samples at 5 Hz, 0.1–10 m, 1 cm Gaussian noise.

Phase 1 of this repo replaces the odometry and actuation parts with realistic
models; the world, LiDAR and upstream launch files are reused as-is.

## Practical notes

- Wi-Fi must be 2.4 GHz, without client isolation, for the robot to reach the
  micro-ROS agent.
- The upstream `kaiaai_gazebo` README still describes the older Humble/Docker
  flow; the `jazzy` branch itself already targets Gazebo Harmonic via `ros_gz`.
- Keep teleop slow while mapping: fast turns smear the LiDAR scans.
- `map_saver_cli -f <dir>/<name>` needs a writable `<dir>`. A folder created
  earlier with `sudo` (or from inside a Docker container) is owned by root, and
  the save quietly lands elsewhere.
- Keep the ROS install consistent: upgrade **all** `ros-jazzy-*` packages
  together. A partial upgrade (new Nav2, old `diagnostic_updater`) makes the
  Nav2 container die with exit code 127 (undefined symbol) and RViz reports
  "no map received". Check with
  `ldd -r /opt/ros/jazzy/lib/libnav2_lifecycle_manager_core.so | grep undefined`.
- After upgrading ROS packages, rebuild C++ workspaces with
  `colcon build --symlink-install --cmake-clean-cache`; stale CMake caches point
  at replaced library versions (e.g. `No rule to make target libfastcdr.so.X`).
- Stay on Ubuntu 24.04: ROS 2 Jazzy has no binaries for 26.04.
