#!/usr/bin/env bash
# Rehearse the real-robot protocol (docs/PROTOCOL.md) against the simulation:
# the same experiment launch, the same `ros2 param` commands, the same files.
# It checks the tooling before it touches the robot, and its results are the
# simulation's predictions for each experiment (nominal motor model).
#
# Usage: scripts/rehearse_experiments.sh [OUT_DIR]
#   OUT_DIR  default: ~/bld120_runs/sim_rehearsal
set -eo pipefail
set -m   # job control: background jobs keep SIGINT, so the sim can be stopped cleanly

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${1:-$HOME/bld120_runs/sim_rehearsal}"
BLD120_WS="${BLD120_WS:-$HOME/ros2_workspaces/bld120_ws}"
source /opt/ros/jazzy/setup.bash
source "$BLD120_WS/install/setup.bash"
export ROS_DOMAIN_ID=87 GZ_PARTITION="bld120_rehearsal_$$"
mkdir -p "$OUT_DIR"

ros2 launch bld120_sim sim.launch.py world:=flat.sdf headless:=true x_pose:=0.0 y_pose:=0.0 \
  > "$OUT_DIR/sim.log" 2>&1 &
SIM=$!
stop_sim() {
  kill -INT "$SIM" 2>/dev/null || true
  wait "$SIM" 2>/dev/null || true
  if pgrep -f '[b]ld120_sim/worlds/flat.sdf' > /dev/null; then
    echo "WARN   Gazebo still running after the rehearsal:" >&2
    pgrep -af '[b]ld120_sim/worlds/flat.sdf' >&2
  fi
}
trap stop_sim EXIT
sleep 12

FW=/virtual_firmware
run() {  # label, experiment arguments...
  local label="$1"; shift
  echo "run    $label"
  timeout 300 ros2 launch bld120_bringup experiment.launch.py use_sim_time:=true \
    firmware_node:=$FW output_dir:="$OUT_DIR" "$@" > "$OUT_DIR/$label.log" 2>&1
  grep -o 'run folder: .*' "$OUT_DIR/$label.log" | sed 's/^/       /' || true
  grep -hE 'stopping:|finished;' "$OUT_DIR/$label.log" | sed 's/^.*\]: /       /' || true
}
pid() {  # period ki
  ros2 param set $FW motor.driver.pid.period "$1" > /dev/null
  ros2 param set $FW motor.driver.pid.ki "$2" > /dev/null
  echo "pid    period $1 s, ki $2"
}

run dry_run experiment:=steps dry_run:=true
run steps_stock experiment:=steps
pid 0.01 0.001
run steps_fast_sampling experiment:=steps
pid 0.01 0.003
run steps_fast_stock_gains experiment:=steps
pid 0.03 0.003
run staircase experiment:=staircase
run line experiment:=line
run square_ccw experiment:=square direction:=ccw
run square_cw experiment:=square direction:=cw

steps_csv() { grep -o 'run folder: .*' "$OUT_DIR/$1.log" | cut -d' ' -f3- | sed 's|$|/steps.csv|'; }
ros2 run bld120_sim analyze_steps \
  "$(steps_csv steps_stock)" "$(steps_csv steps_fast_sampling)" "$(steps_csv steps_fast_stock_gains)" \
  --labels "period 30 ms, ki 0.003 (stock)" "period 10 ms, ki 0.001" "period 10 ms, ki 0.003" \
  --out "$OUT_DIR/pid_period"
