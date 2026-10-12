#!/usr/bin/env bash
# M2 experiment: the step-test profile with upstream's ideal drive vs. the
# virtual firmware (ramp + wheel PID + DC motor model), then plots and summary.
#
# Usage: scripts/drive_experiment.sh [OUT_DIR]
#   OUT_DIR  default: docs/results/m2_drive
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${1:-$REPO_DIR/docs/results/m2_drive}"
BLD120_WS="${BLD120_WS:-$HOME/ros2_workspaces/bld120_ws}"

set +u
source /opt/ros/jazzy/setup.bash
source "$BLD120_WS/install/setup.bash"
set -u

mkdir -p "$OUT_DIR/csv" "$OUT_DIR/logs"

run_one() {  # name motor_model domain
  local name="$1" motor="$2" domain="$3"
  echo "start  $name"
  ROS_DOMAIN_ID="$domain" GZ_PARTITION="bld120_$$_$domain" \
    timeout 300 ros2 launch bld120_sim step_test.launch.py \
      motor_model:="$motor" output:="$OUT_DIR/csv/$name.csv" \
      > "$OUT_DIR/logs/$name.log" 2>&1 || echo "WARN   $name exited with $?"
  echo "done   $name"
}

run_one ideal_drive false 80 &
run_one virtual_firmware true 81 &
wait

if pgrep -f '[b]ld120_sim/worlds/flat.sdf' > /dev/null; then
  echo "WARN   Gazebo processes still running after the runs:" >&2
  pgrep -af '[b]ld120_sim/worlds/flat.sdf' >&2
fi

ros2 run bld120_sim analyze_steps \
  "$OUT_DIR/csv/ideal_drive.csv" "$OUT_DIR/csv/virtual_firmware.csv" \
  --labels "upstream ideal drive" "virtual firmware (nominal motor)" --out "$OUT_DIR"
