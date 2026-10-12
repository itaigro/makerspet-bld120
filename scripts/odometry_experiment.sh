#!/usr/bin/env bash
# M1 experiment: simulated square tests (CW and CCW) under different odometry
# error sources, then plots and a summary. Uses upstream's ideal drive
# (motor_model:=false), as the published M1 results do.
#
# Runs several headless simulations in parallel, each isolated with its own
# ROS_DOMAIN_ID and GZ_PARTITION so they cannot see each other.
#
# Usage: scripts/odometry_experiment.sh [OUT_DIR] [PARALLEL]
#   OUT_DIR   default: docs/results/m1_odometry
#   PARALLEL  simultaneous simulations, default 4
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${1:-$REPO_DIR/docs/results/m1_odometry}"
PARALLEL="${2:-4}"
BLD120_WS="${BLD120_WS:-$HOME/ros2_workspaces/bld120_ws}"

set +u
source /opt/ros/jazzy/setup.bash
source "$BLD120_WS/install/setup.bash"
set -u

mkdir -p "$OUT_DIR/csv" "$OUT_DIR/logs"

# name | extra launch arguments
CASES=(
  "ideal|"
  "wheel_dia_left_+1pct|err_left_pct:=1.0"
  "wheel_track_+3pct|err_track_pct:=3.0"
  "slip_2pct|slip_std:=0.02 seed:=7"
)

run_one() {  # name args direction domain
  local name="$1" args="$2" dir="$3" domain="$4"
  local tag="${name}_${dir}"
  echo "start  $tag"
  # Partition names are unique per invocation, so a leftover simulation from an
  # earlier run can never join this one.
  ROS_DOMAIN_ID="$domain" GZ_PARTITION="bld120_$$_$domain" \
    timeout 300 ros2 launch bld120_sim square_test.launch.py \
      motor_model:=false direction:="$dir" output:="$OUT_DIR/csv/$tag.csv" $args \
      > "$OUT_DIR/logs/$tag.log" 2>&1 || echo "WARN   $tag exited with $?"
  echo "done   $tag"
}

domain=60
for case in "${CASES[@]}"; do
  name="${case%%|*}"; args="${case#*|}"
  for dir in ccw cw; do
    while (( $(jobs -rp | wc -l) >= PARALLEL )); do wait -n; done
    run_one "$name" "$args" "$dir" "$domain" &
    domain=$((domain + 1))
  done
done
wait

if pgrep -f '[b]ld120_sim/worlds/flat.sdf' > /dev/null; then
  echo "WARN   Gazebo processes still running after the runs:" >&2
  pgrep -af '[b]ld120_sim/worlds/flat.sdf' >&2
fi

for case in "${CASES[@]}"; do
  name="${case%%|*}"
  ros2 run bld120_sim analyze_square \
    "$OUT_DIR/csv/${name}_ccw.csv" "$OUT_DIR/csv/${name}_cw.csv" \
    --out "$OUT_DIR/$name" --title "Square test: $name"
done
