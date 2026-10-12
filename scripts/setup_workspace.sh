#!/usr/bin/env bash
# Create (or refresh) the colcon workspace that builds this repo.
#
# Layout:
#   $UPSTREAM_WS  existing workspace with the Kaia.ai / Maker's Pet packages
#                 (see upstream.repos), built and sourced as an underlay
#   $BLD120_WS    overlay workspace; its src/makerspet-bld120 symlinks to this repo
#
# Usage: scripts/setup_workspace.sh [--build]
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM_WS="${UPSTREAM_WS:-$HOME/ros_ws}"
BLD120_WS="${BLD120_WS:-$HOME/ros2_workspaces/bld120_ws}"

if [[ ! -f "$UPSTREAM_WS/install/setup.bash" ]]; then
  echo "Upstream workspace not built: $UPSTREAM_WS/install/setup.bash missing." >&2
  echo "Import and build it first:  vcs import $UPSTREAM_WS/src < $REPO_DIR/upstream.repos" >&2
  exit 1
fi

mkdir -p "$BLD120_WS/src"
ln -sfn "$REPO_DIR" "$BLD120_WS/src/makerspet-bld120"
echo "Workspace: $BLD120_WS  (src/makerspet-bld120 -> $REPO_DIR)"

if [[ "${1:-}" == "--build" ]]; then
  # ROS setup scripts reference unset variables, so relax -u while sourcing.
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$UPSTREAM_WS/install/setup.bash"
  set -u
  cd "$BLD120_WS"
  colcon build --symlink-install
  echo "Built. In every new shell:  source $BLD120_WS/install/setup.bash"
fi
