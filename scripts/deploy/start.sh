#!/usr/bin/env bash
set -eo pipefail        # no -u here yet

# Paths
REPO_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
WS=~/cl2_ws

# --- source ROS/env (temporarily allow unset vars) ---
set +u
source /opt/ros/humble/setup.bash
set -u 2>/dev/null || true

cd "$WS"
colcon build --symlink-install

exec "$REPO_DIR/scripts/deploy/app.sh"

