set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u 2>/dev/null || true
cd ~/cl2_ws
colcon build --symlink-install
echo "Local build complete."
