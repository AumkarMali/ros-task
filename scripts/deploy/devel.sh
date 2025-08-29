set -eo pipefail
set +u
source /opt/ros/humble/setup.bash
source ~/cl2_ws/install/setup.bash
set -u 2>/dev/null || true
export LIBGL_ALWAYS_SOFTWARE=1   # keep Gazebo stable on WSL
exec bash
