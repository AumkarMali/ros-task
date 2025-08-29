#!/usr/bin/env bash
set -eo pipefail

# Allow unset vars while sourcing ROS env
set +u
source /opt/ros/humble/setup.bash
source ~/cl2_ws/install/setup.bash
set -u 2>/dev/null || true

export LIBGL_ALWAYS_SOFTWARE=1

XG="${XG:-2.0}"
YG="${YG:-1.0}"
TH="${TH:-1.57}"
ODOM="${ODOM:-/odom}"
CMD="${CMD:-/cmd_vel}"

exec ros2 launch limo_control all.launch.py \
  xg:="$XG" yg:="$YG" thetag:="$TH" \
  odom_topic:="$ODOM" cmd_vel_topic:="$CMD"

