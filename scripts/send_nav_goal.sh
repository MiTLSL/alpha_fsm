#!/usr/bin/env bash
set -euo pipefail

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"

set +u
source /opt/ros/humble/setup.bash
source /home/sevnova/projects/fsm_simulation_plan/alpha_fsm/fsm_ws/install/setup.bash
source /home/sevnova/ros2_ws/install/setup.bash
set -u

exec ros2 run robot_navigation nav_goal_sender --ros-args \
  -p x:="${NAV_X:--2.0}" \
  -p y:="${NAV_Y:-1.0}" \
  -p yaw:="${NAV_YAW:-0.0}" \
  "$@"
