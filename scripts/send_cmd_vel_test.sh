#!/usr/bin/env bash
set -euo pipefail

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"

set +u
source /opt/ros/humble/setup.bash
source /home/sevnova/projects/fsm_simulation_plan/alpha_fsm/fsm_ws/install/setup.bash
source /home/sevnova/ros2_ws/install/setup.bash
set -u

timeout "${1:-4}" ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
