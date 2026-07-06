#!/usr/bin/env bash
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"

set +u
source /opt/ros/humble/setup.bash
source /home/sevnova/projects/fsm_simulation_plan/alpha_fsm/fsm_ws/install/setup.bash
source /home/sevnova/ros2_ws/install/setup.bash
set -u

exec ros2 launch robot_bringup alfa_task_demo.launch.py "$@"
