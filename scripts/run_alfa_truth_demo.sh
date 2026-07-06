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

pkill -INT -f '/home/sevnova/ros2_ws/install/robot_state_machine/lib/robot_state_machine/alfa_truth_demo_state_machine_node' 2>/dev/null || true
pkill -INT -f '/home/sevnova/ros2_ws/install/isaac_visualization_bridge/lib/isaac_visualization_bridge/validation_status_node' 2>/dev/null || true
sleep 0.8
pkill -TERM -f '/home/sevnova/ros2_ws/install/robot_state_machine/lib/robot_state_machine/alfa_truth_demo_state_machine_node' 2>/dev/null || true
pkill -TERM -f '/home/sevnova/ros2_ws/install/isaac_visualization_bridge/lib/isaac_visualization_bridge/validation_status_node' 2>/dev/null || true

exec ros2 launch robot_bringup alfa_truth_demo.launch.py "$@"
