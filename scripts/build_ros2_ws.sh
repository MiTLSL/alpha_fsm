#!/usr/bin/env bash
set -euo pipefail

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3

set +u
source /opt/ros/humble/setup.bash
source /home/sevnova/projects/fsm_simulation_plan/alpha_fsm/fsm_ws/install/setup.bash
set -u

cd /home/sevnova/ros2_ws
colcon build --symlink-install "$@"
