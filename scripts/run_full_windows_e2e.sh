#!/usr/bin/env bash
set -euo pipefail

bash /home/sevnova/ros2_ws/scripts/start_windows_isaac.sh
bash /home/sevnova/ros2_ws/scripts/wait_windows_isaac_ready.sh
bash /home/sevnova/ros2_ws/scripts/verify_alfa_windows_e2e.sh
