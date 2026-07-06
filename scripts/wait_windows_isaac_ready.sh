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

timeout_sec="${TIMEOUT_SEC:-180}"
deadline=$((SECONDS + timeout_sec))

echo "Waiting for Windows Isaac DDS topics: /clock and /alfa/state_json"
while (( SECONDS < deadline )); do
  clock_ok=0
  state_ok=0
  if ros2 topic info /clock >/tmp/alfa_clock_info.txt 2>&1 && grep -q "Publisher count: [1-9]" /tmp/alfa_clock_info.txt; then
    clock_ok=1
  fi
  if ros2 topic info /alfa/state_json >/tmp/alfa_state_info.txt 2>&1 && grep -q "Publisher count: [1-9]" /tmp/alfa_state_info.txt; then
    state_ok=1
  fi

  if (( clock_ok == 1 && state_ok == 1 )); then
    echo "Windows Isaac DDS ready."
    ros2 topic info /clock -v | sed -n '1,40p'
    ros2 topic info /alfa/state_json -v | sed -n '1,40p'
    exit 0
  fi

  echo "  waiting... clock=${clock_ok} state_json=${state_ok}"
  sleep 3
done

echo "Timed out waiting for Windows Isaac DDS topics." >&2
echo "--- /clock ---" >&2
cat /tmp/alfa_clock_info.txt >&2 || true
echo "--- /alfa/state_json ---" >&2
cat /tmp/alfa_state_info.txt >&2 || true
exit 1
