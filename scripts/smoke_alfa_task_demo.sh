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

log_file=/tmp/alfa_task_demo_smoke.log
rm -f "$log_file"

ros2 launch robot_bringup alfa_task_demo.launch.py \
  auto_start:=true \
  wait_for_clock:=false \
  stage_time_scale:=0.03 \
  >"$log_file" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT "$launch_pid" 2>/dev/null || true
  for _ in $(seq 1 16); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      wait "$launch_pid" 2>/dev/null || true
      return
    fi
    sleep 0.25
  done
  kill -TERM "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

/usr/bin/python3 - <<'PY'
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__("alfa_task_demo_smoke_monitor")
        self.commands = 0
        self.events = []
        self.stage = ""
        self.complete = False
        self.create_subscription(String, "/alfa/command_json", self._command, 10)
        self.create_subscription(String, "/alfa/fsm_event_json", self._event, 10)
        self.create_subscription(String, "/alfa_task/state_json", self._state, 10)

    def _command(self, msg):
        self.commands += 1

    def _event(self, msg):
        try:
            event = json.loads(msg.data)
        except Exception:
            return
        self.events.append(event.get("event_type", ""))
        if len(self.events) <= 5:
            print("event", event.get("event_type"), event.get("stage", ""), flush=True)

    def _state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.stage = data.get("stage", "")
        self.complete = bool(data.get("complete", False))


rclpy.init()
node = Monitor()
deadline = time.monotonic() + 15.0
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.complete and node.commands > 5 and "task_completed" in node.events:
            print(f"SMOKE_OK commands={node.commands} final_stage={node.stage}", flush=True)
            raise SystemExit(0)
    raise SystemExit(f"SMOKE_TIMEOUT commands={node.commands} stage={node.stage} events={node.events[-5:]}")
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY

cleanup
trap - EXIT

echo "--- launch errors ---"
grep -E "Traceback|Exception|ERROR|failed" "$log_file" | tail -80 || true
