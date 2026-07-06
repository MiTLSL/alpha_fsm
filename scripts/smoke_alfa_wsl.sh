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

log_file=/tmp/alfa_wsl_bringup_smoke.log
rm -f "$log_file"

ros2 launch robot_bringup alfa_wsl_bringup.launch.py \
  auto_start:=true \
  write_command_file:=false \
  enable_cmd_vel_bridge:=false \
  >"$log_file" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT "$launch_pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      wait "$launch_pid" 2>/dev/null || true
      return
    fi
    sleep 0.5
  done
  kill -TERM "$launch_pid" 2>/dev/null || true
  sleep 1
  kill -KILL "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 5

/usr/bin/python3 - <<'PY'
import time

import rclpy
from fsm_msgs.msg import FsmStateSnapshot
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class Monitor(Node):
    def __init__(self):
        super().__init__("alfa_wsl_smoke_monitor")
        self.clock_seen = False
        self.command_seen = False
        self.event_seen = False
        self.validation_seen = False
        self.system_state = ""
        self.task_state = ""
        self.started = time.monotonic()
        self.create_subscription(Clock, "/clock", self._clock, 10)
        self.create_subscription(String, "/alfa/command_json", self._command, 10)
        self.create_subscription(String, "/alfa/fsm_event_json", self._event, 10)
        self.create_subscription(String, "/alfa/validation_status", self._validation, 10)
        self.create_subscription(FsmStateSnapshot, "/fsm/system_state", self._system, 10)
        self.create_subscription(FsmStateSnapshot, "/fsm/task_state", self._task, 10)

    def _clock(self, msg):
        del msg
        self.clock_seen = True

    def _command(self, msg):
        if not self.command_seen:
            print("command_json_seen", msg.data[:180], flush=True)
        self.command_seen = True

    def _event(self, msg):
        if not self.event_seen:
            print("fsm_event_json_seen", msg.data[:180], flush=True)
        self.event_seen = True

    def _validation(self, msg):
        if not self.validation_seen:
            print("validation_status_seen", msg.data[:180], flush=True)
        self.validation_seen = True

    def _system(self, msg):
        self.system_state = msg.current_state

    def _task(self, msg):
        self.task_state = msg.current_state


rclpy.init()
node = Monitor()
deadline = time.monotonic() + 70.0
last_print = 0.0
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now - last_print >= 1.0:
            print(
                f"t={now - node.started:.1f}s clock={node.clock_seen} "
                f"command={node.command_seen} event={node.event_seen} validation={node.validation_seen} "
                f"system={node.system_state or 'none'} task={node.task_state or 'none'}",
                flush=True,
            )
            last_print = now
        if (
            node.clock_seen
            and node.command_seen
            and node.event_seen
            and node.validation_seen
            and node.system_state == "STANDBY"
            and node.task_state == "WAIT_TASK"
            and now - node.started > 8.0
        ):
            print("SMOKE_OK", flush=True)
            raise SystemExit(0)
    raise SystemExit("SMOKE_TIMEOUT")
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY

cleanup
trap - EXIT

echo "--- launch errors ---"
grep -E "Traceback|Exception|ERROR|failed" "$log_file" | grep -v "rcl_shutdown already called" | tail -80 || true
