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

stage_time_scale="${STAGE_TIME_SCALE:-0.5}"
timeout_sec="${TIMEOUT_SEC:-90}"
log_file=/tmp/alfa_windows_e2e.log
rm -f "$log_file"

ros2 launch robot_bringup alfa_task_demo.launch.py \
  auto_start:=true \
  wait_for_clock:=true \
  stage_time_scale:="$stage_time_scale" \
  >"$log_file" 2>&1 &
launch_pid=$!

cleanup() {
  kill -INT "$launch_pid" 2>/dev/null || true
  for _ in $(seq 1 24); do
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

/usr/bin/python3 - "$timeout_sec" <<'PY'
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def number(value, default=0.0):
    try:
        value = float(value)
    except Exception:
        return default
    return value if math.isfinite(value) else default


def base_pose(state_msg):
    state = state_msg.get("state", {}) if isinstance(state_msg, dict) else {}
    base = state.get("base", {}) if isinstance(state, dict) else {}
    return (
        number(base.get("x")),
        number(base.get("y")),
        number(base.get("yaw_rad")),
    )


def container_angle(state_msg):
    state = state_msg.get("state", {}) if isinstance(state_msg, dict) else {}
    doors = state.get("doors", {}) if isinstance(state, dict) else {}
    return number(doors.get("container_angle_deg"))


def joint_values(state_msg):
    state = state_msg.get("state", {}) if isinstance(state_msg, dict) else {}
    joints = state.get("joints", {}) if isinstance(state, dict) else {}
    values = []

    def walk(value):
        if isinstance(value, dict):
            for item in value.values():
                walk(item)
            return
        try:
            values.append(number(value))
        except Exception:
            pass

    walk(joints)
    return values


def movement_delta(first, last):
    if not first or not last:
        return 0.0
    fx, fy, fyaw = base_pose(first)
    lx, ly, lyaw = base_pose(last)
    return max(abs(lx - fx), abs(ly - fy), abs(lyaw - fyaw))


def joint_delta(first, last):
    first_values = joint_values(first)
    last_values = joint_values(last)
    if not first_values or not last_values:
        return 0.0
    count = min(len(first_values), len(last_values))
    return max(abs(last_values[index] - first_values[index]) for index in range(count))


class Monitor(Node):
    def __init__(self):
        super().__init__("alfa_windows_e2e_monitor")
        self.clock_seen = False
        self.commands = 0
        self.events = []
        self.task_complete = False
        self.task_stage = ""
        self.windows_states = []
        self.first_windows_state = None
        self.last_windows_state = None
        self.last_report = 0.0
        self.started = time.monotonic()
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(String, "/alfa/command_json", self._command, 10)
        self.create_subscription(String, "/alfa/fsm_event_json", self._event, 10)
        self.create_subscription(String, "/alfa_task/state_json", self._task_state, 10)
        self.create_subscription(String, "/alfa/state_json", self._windows_state, 10)

    def _on_clock(self, msg):
        del msg
        self.clock_seen = True

    def _command(self, msg):
        del msg
        self.commands += 1

    def _event(self, msg):
        try:
            event = json.loads(msg.data)
        except Exception:
            return
        event_type = event.get("event_type", "")
        self.events.append(event_type)
        if event_type == "task_completed":
            self.task_complete = True

    def _task_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.task_stage = str(data.get("stage", ""))
        self.task_complete = self.task_complete or bool(data.get("complete", False))

    def _windows_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if self.first_windows_state is None:
            self.first_windows_state = data
        self.last_windows_state = data
        self.windows_states.append(data)
        self.windows_states = self.windows_states[-200:]

    def changed(self):
        if self.first_windows_state is None or not self.windows_states:
            return False
        move = self.max_move_delta()
        angle_delta = self.max_container_angle_delta()
        joint = self.max_joint_delta()
        return move >= 0.02 or angle_delta >= 5.0 or joint >= 0.05

    def max_move_delta(self):
        if self.first_windows_state is None:
            return 0.0
        return max((movement_delta(self.first_windows_state, item) for item in self.windows_states), default=0.0)

    def max_container_angle_delta(self):
        if self.first_windows_state is None:
            return 0.0
        first_angle = container_angle(self.first_windows_state)
        return max((abs(container_angle(item) - first_angle) for item in self.windows_states), default=0.0)

    def max_joint_delta(self):
        if self.first_windows_state is None:
            return 0.0
        return max((joint_delta(self.first_windows_state, item) for item in self.windows_states), default=0.0)

    def report(self):
        move = self.max_move_delta()
        angle = self.max_container_angle_delta()
        joint = self.max_joint_delta()
        print(
            f"clock={self.clock_seen} states={len(self.windows_states)} commands={self.commands} "
            f"stage={self.task_stage or 'none'} complete={self.task_complete} "
            f"max_move_delta={move:.3f} max_container_angle_delta={angle:.3f} max_joint_delta={joint:.3f}",
            flush=True,
        )


timeout_sec = float(sys.argv[1])
rclpy.init()
node = Monitor()
deadline = time.monotonic() + timeout_sec
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now - node.last_report >= 2.0:
            node.report()
            node.last_report = now
        if node.clock_seen and node.task_complete and node.commands > 10 and len(node.windows_states) > 5 and node.changed():
            node.report()
            print("E2E_OK", flush=True)
            raise SystemExit(0)
    node.report()
    missing = []
    if not node.clock_seen:
        missing.append("/clock")
    if len(node.windows_states) <= 5:
        missing.append("/alfa/state_json")
    if node.commands <= 10:
        missing.append("/alfa/command_json")
    if not node.task_complete:
        missing.append("task_completed")
    if not node.changed():
        missing.append("windows_state_changed")
    raise SystemExit("E2E_TIMEOUT missing=" + ",".join(missing))
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY

cleanup
trap - EXIT

echo "--- launch errors ---"
grep -E "Traceback|Exception|ERROR|failed" "$log_file" | tail -80 || true
