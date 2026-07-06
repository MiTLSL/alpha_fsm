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

timeout_sec="${TIMEOUT_SEC:-240}"
target_cargo_name="${TARGET_CARGO_NAME:-}"
task_id="alfa_truth_pick_return_demo_$(date +%s)_$$"
log_file=/tmp/alfa_truth_demo_e2e.log
rm -f "$log_file"

cleanup_existing_demo_nodes() {
  pkill -INT -f '/home/sevnova/ros2_ws/install/robot_state_machine/lib/robot_state_machine/alfa_truth_demo_state_machine_node' 2>/dev/null || true
  pkill -INT -f '/home/sevnova/ros2_ws/install/isaac_visualization_bridge/lib/isaac_visualization_bridge/validation_status_node' 2>/dev/null || true
  sleep 0.8
  pkill -TERM -f '/home/sevnova/ros2_ws/install/robot_state_machine/lib/robot_state_machine/alfa_truth_demo_state_machine_node' 2>/dev/null || true
  pkill -TERM -f '/home/sevnova/ros2_ws/install/isaac_visualization_bridge/lib/isaac_visualization_bridge/validation_status_node' 2>/dev/null || true
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 0.5
  ros2 daemon start >/dev/null 2>&1 || true
}

cleanup_existing_demo_nodes

launch_args=(
  auto_start:=true
  wait_for_clock:=true
  task_id:="$task_id"
)
if [[ -n "$target_cargo_name" ]]; then
  launch_args+=(target_cargo_name:="$target_cargo_name")
fi

ros2 launch robot_bringup alfa_truth_demo.launch.py "${launch_args[@]}" >"$log_file" 2>&1 &
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

/usr/bin/python3 - "$timeout_sec" "$task_id" <<'PY'
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


def xy_distance(a, b):
    if not a or not b:
        return 0.0
    return math.hypot(number(a.get("x")) - number(b.get("x")), number(a.get("y")) - number(b.get("y")))


def cargo_item(state_msg, target_path):
    state = state_msg.get("state", {}) if isinstance(state_msg, dict) else {}
    cargo = state.get("cargo", {}) if isinstance(state, dict) else {}
    for item in cargo.get("items", []) if isinstance(cargo, dict) else []:
        if isinstance(item, dict) and item.get("path") == target_path:
            return item
    return None


def item_center(item):
    if not isinstance(item, dict):
        return None
    center = item.get("bbox_center") or item.get("position")
    return center if isinstance(center, dict) else None


class Monitor(Node):
    def __init__(self, task_id):
        super().__init__("alfa_truth_demo_e2e_monitor")
        self.task_id = str(task_id)
        self.clock = False
        self.states = 0
        self.commands = 0
        self.demo_states = 0
        self.stage = "none"
        self.complete = False
        self.failed = False
        self.error = ""
        self.home_pose = None
        self.base_pose = None
        self.target_path = ""
        self.initial_target_center = None
        self.max_target_delta = 0.0
        self.held_seen = False
        self.released_after_held = False
        self.last_windows_state = {}
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(String, "/alfa/state_json", self._windows_state, 10)
        self.create_subscription(String, "/alfa/command_json", self._command, 10)
        self.create_subscription(String, "/alfa_truth_demo/state_json", self._demo_state, 10)

    def _on_clock(self, _msg):
        self.clock = True

    def _command(self, _msg):
        self.commands += 1

    def _demo_state(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if str(data.get("task_id", "")) != self.task_id:
            return
        self.demo_states += 1
        self.stage = str(data.get("stage", self.stage))
        self.complete = bool(data.get("complete", False))
        self.failed = bool(data.get("failed", False))
        self.error = str(data.get("last_error", ""))
        self.home_pose = data.get("home_pose") if isinstance(data.get("home_pose"), dict) else self.home_pose
        self.base_pose = data.get("base_pose") if isinstance(data.get("base_pose"), dict) else self.base_pose
        target = data.get("target") if isinstance(data.get("target"), dict) else {}
        if target.get("path"):
            self.target_path = str(target["path"])

    def _windows_state(self, msg):
        self.states += 1
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.last_windows_state = data
        state = data.get("state", {}) if isinstance(data, dict) else {}
        suction = state.get("suction", {}) if isinstance(state, dict) else {}
        if self.target_path and isinstance(suction, dict):
            held = any(str(value) == self.target_path for value in suction.values() if value)
            self.held_seen = self.held_seen or held
            self.released_after_held = self.released_after_held or (self.held_seen and not held)

        if not self.target_path:
            return
        item = cargo_item(data, self.target_path)
        center = item_center(item)
        if center is None:
            return
        if self.initial_target_center is None:
            self.initial_target_center = dict(center)
        self.max_target_delta = max(self.max_target_delta, xy_distance(self.initial_target_center, center))

    def summary(self):
        home_error = xy_distance(self.home_pose, self.base_pose) if self.home_pose and self.base_pose else 999.0
        return (
            f"clock={self.clock} states={self.states} commands={self.commands} "
            f"demo_states={self.demo_states} stage={self.stage} complete={self.complete} "
            f"failed={self.failed} held_seen={self.held_seen} released_after_held={self.released_after_held} "
            f"target_delta={self.max_target_delta:.3f} home_error={home_error:.3f} error={self.error}"
        )


rclpy.init()
node = Monitor(sys.argv[2])
deadline = time.monotonic() + float(sys.argv[1])
last_report = 0.0
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now - last_report > 5.0:
            print(node.summary(), flush=True)
            last_report = now
        if node.failed:
            raise SystemExit("TRUTH_DEMO_FAILED " + node.summary())
        if node.complete:
            home_error = xy_distance(node.home_pose, node.base_pose) if node.home_pose and node.base_pose else 999.0
            checks = [
                (node.clock, "clock"),
                (node.states >= 3, "states"),
                (node.commands >= 5, "commands"),
                (node.demo_states >= 3, "demo_states"),
                (bool(node.target_path), "target_path"),
                (node.held_seen, "held_seen"),
                (node.released_after_held, "released_after_held"),
                (node.max_target_delta >= 0.05, "target_moved"),
                (home_error <= 0.45, "home_return"),
            ]
            missing = [name for ok, name in checks if not ok]
            if missing:
                raise SystemExit("TRUTH_DEMO_INCOMPLETE missing=" + ",".join(missing) + " " + node.summary())
            print(node.summary(), flush=True)
            print("TRUTH_DEMO_E2E_OK", flush=True)
            raise SystemExit(0)
    raise SystemExit("TRUTH_DEMO_TIMEOUT " + node.summary())
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY

cleanup
trap - EXIT

echo "--- launch errors ---"
grep -E "Traceback|Exception|ERROR|failed|FAILED" "$log_file" | tail -120 || true
