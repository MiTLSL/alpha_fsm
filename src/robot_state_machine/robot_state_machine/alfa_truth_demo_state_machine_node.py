from __future__ import annotations

import json
import math
import re
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import rclpy
from fsm_msgs.msg import FsmStateSnapshot
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_msgs.msg import BehaviorState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from isaac_sim_bridge.alfa_command import command_json, default_alfa_command
except ImportError:
    def default_alfa_command() -> dict[str, Any]:
        return {
            "base": {"linear": 0.0, "yaw": 0.0},
            "turn_joint": 0.0,
            "updown": 0.0,
            "warehouse_door": None,
            "container_door": None,
            "arm": {
                "left": {f"joint{i}": 0.0 for i in range(1, 7)},
                "right": {f"joint{i}": 0.0 for i in range(1, 7)},
            },
            "suction": {"left": "open", "right": "open"},
            "reset": False,
        }

    def command_json(command: dict[str, Any]) -> str:
        return json.dumps(command, ensure_ascii=False, sort_keys=True, allow_nan=False)


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class TruthTarget:
    path: str = ""
    name: str = ""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    held_by: Optional[str] = None
    suction_side: str = "left"
    align_y: Optional[float] = None


@dataclass
class TruthDemoRuntime:
    active: bool = False
    complete: bool = False
    failed: bool = False
    stage: str = "IDLE"
    stage_started_at: float = 0.0
    started_at: float = 0.0
    task_id: str = "alfa_truth_pick_return_demo"
    clock_seen: bool = False
    windows_state_seen: bool = False
    cargo_truth_seen: bool = False
    home_pose: Optional[Pose2D] = None
    target: Optional[TruthTarget] = None
    target_candidates: list[TruthTarget] = field(default_factory=list)
    target_index: int = 0
    completed_targets: list[str] = field(default_factory=list)
    current_goal: Optional[Pose2D] = None
    pregrasp_waypoint_index: int = 0
    home_waypoint_index: int = 0
    last_error: str = ""
    events: list[dict[str, Any]] = field(default_factory=list)


def _finite(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except Exception:
        return default
    return result if math.isfinite(result) else default


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _angle_delta(target: float, current: float) -> float:
    return math.atan2(math.sin(target - current), math.cos(target - current))


def _distance(a: Pose2D, x: float, y: float) -> float:
    return math.hypot(float(x) - a.x, float(y) - a.y)


def _arm_pregrasp() -> dict[str, dict[str, float]]:
    left = {f"joint{i}": 0.0 for i in range(1, 7)}
    right = {f"joint{i}": 0.0 for i in range(1, 7)}
    left.update({"joint2": 0.30, "joint3": -0.22, "joint5": 0.04})
    right.update({"joint2": 0.30, "joint3": 0.22, "joint5": 0.04})
    return {"left": left, "right": right}


def _arm_contact() -> dict[str, dict[str, float]]:
    left = {f"joint{i}": 0.0 for i in range(1, 7)}
    right = {f"joint{i}": 0.0 for i in range(1, 7)}
    left.update({"joint2": 0.16, "joint3": -0.10})
    right.update({"joint2": 0.16, "joint3": 0.10})
    return {"left": left, "right": right}


class AlfaTruthDemoStateMachineNode(Node):
    def __init__(self) -> None:
        super().__init__("alfa_truth_demo_state_machine_node")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("wait_for_clock", True)
        self.declare_parameter("start_delay_sec", 1.0)
        self.declare_parameter("command_rate_hz", 1.0)
        self.declare_parameter("task_id", "alfa_truth_pick_return_demo")
        self.declare_parameter("target_cargo_name", "")
        self.declare_parameter("approach_distance_m", 1.05)
        self.declare_parameter("contact_distance_m", 0.30)
        self.declare_parameter("initial_forward_clearance_m", 2.0)
        self.declare_parameter("xy_tolerance_m", 0.12)
        self.declare_parameter("yaw_tolerance_rad", 0.12)
        self.declare_parameter("max_linear_cmd", 0.9)
        self.declare_parameter("max_yaw_cmd", 2.0)
        self.declare_parameter("command_topic", "/alfa/command_json")
        self.declare_parameter("event_topic", "/alfa/fsm_event_json")
        self.declare_parameter("state_json_topic", "/alfa_truth_demo/state_json")
        self.declare_parameter("windows_state_topic", "/alfa/state_json")
        self.declare_parameter("behavior_state_topic", "/behavior_state")
        self.declare_parameter("structured_state_topic", "/robot/behavior_state")
        self.declare_parameter("fsm_state_topic", "/fsm/alfa_truth_demo_state")
        self.declare_parameter("hold_after_complete", True)

        self._runtime = TruthDemoRuntime(task_id=str(self.get_parameter("task_id").value))
        self._node_start = time.monotonic()
        self._latest_clock_sec: Optional[float] = None
        self._latest_base: Optional[Pose2D] = None
        self._latest_cargo: list[dict[str, Any]] = []
        self._latest_windows_state: dict[str, Any] = {}
        self._last_command_stage = ""

        self._command_pub = self.create_publisher(String, self.get_parameter("command_topic").value, 10)
        self._event_pub = self.create_publisher(String, self.get_parameter("event_topic").value, 50)
        self._state_json_pub = self.create_publisher(String, self.get_parameter("state_json_topic").value, 10)
        self._behavior_pub = self.create_publisher(String, self.get_parameter("behavior_state_topic").value, 10)
        self._structured_pub = self.create_publisher(BehaviorState, self.get_parameter("structured_state_topic").value, 10)
        self._fsm_state_pub = self.create_publisher(FsmStateSnapshot, self.get_parameter("fsm_state_topic").value, 10)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(String, self.get_parameter("windows_state_topic").value, self._on_windows_state, 10)
        self.create_service(Trigger, "/alfa_truth_demo/start", self._handle_start)
        self.create_service(Trigger, "/alfa_truth_demo/stop", self._handle_stop)
        self.create_service(Trigger, "/alfa_truth_demo/reset", self._handle_reset)

        rate = max(0.1, _finite(self.get_parameter("command_rate_hz").value, 1.0))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info("alfa_truth_demo_state_machine_node ready")

    def _bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _on_clock(self, msg: Clock) -> None:
        self._runtime.clock_seen = True
        self._latest_clock_sec = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9

    def _on_windows_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        state = payload.get("state", payload) if isinstance(payload, dict) else {}
        if not isinstance(state, dict):
            return
        self._latest_windows_state = state
        self._runtime.windows_state_seen = True

        base = state.get("base", {})
        if isinstance(base, dict):
            self._latest_base = Pose2D(
                x=_finite(base.get("x")),
                y=_finite(base.get("y")),
                yaw=_finite(base.get("yaw_rad")),
            )

        cargo = state.get("cargo", {})
        items = cargo.get("items", []) if isinstance(cargo, dict) else []
        if isinstance(items, list):
            self._latest_cargo = [item for item in items if isinstance(item, dict)]
            self._runtime.cargo_truth_seen = bool(self._latest_cargo)
        self._refresh_target_from_latest_truth()

    def _handle_start(self, request, response):
        del request
        if self._runtime.active:
            response.success = False
            response.message = f"already active: {self._runtime.stage}"
            return response
        self._start_task()
        response.success = True
        response.message = f"started {self._runtime.task_id}"
        return response

    def _handle_stop(self, request, response):
        del request
        self._runtime.active = False
        self._runtime.failed = False
        self._set_stage("IDLE")
        self._publish_command(self._command(), "STOPPED")
        self._publish_event("task_stopped")
        response.success = True
        response.message = "stopped"
        return response

    def _handle_reset(self, request, response):
        del request
        self._runtime = TruthDemoRuntime(task_id=str(self.get_parameter("task_id").value))
        self._publish_command(self._command(reset=True), "RESET_BY_SERVICE")
        self._publish_event("task_reset")
        response.success = True
        response.message = "reset command published"
        return response

    def _tick(self) -> None:
        now = time.monotonic()
        if self._should_auto_start(now):
            self._start_task()

        if self._runtime.active:
            command = self._command_for_active_stage(now)
            self._publish_command(command, self._runtime.stage)
        elif self._runtime.complete and self._bool_param("hold_after_complete"):
            self._publish_command(self._command(suction={"left": "open", "right": "open"}), "HOLD_COMPLETE")

        self._publish_state(now)

    def _should_auto_start(self, now: float) -> bool:
        if not self._bool_param("auto_start") or self._runtime.active or self._runtime.complete or self._runtime.failed:
            return False
        if now - self._node_start < _finite(self.get_parameter("start_delay_sec").value, 1.0):
            return False
        if self._bool_param("wait_for_clock") and not self._runtime.clock_seen:
            return False
        return self._runtime.windows_state_seen and self._runtime.cargo_truth_seen

    def _start_task(self) -> None:
        clock_seen = self._runtime.clock_seen
        windows_state_seen = self._latest_base is not None
        cargo_truth_seen = bool(self._latest_cargo)
        self._runtime = TruthDemoRuntime(task_id=str(self.get_parameter("task_id").value))
        self._runtime.clock_seen = clock_seen
        self._runtime.windows_state_seen = windows_state_seen
        self._runtime.cargo_truth_seen = cargo_truth_seen
        self._runtime.active = True
        self._runtime.started_at = self._stage_time()
        self._set_stage("RESET_SCENE")
        self._publish_event("task_started", task_id=self._runtime.task_id)

    def _set_stage(self, stage: str) -> None:
        if self._runtime.stage == stage:
            return
        self._runtime.stage = stage
        self._runtime.stage_started_at = self._stage_time()
        if stage == "NAVIGATE_TO_PREGRASP":
            self._runtime.pregrasp_waypoint_index = 0
        elif stage == "NAVIGATE_HOME":
            self._runtime.home_waypoint_index = 0
        self._publish_event("stage_entered", stage=stage)
        self.get_logger().info(f"stage={stage}")

    def _fail(self, reason: str) -> dict[str, Any]:
        self._runtime.active = False
        self._runtime.failed = True
        self._runtime.last_error = str(reason)
        self._set_stage("FAILED")
        self._publish_event("task_failed", reason=reason)
        return self._command(suction={"left": "open", "right": "open"})

    def _complete(self) -> dict[str, Any]:
        self._runtime.active = False
        self._runtime.complete = True
        self._set_stage("COMPLETE")
        self._publish_event("task_completed", task_id=self._runtime.task_id)
        return self._command(suction={"left": "open", "right": "open"})

    def _elapsed(self, now: float) -> float:
        return max(0.0, self._stage_time(now) - self._runtime.stage_started_at)

    def _stage_time(self, fallback_now: Optional[float] = None) -> float:
        if self._latest_clock_sec is not None:
            return self._latest_clock_sec
        return time.monotonic() if fallback_now is None else fallback_now

    def _command_for_active_stage(self, now: float) -> dict[str, Any]:
        stage = self._runtime.stage
        elapsed = self._elapsed(now)

        if stage == "RESET_SCENE":
            if elapsed >= 0.8:
                self._set_stage("CAPTURE_TRUTH")
            return self._command(reset=True)

        if stage == "CAPTURE_TRUTH":
            if self._latest_base is None:
                if elapsed > 5.0:
                    return self._fail("missing base truth from /alfa/state_json")
                return self._command()
            self._runtime.home_pose = Pose2D(self._latest_base.x, self._latest_base.y, self._latest_base.yaw)
            target = self._select_target()
            if target is None:
                if elapsed > 5.0:
                    return self._fail("missing cargo truth from /alfa/state_json")
                return self._command()
            self._runtime.target = target
            self._publish_event("truth_selected", target=target.__dict__, home=self._runtime.home_pose.__dict__)
            self._set_stage("PREPARE_BOX_WALL")
            return self._command()

        if stage == "PREPARE_BOX_WALL":
            if elapsed >= 1.0:
                self._set_stage("NAVIGATE_TO_PREGRASP")
            return self._command(warehouse_door="open")

        if stage == "NAVIGATE_TO_PREGRASP":
            target = self._runtime.target
            home = self._runtime.home_pose
            if target is None or home is None:
                return self._fail("target or home pose missing")
            waypoints = self._pregrasp_waypoints(target)
            waypoint_index = min(self._runtime.pregrasp_waypoint_index, len(waypoints) - 1)
            goal_x, goal_y = waypoints[waypoint_index]
            self._runtime.current_goal = Pose2D(goal_x, goal_y, 0.0)
            command, reached = self._drive_to_xy(goal_x, goal_y)
            command.update({"warehouse_door": "open"})
            if reached and waypoint_index < len(waypoints) - 1:
                self._runtime.pregrasp_waypoint_index += 1
            elif reached:
                self._set_stage("ALIGN_TO_TARGET")
            elif elapsed > 70.0:
                return self._fail("navigate to pregrasp timed out")
            return self._command(**command)

        if stage == "ALIGN_TO_TARGET":
            target = self._runtime.target
            if target is None:
                return self._fail("target missing")
            align_y = self._target_align_y(target)
            self._runtime.current_goal = Pose2D(target.x, align_y, 0.0)
            command, aligned = self._face_xy(target.x, align_y)
            command.update({"warehouse_door": "open"})
            if aligned or elapsed > 6.0:
                self._set_stage("MOVE_TO_PREGRASP")
            return self._command(**command)

        if stage == "MOVE_TO_PREGRASP":
            if elapsed >= 3.5:
                self._set_stage("APPROACH_AND_GRASP")
            return self._command(
                warehouse_door="open",
                updown=0.16,
                arm=_arm_pregrasp(),
            )

        if stage == "APPROACH_AND_GRASP":
            target = self._runtime.target
            home = self._runtime.home_pose
            if target is None or home is None:
                return self._fail("target or home pose missing")
            if self._target_held(target):
                self._set_stage("RETREAT_WITH_BOX")
            elif elapsed > 12.0:
                return self._fail(f"suction did not attach target {target.name}")
            goal = self._runtime.current_goal or self._wall_staging_point(target)
            self._runtime.current_goal = goal
            suction, suction_target = self._target_suction_command(target)
            command = {
                "base": {"linear": 0.0, "yaw": 0.0},
                "warehouse_door": "open",
                "updown": 0.04,
                "arm": _arm_contact(),
                "suction": suction,
                "suction_target": suction_target,
            }
            return self._command(**command)

        if stage == "RETREAT_WITH_BOX":
            target = self._runtime.target
            if target is None:
                return self._fail("target missing")
            if elapsed >= 3.5:
                self._set_stage("NAVIGATE_HOME")
            suction, suction_target = self._target_suction_command(target)
            return self._command(
                base={"linear": -0.25, "yaw": 0.0},
                warehouse_door="open",
                updown=-0.04,
                suction=suction,
                suction_target=suction_target,
            )

        if stage == "NAVIGATE_HOME":
            target = self._runtime.target
            home = self._runtime.home_pose
            if target is None or home is None:
                return self._fail("target or home pose missing")
            waypoints = self._home_waypoints(target, home)
            waypoint_index = min(self._runtime.home_waypoint_index, len(waypoints) - 1)
            goal_x, goal_y, goal_yaw = waypoints[waypoint_index]
            final_yaw = goal_yaw if waypoint_index == len(waypoints) - 1 else None
            self._runtime.current_goal = Pose2D(goal_x, goal_y, goal_yaw or 0.0)
            command, reached = self._drive_to_xy(goal_x, goal_y, final_yaw=final_yaw)
            suction, suction_target = self._target_suction_command(target)
            command.update(
                {
                    "warehouse_door": "open",
                    "suction": suction,
                    "suction_target": suction_target,
                }
            )
            if reached and waypoint_index < len(waypoints) - 1:
                self._runtime.home_waypoint_index += 1
            elif reached:
                self._set_stage("DROP_AT_START")
            elif elapsed > 70.0:
                return self._fail("navigate home timed out")
            return self._command(**command)

        if stage == "DROP_AT_START":
            if elapsed >= 1.4:
                return self._complete_current_target_or_continue()
            return self._command(
                warehouse_door="open",
                suction={"left": "open", "right": "open"},
                updown=-0.12,
            )

        return self._command()

    def _complete_current_target_or_continue(self) -> dict[str, Any]:
        if self._runtime.target is not None and self._runtime.target.path not in self._runtime.completed_targets:
            self._runtime.completed_targets.append(self._runtime.target.path)

        next_index = self._runtime.target_index + 1
        if next_index < len(self._runtime.target_candidates):
            self._runtime.target_index = next_index
            self._runtime.target = self._runtime.target_candidates[next_index]
            self._runtime.current_goal = None
            self._runtime.pregrasp_waypoint_index = 0
            self._runtime.home_waypoint_index = 0
            self._publish_event("truth_selected", target=self._runtime.target.__dict__, target_index=next_index)
            self._set_stage("PREPARE_BOX_WALL")
            return self._command(
                warehouse_door="open",
                suction={"left": "open", "right": "open"},
                updown=-0.12,
            )

        return self._complete()

    def _command(self, **updates: Any) -> dict[str, Any]:
        command = default_alfa_command()
        for key, value in updates.items():
            command[key] = value
        command["meta"] = {
            "source": "alfa_truth_demo_state_machine",
            "kind": "truth_pick_return_demo",
            "task_id": self._runtime.task_id,
            "stage": self._runtime.stage,
        }
        return command

    def _publish_command(self, command: dict[str, Any], stage_name: str) -> None:
        msg = String()
        msg.data = command_json(command)
        self._command_pub.publish(msg)
        self._last_command_stage = stage_name

    def _publish_event(self, event_type: str, **payload: Any) -> None:
        event = {
            "event_type": event_type,
            "task_id": self._runtime.task_id,
            "node": self.get_name(),
            "stamp_sec": time.time(),
        }
        event.update(payload)
        msg = String()
        msg.data = json.dumps(event, ensure_ascii=False, sort_keys=True, allow_nan=False)
        self._event_pub.publish(msg)
        self._runtime.events.append(event)
        self._runtime.events = self._runtime.events[-30:]

    def _publish_state(self, now: float) -> None:
        del now
        home = self._runtime.home_pose.__dict__ if self._runtime.home_pose is not None else None
        target = self._runtime.target.__dict__ if self._runtime.target is not None else None
        target_candidates = [candidate.__dict__ for candidate in self._runtime.target_candidates]
        base = self._latest_base.__dict__ if self._latest_base is not None else None
        goal = self._runtime.current_goal.__dict__ if self._runtime.current_goal is not None else None
        data = {
            "task_id": self._runtime.task_id,
            "active": self._runtime.active,
            "complete": self._runtime.complete,
            "failed": self._runtime.failed,
            "stage": self._runtime.stage,
            "clock_seen": self._runtime.clock_seen,
            "windows_state_seen": self._runtime.windows_state_seen,
            "cargo_truth_seen": self._runtime.cargo_truth_seen,
            "home_pose": home,
            "base_pose": base,
            "target": target,
            "target_candidates": target_candidates,
            "target_index": self._runtime.target_index,
            "completed_targets": list(self._runtime.completed_targets),
            "goal_pose": goal,
            "goal_distance": self._goal_distance(),
            "pregrasp_waypoint_index": self._runtime.pregrasp_waypoint_index,
            "home_waypoint_index": self._runtime.home_waypoint_index,
            "target_held": self._target_held(self._runtime.target) if self._runtime.target is not None else False,
            "stage_elapsed_sec": self._elapsed(time.monotonic()),
            "last_error": self._runtime.last_error,
        }
        payload = json.dumps(data, ensure_ascii=False, sort_keys=True, allow_nan=False)

        state_msg = String()
        state_msg.data = payload
        self._state_json_pub.publish(state_msg)

        behavior = String()
        behavior.data = payload
        self._behavior_pub.publish(behavior)

        structured = BehaviorState()
        structured.header.stamp = self.get_clock().now().to_msg()
        structured.header.frame_id = "map"
        structured.system_state = "AUTO_MODE" if self._runtime.active else ("FAULT" if self._runtime.failed else "STANDBY")
        structured.task_state = self._runtime.stage
        structured.wall_state = "TRUTH_PICK_RETURN_DEMO"
        structured.active_substate = self._runtime.stage
        structured.task_id = self._runtime.task_id
        structured.last_error_code = 1 if self._runtime.failed else 0
        structured.summary_json = payload
        self._structured_pub.publish(structured)

        fsm = FsmStateSnapshot()
        fsm.header = structured.header
        fsm.node_name = self.get_name()
        fsm.fsm_name = "AlfaTruthPickReturnFSM"
        fsm.current_state = self._runtime.stage
        fsm.parent_fsm = "AlfaDemo"
        fsm.parent_state = structured.system_state
        fsm.task_id = self._runtime.task_id
        fsm.wall_index = 0
        fsm.phase = 0
        fsm.state_elapsed_sec = float(self._elapsed(time.monotonic()))
        fsm.retry_count = 0
        fsm.last_error_code = structured.last_error_code
        fsm.extra_json = payload
        self._fsm_state_pub.publish(fsm)

    def _refresh_target_from_latest_truth(self) -> None:
        if self._runtime.target is None:
            return
        current = self._runtime.target
        updated = self._target_from_item(
            self._find_cargo_item(current.path),
            suction_side=current.suction_side,
            align_y=current.align_y,
        )
        if updated is not None:
            self._runtime.target = updated

    def _select_target(self) -> Optional[TruthTarget]:
        preferred = str(self.get_parameter("target_cargo_name").value or "")
        if preferred:
            for item in self._latest_cargo:
                path = str(item.get("path", ""))
                name = str(item.get("name", ""))
                if preferred == name or preferred == path or preferred in path:
                    target = self._target_from_item(item)
                    if target is not None:
                        self._runtime.target_candidates = [target]
                        self._runtime.target_index = 0
                        return target

        front_face_targets = self._fixed_wall_top_targets()
        if front_face_targets:
            self._runtime.target_candidates = front_face_targets
            self._runtime.target_index = 0
            return front_face_targets[0]

        candidates = [item for item in self._latest_cargo if str(item.get("path", "")).startswith("/World/CargoBoxWall/")]
        if not candidates:
            candidates = list(self._latest_cargo)
        if not candidates:
            return None
        base = self._latest_base or Pose2D()
        candidates.sort(key=lambda item: self._item_distance_to_base(item, base))
        target = self._target_from_item(candidates[0])
        self._runtime.target_candidates = [target] if target is not None else []
        self._runtime.target_index = 0
        return target

    def _fixed_wall_top_targets(self) -> list[TruthTarget]:
        wall_items: dict[str, dict[str, Any]] = {}
        pattern = re.compile(r"^box_x([0-9]+)_y([0-9]+)_z([0-9]+)$")
        indexed: list[tuple[int, int, int, dict[str, Any]]] = []
        for item in self._latest_cargo:
            path = str(item.get("path", ""))
            name = str(item.get("name", ""))
            if not path.startswith("/World/CargoBoxWall/"):
                continue
            wall_items[name] = item
            match = pattern.match(name)
            if not match:
                continue
            indexed.append((int(match.group(1)), int(match.group(2)), int(match.group(3)), item))

        first = self._target_from_item(wall_items.get("box_x0_y0_z4"), suction_side="left")
        third = self._target_from_item(wall_items.get("box_x2_y0_z4"), suction_side="right")
        fixed_targets = [target for target in (first, third) if target is not None]
        if fixed_targets:
            return fixed_targets

        if not indexed:
            return []

        top_z = max(z for _x, _y, z, _item in indexed)
        top_items = [(x, y, item) for x, y, z, item in indexed if z == top_z]
        front_y = min(y for _x, y, _item in top_items)
        front_row = sorted((x, item) for x, y, item in top_items if y == front_y)
        targets: list[TruthTarget] = []
        by_x = {x: item for x, item in front_row}
        for x_index, side in ((0, "left"), (2, "right")):
            item = by_x.get(x_index)
            if item is None and len(front_row) > x_index:
                item = front_row[x_index][1]
            target = self._target_from_item(item, suction_side=side)
            if target is not None:
                targets.append(target)
        return targets

    def _find_cargo_item(self, path: str) -> Optional[dict[str, Any]]:
        for item in self._latest_cargo:
            if str(item.get("path", "")) == path:
                return item
        return None

    def _target_from_item(
        self,
        item: Optional[dict[str, Any]],
        *,
        suction_side: str = "left",
        align_y: Optional[float] = None,
    ) -> Optional[TruthTarget]:
        if not item:
            return None
        center = item.get("bbox_center", item.get("position", {}))
        if not isinstance(center, dict):
            return None
        return TruthTarget(
            path=str(item.get("path", "")),
            name=str(item.get("name", "")),
            x=_finite(center.get("x")),
            y=_finite(center.get("y")),
            z=_finite(center.get("z")),
            held_by=item.get("held_by") if item.get("held_by") is not None else None,
            suction_side=suction_side if suction_side in ("left", "right") else "left",
            align_y=align_y,
        )

    def _item_distance_to_base(self, item: dict[str, Any], base: Pose2D) -> float:
        target = self._target_from_item(item)
        if target is None:
            return float("inf")
        return math.hypot(target.x - base.x, target.y - base.y)

    def _target_held(self, target: Optional[TruthTarget]) -> bool:
        if target is None:
            return False
        item = self._find_cargo_item(target.path)
        if item and item.get("held_by"):
            return True
        suction = self._latest_windows_state.get("suction", {})
        if isinstance(suction, dict):
            return any(str(value) == target.path for value in suction.values() if value)
        return False

    def _goal_distance(self) -> Optional[float]:
        if self._latest_base is None or self._runtime.current_goal is None:
            return None
        return round(_distance(self._latest_base, self._runtime.current_goal.x, self._runtime.current_goal.y), 4)

    def _approach_point(self, target: TruthTarget, home: Pose2D, offset: float) -> tuple[float, float]:
        dx = home.x - target.x
        dy = home.y - target.y
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return target.x, target.y
        return target.x + dx / length * float(offset), target.y + dy / length * float(offset)

    def _pregrasp_waypoints(self, target: TruthTarget) -> list[tuple[float, float]]:
        staging = self._wall_staging_point(target)
        waypoints: list[tuple[float, float]] = []
        if self._runtime.home_pose is not None:
            home = self._runtime.home_pose
            forward = abs(self._param_float("initial_forward_clearance_m", 2.0))
            waypoints.append((home.x + math.cos(home.yaw) * forward, home.y + math.sin(home.yaw) * forward))
        waypoints.append((staging.x, staging.y))
        return waypoints

    def _home_waypoints(self, target: TruthTarget, home: Pose2D) -> list[tuple[float, float, float]]:
        if self._runtime.home_pose is not None:
            home_pose = self._runtime.home_pose
        else:
            home_pose = home
        forward = abs(self._param_float("initial_forward_clearance_m", 2.0))
        safe_x = home_pose.x + math.cos(home_pose.yaw) * forward
        safe_y = home_pose.y + math.sin(home_pose.yaw) * forward
        return [(safe_x, safe_y, 0.0), (home.x, home.y, home.yaw)]

    def _wall_staging_point(self, target: TruthTarget) -> Pose2D:
        home = self._runtime.home_pose or self._latest_base or Pose2D()
        approach = abs(self._param_float("approach_distance_m", 1.05))
        direction_x = 1.0 if home.x >= target.x else -1.0
        return Pose2D(target.x + direction_x * approach, target.y, 0.0)

    def _target_align_y(self, target: TruthTarget) -> float:
        return float(target.align_y) if target.align_y is not None else target.y

    def _target_suction_command(self, target: TruthTarget) -> tuple[dict[str, str], dict[str, Any]]:
        side = target.suction_side if target.suction_side in ("left", "right") else "left"
        other = "right" if side == "left" else "left"
        return {
            side: "close",
            other: "open",
        }, {
            side: target.path,
            "snap": True,
            "max_distance": abs(self._param_float("contact_distance_m", 0.30)),
        }

    def _drive_to_xy(self, x: float, y: float, *, final_yaw: Optional[float] = None, tolerance: Optional[float] = None) -> tuple[dict[str, Any], bool]:
        base = self._latest_base
        if base is None:
            return {"base": {"linear": 0.0, "yaw": 0.0}}, False
        tol = self._param_float("xy_tolerance_m", 0.12) if tolerance is None else float(tolerance)
        dx = float(x) - base.x
        dy = float(y) - base.y
        dist = math.hypot(dx, dy)
        yaw_cmd = 0.0
        linear_cmd = 0.0

        if dist > tol:
            desired_yaw = math.atan2(dy, dx)
            yaw_error = _angle_delta(desired_yaw, base.yaw)
            yaw_cmd = _clip(yaw_error * 2.0, -self._param_float("max_yaw_cmd", 1.0), self._param_float("max_yaw_cmd", 1.0))
            linear_cmd = _clip(dist * 0.8, 0.0, self._param_float("max_linear_cmd", 0.35))
            if abs(yaw_error) > 0.75:
                linear_cmd = 0.0
            elif abs(yaw_error) > 0.35:
                linear_cmd *= 0.35
            return {"base": {"linear": linear_cmd, "yaw": yaw_cmd}}, False

        if final_yaw is not None:
            yaw_error = _angle_delta(float(final_yaw), base.yaw)
            if abs(yaw_error) > self._param_float("yaw_tolerance_rad", 0.12):
                yaw_cmd = _clip(yaw_error * 2.0, -self._param_float("max_yaw_cmd", 1.0), self._param_float("max_yaw_cmd", 1.0))
                return {"base": {"linear": 0.0, "yaw": yaw_cmd}}, False
        return {"base": {"linear": 0.0, "yaw": 0.0}}, True

    def _face_xy(self, x: float, y: float) -> tuple[dict[str, Any], bool]:
        base = self._latest_base
        if base is None:
            return {"base": {"linear": 0.0, "yaw": 0.0}}, False
        desired_yaw = math.atan2(float(y) - base.y, float(x) - base.x)
        yaw_error = _angle_delta(desired_yaw, base.yaw)
        if abs(yaw_error) <= self._param_float("yaw_tolerance_rad", 0.12):
            return {"base": {"linear": 0.0, "yaw": 0.0}}, True
        yaw_cmd = _clip(yaw_error * 2.0, -self._param_float("max_yaw_cmd", 1.0), self._param_float("max_yaw_cmd", 1.0))
        return {"base": {"linear": 0.0, "yaw": yaw_cmd}}, False

    def _param_float(self, name: str, default: float) -> float:
        return _finite(self.get_parameter(name).value, default)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlfaTruthDemoStateMachineNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
