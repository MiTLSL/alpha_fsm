from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable

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


def _finite(value: float, default: float = 0.0) -> float:
    return value if math.isfinite(value) else default


def _base_command(*, linear: float = 0.0, yaw: float = 0.0) -> dict[str, float]:
    return {"linear": float(_finite(linear)), "yaw": float(_finite(yaw))}


def _arm_command(
    *,
    left_joint2: float = 0.0,
    left_joint3: float = 0.0,
    right_joint2: float = 0.0,
    right_joint3: float = 0.0,
    joint5: float = 0.0,
) -> dict[str, dict[str, float]]:
    left = {f"joint{i}": 0.0 for i in range(1, 7)}
    right = {f"joint{i}": 0.0 for i in range(1, 7)}
    left.update({"joint2": float(left_joint2), "joint3": float(left_joint3), "joint5": float(joint5)})
    right.update({"joint2": float(right_joint2), "joint3": float(right_joint3), "joint5": float(joint5)})
    return {"left": left, "right": right}


@dataclass(frozen=True)
class TaskStage:
    name: str
    label: str
    duration_sec: float
    command_factory: Callable[[], dict[str, Any]]
    description: str = ""


@dataclass
class TaskRuntime:
    active: bool = False
    complete: bool = False
    started_at: float = 0.0
    stage_index: int = 0
    stage_started_at: float = 0.0
    last_stage_name: str = ""
    task_id: str = ""
    last_state_json: str = ""
    clock_seen: bool = False
    clock_seen_at: float = 0.0
    events: list[dict[str, Any]] = field(default_factory=list)


class AlfaTaskStateMachineNode(Node):
    def __init__(self) -> None:
        super().__init__("alfa_task_state_machine_node")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("wait_for_clock", True)
        self.declare_parameter("clock_timeout_sec", 20.0)
        self.declare_parameter("start_delay_sec", 1.0)
        self.declare_parameter("stage_time_scale", 1.0)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("task_id", "alfa_v7_nav_grasp_demo")
        self.declare_parameter("command_topic", "/alfa/command_json")
        self.declare_parameter("event_topic", "/alfa/fsm_event_json")
        self.declare_parameter("state_json_topic", "/alfa_task/state_json")
        self.declare_parameter("behavior_state_topic", "/behavior_state")
        self.declare_parameter("structured_state_topic", "/robot/behavior_state")
        self.declare_parameter("fsm_state_topic", "/fsm/alfa_task_state")
        self.declare_parameter("windows_state_topic", "/alfa/state_json")
        self.declare_parameter("hold_after_complete", True)

        self._runtime = TaskRuntime(task_id=str(self.get_parameter("task_id").value))
        self._node_start = time.monotonic()
        self._last_publish = 0.0
        self._stages = self._build_stages()

        self._command_pub = self.create_publisher(String, self.get_parameter("command_topic").value, 10)
        self._event_pub = self.create_publisher(String, self.get_parameter("event_topic").value, 10)
        self._state_json_pub = self.create_publisher(String, self.get_parameter("state_json_topic").value, 10)
        self._behavior_pub = self.create_publisher(String, self.get_parameter("behavior_state_topic").value, 10)
        self._structured_pub = self.create_publisher(BehaviorState, self.get_parameter("structured_state_topic").value, 10)
        self._fsm_state_pub = self.create_publisher(FsmStateSnapshot, self.get_parameter("fsm_state_topic").value, 10)

        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(String, self.get_parameter("windows_state_topic").value, self._on_windows_state, 10)

        self.create_service(Trigger, "/alfa_task/start", self._handle_start)
        self.create_service(Trigger, "/alfa_task/stop", self._handle_stop)
        self.create_service(Trigger, "/alfa_task/reset", self._handle_reset)

        rate = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "alfa_task_state_machine_node ready: "
            f"stages={len(self._stages)} auto_start={self.get_parameter('auto_start').value}"
        )

    def _bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _build_stages(self) -> list[TaskStage]:
        return [
            TaskStage("RESET_SCENE", "复位场景", 0.8, lambda: self._command(reset=True), "复位机器人、门和吸盘"),
            TaskStage("OPEN_DOORS", "打开仓库门和集装箱门", 3.5, lambda: self._command(warehouse_door="open", container_door="open")),
            TaskStage(
                "NAVIGATE_TO_CONTAINER",
                "导航到集装箱前",
                4.0,
                lambda: self._command(base=_base_command(linear=0.35), warehouse_door="open", container_door="open"),
            ),
            TaskStage(
                "ALIGN_TO_BOX_WALL",
                "贴近箱墙并微调姿态",
                1.4,
                lambda: self._command(base=_base_command(linear=0.08, yaw=0.12), warehouse_door="open", container_door="open"),
            ),
            TaskStage(
                "MOVE_TO_PREGRASP",
                "移动双臂到预抓取位",
                2.6,
                lambda: self._command(
                    updown=0.18,
                    arm=_arm_command(left_joint2=0.30, left_joint3=-0.22, right_joint2=0.30, right_joint3=0.22),
                    warehouse_door="open",
                    container_door="open",
                ),
            ),
            TaskStage(
                "APPROACH_AND_CONTACT",
                "低速前进接触箱体",
                2.0,
                lambda: self._command(
                    base=_base_command(linear=0.10),
                    arm=_arm_command(left_joint2=0.18, left_joint3=-0.12, right_joint2=0.18, right_joint3=0.12),
                    warehouse_door="open",
                    container_door="open",
                ),
            ),
            TaskStage(
                "VACUUM_GRASP",
                "闭合左右吸盘吸附箱体",
                1.4,
                lambda: self._command(
                    suction={"left": "close", "right": "close"},
                    arm=_arm_command(left_joint2=0.05, right_joint2=0.05),
                    warehouse_door="open",
                    container_door="open",
                ),
            ),
            TaskStage(
                "EXTRACT_BOX",
                "后退抽箱",
                3.0,
                lambda: self._command(
                    base=_base_command(linear=-0.18),
                    suction={"left": "close", "right": "close"},
                    updown=-0.04,
                    warehouse_door="open",
                    container_door="open",
                ),
            ),
            TaskStage(
                "CARRY_TO_PLACE",
                "搬运到放置区",
                3.6,
                lambda: self._command(
                    base=_base_command(linear=-0.16, yaw=-0.08),
                    suction={"left": "close", "right": "close"},
                    updown=-0.10,
                    warehouse_door="open",
                    container_door="open",
                ),
            ),
            TaskStage(
                "RELEASE_BOX",
                "释放箱体",
                1.6,
                lambda: self._command(
                    suction={"left": "open", "right": "open"},
                    updown=-0.16,
                    warehouse_door="open",
                    container_door="open",
                ),
            ),
            TaskStage(
                "RETREAT_SAFE",
                "退回安全位",
                2.0,
                lambda: self._command(base=_base_command(linear=-0.14), suction={"left": "open", "right": "open"}),
            ),
            TaskStage("COMPLETE", "任务完成并保持静止", 0.8, lambda: self._command(suction={"left": "open", "right": "open"})),
        ]

    def _command(self, **updates: Any) -> dict[str, Any]:
        command = default_alfa_command()
        for key, value in updates.items():
            command[key] = value
        return command

    def _on_clock(self, msg: Clock) -> None:
        del msg
        self._runtime.clock_seen = True
        self._runtime.clock_seen_at = time.monotonic()

    def _on_windows_state(self, msg: String) -> None:
        self._runtime.last_state_json = msg.data

    def _handle_start(self, request, response):
        del request
        if self._runtime.active:
            response.success = False
            response.message = f"task already active: {self._current_stage_name()}"
            return response
        self._start_task()
        response.success = True
        response.message = f"started {self._runtime.task_id}"
        return response

    def _handle_stop(self, request, response):
        del request
        self._stop_task("service_stop")
        response.success = True
        response.message = "stopped"
        return response

    def _handle_reset(self, request, response):
        del request
        self._runtime = TaskRuntime(task_id=str(self.get_parameter("task_id").value))
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
            self._advance_if_needed(now)
            command = self._stage_command(now)
            self._publish_command(command, self._current_stage_name())
        elif self._runtime.complete and self._bool_param("hold_after_complete"):
            self._publish_command(self._command(suction={"left": "open", "right": "open"}), "HOLD_COMPLETE")

        self._publish_state(now)

    def _should_auto_start(self, now: float) -> bool:
        if not self._bool_param("auto_start"):
            return False
        if self._runtime.active or self._runtime.complete:
            return False
        if now - self._node_start < float(self.get_parameter("start_delay_sec").value):
            return False
        if not self._bool_param("wait_for_clock"):
            return True
        if self._runtime.clock_seen:
            return True
        timeout = float(self.get_parameter("clock_timeout_sec").value)
        if timeout > 0.0 and now - self._node_start >= timeout:
            self.get_logger().warning("clock wait timed out; starting task with local time")
            return True
        return False

    def _start_task(self) -> None:
        now = time.monotonic()
        self._runtime.active = True
        self._runtime.complete = False
        self._runtime.started_at = now
        self._runtime.stage_started_at = now
        self._runtime.stage_index = 0
        self._runtime.last_stage_name = ""
        self._publish_event("task_started", task_id=self._runtime.task_id)
        self.get_logger().info(f"started task {self._runtime.task_id}")

    def _stop_task(self, reason: str) -> None:
        self._runtime.active = False
        self._runtime.complete = False
        self._publish_command(self._command(), "STOPPED")
        self._publish_event("task_stopped", reason=reason)

    def _advance_if_needed(self, now: float) -> None:
        stage = self._stages[self._runtime.stage_index]
        if stage.name != self._runtime.last_stage_name:
            self._runtime.last_stage_name = stage.name
            self._publish_event(
                "stage_entered",
                stage=stage.name,
                label=stage.label,
                index=self._runtime.stage_index,
                duration_sec=self._scaled_duration(stage),
            )
            self.get_logger().info(f"stage {self._runtime.stage_index + 1}/{len(self._stages)} {stage.name}: {stage.label}")

        if now - self._runtime.stage_started_at < self._scaled_duration(stage):
            return

        if self._runtime.stage_index + 1 >= len(self._stages):
            self._runtime.active = False
            self._runtime.complete = True
            self._publish_event("task_completed", task_id=self._runtime.task_id)
            self.get_logger().info(f"completed task {self._runtime.task_id}")
            return

        self._runtime.stage_index += 1
        self._runtime.stage_started_at = now
        self._runtime.last_stage_name = ""

    def _scaled_duration(self, stage: TaskStage) -> float:
        return max(0.05, float(stage.duration_sec) * max(0.01, float(self.get_parameter("stage_time_scale").value)))

    def _current_stage(self) -> TaskStage:
        return self._stages[min(self._runtime.stage_index, len(self._stages) - 1)]

    def _current_stage_name(self) -> str:
        if self._runtime.active or self._runtime.complete:
            return self._current_stage().name
        return "IDLE"

    def _stage_command(self, now: float) -> dict[str, Any]:
        stage = self._current_stage()
        command = stage.command_factory()
        command["meta"] = {
            "source": "alfa_task_state_machine",
            "kind": "state_machine_nav_grasp",
            "task_id": self._runtime.task_id,
            "stage": stage.name,
            "stage_label": stage.label,
            "stage_index": int(self._runtime.stage_index),
            "stage_elapsed_sec": round(now - self._runtime.stage_started_at, 3),
            "stage_duration_sec": round(self._scaled_duration(stage), 3),
        }
        return command

    def _publish_command(self, command: dict[str, Any], stage_name: str) -> None:
        msg = String()
        msg.data = command_json(command)
        self._command_pub.publish(msg)
        self._last_publish = time.monotonic()
        if stage_name == "RESET_SCENE":
            return

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
        self._runtime.events = self._runtime.events[-20:]

    def _publish_state(self, now: float) -> None:
        stage = self._current_stage()
        stage_elapsed = now - self._runtime.stage_started_at if self._runtime.stage_started_at else 0.0
        total_elapsed = now - self._runtime.started_at if self._runtime.started_at else 0.0
        data = {
            "task_id": self._runtime.task_id,
            "active": self._runtime.active,
            "complete": self._runtime.complete,
            "stage": stage.name if self._runtime.active or self._runtime.complete else "IDLE",
            "stage_label": stage.label if self._runtime.active or self._runtime.complete else "待机",
            "stage_index": self._runtime.stage_index,
            "stage_count": len(self._stages),
            "stage_elapsed_sec": round(stage_elapsed, 3),
            "task_elapsed_sec": round(total_elapsed, 3),
            "clock_seen": self._runtime.clock_seen,
            "windows_state_seen": bool(self._runtime.last_state_json),
        }

        state_json = String()
        state_json.data = json.dumps(data, ensure_ascii=False, sort_keys=True, allow_nan=False)
        self._state_json_pub.publish(state_json)

        behavior = String()
        behavior.data = state_json.data
        self._behavior_pub.publish(behavior)

        structured = BehaviorState()
        structured.header.stamp = self.get_clock().now().to_msg()
        structured.header.frame_id = "map"
        structured.system_state = "AUTO_MODE" if self._runtime.active else ("STANDBY" if self._runtime.complete else "IDLE")
        structured.task_state = data["stage"]
        structured.wall_state = "NAV_GRASP_DEMO"
        structured.active_substate = data["stage_label"]
        structured.task_id = self._runtime.task_id
        structured.last_error_code = 0
        structured.summary_json = state_json.data
        self._structured_pub.publish(structured)

        fsm = FsmStateSnapshot()
        fsm.header = structured.header
        fsm.node_name = self.get_name()
        fsm.fsm_name = "AlfaNavGraspTaskFSM"
        fsm.current_state = structured.task_state
        fsm.parent_fsm = "AlfaDemo"
        fsm.parent_state = structured.system_state
        fsm.task_id = self._runtime.task_id
        fsm.wall_index = 0
        fsm.phase = int(self._runtime.stage_index)
        fsm.state_elapsed_sec = float(stage_elapsed)
        fsm.retry_count = 0
        fsm.last_error_code = 0
        fsm.extra_json = state_json.data
        self._fsm_state_pub.publish(fsm)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlfaTaskStateMachineNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
