from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class WaitResult:
    available: bool
    message: str = ""


def wait_for_service(client: Any, timeout_sec: float) -> WaitResult:
    if client is None:
        return WaitResult(False, "client is None")
    available = bool(client.wait_for_service(timeout_sec=timeout_sec))
    return WaitResult(available, "" if available else "service timeout")


def future_done_result(future: Any) -> tuple[bool, Any]:
    if future is None or not future.done():
        return False, None
    return True, future.result()


def make_qos_profile(reliability: str, durability: str, depth: int):
    try:
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        raise RuntimeError("rclpy is required to build ROS2 QoS profiles") from exc

    reliability_policy = ReliabilityPolicy.RELIABLE if reliability == "RELIABLE" else ReliabilityPolicy.BEST_EFFORT
    durability_policy = DurabilityPolicy.TRANSIENT_LOCAL if durability == "TRANSIENT_LOCAL" else DurabilityPolicy.VOLATILE
    return QoSProfile(depth=depth, reliability=reliability_policy, durability=durability_policy)


def unwrap_ros_parameters(data: dict[str, Any]) -> dict[str, Any]:
    if "/**" in data and isinstance(data["/**"], dict):
        return data["/**"].get("ros__parameters", {})
    if "ros__parameters" in data:
        return data.get("ros__parameters", {})
    return data


def load_yaml(path: str | Path) -> dict[str, Any]:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return unwrap_ros_parameters(data)


def flatten_parameters(data: dict[str, Any], prefix: str = "") -> dict[str, Any]:
    flattened: dict[str, Any] = {}
    for key, value in data.items():
        name = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            flattened.update(flatten_parameters(value, name))
        else:
            flattened[name] = value
    return flattened


def load_common_parameter_dict() -> dict[str, Any]:
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:
        raise RuntimeError("ament_index_python is required to load fsm_config") from exc

    config_dir = Path(get_package_share_directory("fsm_config")) / "params"
    merged: dict[str, Any] = {}
    for filename in ("business.yaml", "fsm.yaml", "interfaces.yaml", "error_codes.yaml", "logging.yaml"):
        merged.update(load_yaml(config_dir / filename))
    return merged


def _is_uninitialized_parameter_error(exc: Exception) -> bool:
    try:
        from rclpy.exceptions import ParameterUninitializedException
    except ImportError:
        return exc.__class__.__name__ == "ParameterUninitializedException"
    return isinstance(exc, ParameterUninitializedException)


def declare_parameters_from_dict(node: Any, data: dict[str, Any]) -> dict[str, Any]:
    declared: dict[str, Any] = {}
    for name, default in flatten_parameters(data).items():
        if not node.has_parameter(name):
            node.declare_parameter(name, default)
        try:
            declared[name] = node.get_parameter(name).value
        except Exception as exc:
            if not _is_uninitialized_parameter_error(exc):
                raise
            # ROS 2 launch YAML cannot infer the element type of empty arrays,
            # so it may create an already-declared but uninitialized parameter.
            # Keep the node config usable by falling back to the YAML default.
            declared[name] = default
    return declared


def declare_common_parameters(node: Any) -> dict[str, Any]:
    return declare_parameters_from_dict(node, load_common_parameter_dict())


def get_topic_name(node: Any, key: str, default: str) -> str:
    name = f"interfaces.topics.{key}"
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return str(node.get_parameter(name).value)


def get_service_name(node: Any, key: str, default: str) -> str:
    name = f"interfaces.services.{key}"
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return str(node.get_parameter(name).value)


def get_action_name(node: Any, key: str, default: str) -> str:
    name = f"interfaces.actions.{key}"
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return str(node.get_parameter(name).value)


def make_state_snapshot_msg(snapshot: dict[str, Any]):
    from fsm_msgs.msg import FsmStateSnapshot

    msg = FsmStateSnapshot()
    msg.node_name = snapshot.get("node_name", "")
    msg.fsm_name = snapshot.get("fsm_name", "")
    msg.current_state = snapshot.get("current_state", "")
    msg.parent_fsm = snapshot.get("parent_fsm", "")
    msg.parent_state = snapshot.get("parent_state", "")
    msg.task_id = snapshot.get("task_id", "")
    msg.wall_index = int(snapshot.get("wall_index", 0))
    msg.phase = int(snapshot.get("phase", 0))
    msg.state_elapsed_sec = float(snapshot.get("state_elapsed_sec", 0.0))
    msg.retry_count = int(snapshot.get("retry_count", 0))
    msg.last_error_code = int(snapshot.get("last_error_code", 0))
    msg.extra_json = snapshot.get("extra_json", "{}")
    return msg
