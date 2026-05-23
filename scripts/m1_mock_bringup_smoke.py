#!/usr/bin/env python3
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "fsm_ws"

READY_MARKERS = (
    "safety_monitor_node skeleton ready",
    "task_manager_node skeleton ready",
    "wall_destacking_strategy_node skeleton ready",
    "mock_safety_button ready",
    "mock_perception_adapter_node ready",
    "mock_navigation_manager_node ready",
    "mock_pair_grasp_execution_node ready",
    "mock_vacuum_io_node ready",
)


def main() -> int:
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = "/tmp/sevnova_fsm_ros_log"
    env["PATH"] = "/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 launch fsm_config bringup_with_mock.launch.py"
    )
    proc = subprocess.Popen(
        ["/bin/bash", "-lc", command],
        cwd=WS,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    output: list[str] = []
    deadline = time.monotonic() + 15.0
    try:
        while time.monotonic() < deadline:
            line = proc.stdout.readline() if proc.stdout else ""
            if line:
                output.append(line)
                combined = "".join(output)
                if all(marker in combined for marker in READY_MARKERS):
                    print("M1 mock bringup smoke passed")
                    return 0
            if proc.poll() is not None:
                break
        print("M1 mock bringup smoke failed", file=sys.stderr)
        print("".join(output[-200:]), file=sys.stderr)
        return 1
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)


if __name__ == "__main__":
    raise SystemExit(main())
