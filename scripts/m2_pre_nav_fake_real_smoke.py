#!/usr/bin/env python3
from __future__ import annotations

import os
import selectors
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "fsm_ws"


def _ensure_ros_python() -> None:
    try:
        import rclpy  # noqa: F401
        import fsm_core  # noqa: F401
    except ImportError:
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = "/tmp/sevnova_fsm_ros_log"
        env["PATH"] = "/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin"
        command = (
            "source /opt/ros/humble/setup.bash && "
            f"source {WS}/install/setup.bash && "
            f"/usr/bin/python3 {Path(__file__).resolve()} --ros-python"
        )
        raise SystemExit(subprocess.call(["/bin/bash", "-lc", command], cwd=ROOT, env=env))


def _start_navigation_manager() -> subprocess.Popen:
    env = os.environ.copy()
    env["ROS_LOG_DIR"] = "/tmp/sevnova_fsm_ros_log"
    env["PATH"] = "/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin"
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 run navigation_manager navigation_manager_node"
    )
    return subprocess.Popen(
        ["/bin/bash", "-lc", command],
        cwd=WS,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )


class NavFakeRealHarness:
    def __init__(self):
        import rclpy
        from fsm_msgs.action import NavigateToPose
        from fsm_msgs.msg import BoxDetection, BoxDetectionArray
        from fsm_msgs.srv import BaseRecoveryCommand
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose
        from nav2_msgs.srv import ClearEntireCostmap
        from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
        from rclpy.node import Node
        from std_msgs.msg import Bool
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        self.NavigateToPose = NavigateToPose
        self.BoxDetection = BoxDetection
        self.BoxDetectionArray = BoxDetectionArray
        self.BaseRecoveryCommand = BaseRecoveryCommand
        self.Nav2NavigateToPose = Nav2NavigateToPose
        self.PoseStamped = PoseStamped
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.Trigger = Trigger
        self.ClearEntireCostmap = ClearEntireCostmap
        self.Bool = Bool
        self.GoalResponse = GoalResponse
        self.CancelResponse = CancelResponse
        self.node = Node("m2_pre_nav_fake_real_harness")

        self.nav_behavior = "succeed"
        self.lifecycle_active = True
        self.amcl_covariance = 0.01
        self.chassis_reset_success = True
        self.chassis_enable_success = True
        self.last_estop = None

        self.nav2_server = ActionServer(
            self.node,
            Nav2NavigateToPose,
            "/nav2/navigate_to_pose",
            self._execute_nav2,
            goal_callback=self._on_nav2_goal,
            cancel_callback=self._on_nav2_cancel,
        )
        self.node.create_service(Trigger, "/lifecycle_manager_navigation/is_active", self._handle_lifecycle)
        self.node.create_service(Trigger, "/lifecycle_manager_localization/is_active", self._handle_lifecycle)
        self.node.create_service(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap", self._handle_clear_costmap)
        self.node.create_service(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap", self._handle_clear_costmap)
        self.node.create_service(Trigger, "/chassis_node/reset_fault", self._handle_reset)
        self.node.create_service(Trigger, "/chassis_node/enable", self._handle_enable)
        self.node.create_subscription(Bool, "/estop", self._on_estop, 10)
        self.amcl_pub = self.node.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
        self.detection_pub = self.node.create_publisher(BoxDetectionArray, "/perception/box_detections", 10)
        self.amcl_timer = self.node.create_timer(0.05, self.publish_amcl)
        self.detection_timer = self.node.create_timer(0.05, self.publish_detection)
        self.nav_client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.recovery_client = self.node.create_client(BaseRecoveryCommand, "/nav/base_recovery")

    def _on_nav2_goal(self, goal_request):
        del goal_request
        if self.nav_behavior == "reject":
            return self.GoalResponse.REJECT
        return self.GoalResponse.ACCEPT

    def _on_nav2_cancel(self, goal_handle):
        del goal_handle
        return self.CancelResponse.ACCEPT

    async def _execute_nav2(self, goal_handle):
        feedback = self.Nav2NavigateToPose.Feedback()
        feedback.current_pose = goal_handle.request.pose
        feedback.estimated_time_remaining.sec = 1

        if self.nav_behavior == "timeout":
            while not goal_handle.is_cancel_requested:
                feedback.distance_remaining = 1.0
                goal_handle.publish_feedback(feedback)
                await self._sleep(0.05)
            goal_handle.canceled()
            return self.Nav2NavigateToPose.Result()

        for distance in (0.3, 0.1, 0.0):
            feedback.distance_remaining = float(distance)
            goal_handle.publish_feedback(feedback)
            await self._sleep(0.05)

        if self.nav_behavior == "abort":
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return self.Nav2NavigateToPose.Result()

    async def _sleep(self, duration_sec: float) -> None:
        from rclpy.task import Future

        future = Future()

        def wake():
            timer.cancel()
            if not future.done():
                future.set_result(None)

        timer = self.node.create_timer(float(duration_sec), wake)
        await future

    def _handle_lifecycle(self, request, response):
        del request
        response.success = bool(self.lifecycle_active)
        response.message = "active" if response.success else "inactive"
        return response

    def _handle_clear_costmap(self, request, response):
        del request
        return response

    def _handle_reset(self, request, response):
        del request
        response.success = bool(self.chassis_reset_success)
        response.message = "reset ok" if response.success else "reset failed"
        return response

    def _handle_enable(self, request, response):
        del request
        response.success = bool(self.chassis_enable_success)
        response.message = "enable ok" if response.success else "enable failed"
        return response

    def _on_estop(self, msg):
        self.last_estop = bool(msg.data)

    def publish_amcl(self) -> None:
        msg = self.PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = float(self.amcl_covariance)
        msg.pose.covariance[7] = float(self.amcl_covariance)
        msg.pose.covariance[35] = float(self.amcl_covariance)
        self.amcl_pub.publish(msg)

    def publish_detection(self) -> None:
        msg = self.BoxDetectionArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.frame_seq = 1
        det = self.BoxDetection()
        det.header = msg.header
        det.detection_id = "align_box"
        det.pose.header = msg.header
        det.pose.pose.position.x = 0.60
        det.pose.pose.position.y = 0.0
        det.pose.pose.position.z = 0.8
        det.pose.pose.orientation.w = 1.0
        det.confidence = 0.95
        det.pose_valid = True
        msg.detections.append(det)
        self.detection_pub.publish(msg)

    def spin_until(self, predicate, timeout_sec: float, label: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"timeout waiting for {label}")

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def call_nav_goal(self, timeout_sec: float = 2.0, fine_align: bool = False):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("navigation_manager /navigate_to_pose unavailable")
        goal = self.NavigateToPose.Goal()
        goal.goal_type = "OBSERVATION"
        goal.target_pose = self.make_pose(1.0, 0.0, 0.0)
        goal.timeout_sec = float(timeout_sec)
        goal.require_fine_alignment = bool(fine_align)
        goal.desired_distance_to_wall = 0.60
        goal.desired_yaw_to_wall = 0.0
        goal.desired_lateral_offset = 0.0
        send_future = self.nav_client.send_goal_async(goal)
        self.rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("navigation_manager rejected FSM goal")
        result_future = handle.get_result_async()
        self.rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=max(timeout_sec + 5.0, 6.0))
        if not result_future.done():
            raise RuntimeError("navigation_manager result timeout")
        return result_future.result().result

    def call_recovery(self, command: int):
        if not self.recovery_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/nav/base_recovery unavailable")
        request = self.BaseRecoveryCommand.Request()
        request.command = int(command)
        future = self.recovery_client.call_async(request)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError("base recovery response timeout")
        return future.result()

    def make_pose(self, x: float, y: float, z: float):
        pose = self.PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def destroy(self):
        self.node.destroy_node()


def _drain_output(selector: selectors.DefaultSelector, output: list[str]) -> None:
    for key, _ in selector.select(timeout=0.01):
        line = key.fileobj.readline()
        if line:
            output.append(line)


def _run() -> int:
    _ensure_ros_python()
    import rclpy
    from fsm_core.error_code import ErrorCode

    proc = _start_navigation_manager()
    output: list[str] = []
    selector = selectors.DefaultSelector()
    if proc.stdout:
        selector.register(proc.stdout, selectors.EVENT_READ)

    rclpy.init()
    harness = NavFakeRealHarness()
    try:
        harness.spin_until(lambda: harness.nav_client.wait_for_server(timeout_sec=0.05), 15.0, "navigation action server")
        harness.spin_until(lambda: harness.recovery_client.wait_for_service(timeout_sec=0.05), 5.0, "base recovery service")
        harness.spin_for(0.5)

        harness.nav_behavior = "succeed"
        harness.lifecycle_active = True
        harness.amcl_covariance = 0.01
        result = harness.call_nav_goal(timeout_sec=2.0, fine_align=True)
        if not result.success or result.error_code != 0 or not result.workpose_valid:
            raise AssertionError(f"expected nav success, got success={result.success} code={result.error_code}")

        harness.nav_behavior = "reject"
        result = harness.call_nav_goal(timeout_sec=2.0)
        if result.success or result.error_code != int(ErrorCode.E_NAV_GOAL_REJECTED):
            raise AssertionError(f"expected rejected={int(ErrorCode.E_NAV_GOAL_REJECTED)}, got {result.error_code}")

        harness.nav_behavior = "timeout"
        result = harness.call_nav_goal(timeout_sec=0.35)
        if result.success or result.error_code != int(ErrorCode.E_NAV_GOAL_TIMEOUT):
            raise AssertionError(f"expected timeout={int(ErrorCode.E_NAV_GOAL_TIMEOUT)}, got {result.error_code}")

        harness.nav_behavior = "succeed"
        harness.lifecycle_active = False
        result = harness.call_nav_goal(timeout_sec=1.0)
        if result.success or result.error_code != int(ErrorCode.E_NAV_LIFECYCLE_NOT_ACTIVE):
            raise AssertionError(f"expected lifecycle inactive={int(ErrorCode.E_NAV_LIFECYCLE_NOT_ACTIVE)}, got {result.error_code}")

        harness.lifecycle_active = True
        harness.amcl_covariance = 10.0
        harness.spin_for(0.2)
        result = harness.call_nav_goal(timeout_sec=1.0)
        if result.success or result.error_code != int(ErrorCode.E_NAV_LOCALIZATION_LOST):
            raise AssertionError(f"expected localization lost={int(ErrorCode.E_NAV_LOCALIZATION_LOST)}, got {result.error_code}")

        harness.amcl_covariance = 0.01
        command_type = harness.BaseRecoveryCommand.Request
        release = harness.call_recovery(command_type.RELEASE_ESTOP)
        reset = harness.call_recovery(command_type.RESET_FAULT)
        enable = harness.call_recovery(command_type.ENABLE_CHASSIS)
        if not (release.success and reset.success and enable.success):
            raise AssertionError(
                "base recovery success chain failed: "
                f"release=({release.success},{release.error_code},{release.message}) "
                f"reset=({reset.success},{reset.error_code},{reset.message}) "
                f"enable=({enable.success},{enable.error_code},{enable.message})"
            )
        if harness.last_estop is not False:
            raise AssertionError("release estop did not publish false")

        harness.chassis_reset_success = False
        reset_fail = harness.call_recovery(command_type.RESET_FAULT)
        if reset_fail.success or reset_fail.error_code != int(ErrorCode.E_CHASSIS_FAULT_RESET_FAIL):
            raise AssertionError(f"expected chassis reset fail={int(ErrorCode.E_CHASSIS_FAULT_RESET_FAIL)}, got {reset_fail.error_code}")

        print("M2 navigation fake-real smoke passed")
        return 0
    except Exception as exc:
        _drain_output(selector, output)
        print(f"M2 navigation fake-real smoke failed: {exc}", file=sys.stderr)
        if output:
            print("".join(output[-160:]), file=sys.stderr)
        return 1
    finally:
        harness.destroy()
        rclpy.shutdown()
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)


if __name__ == "__main__":
    if "--ros-python" in sys.argv:
        sys.argv.remove("--ros-python")
    raise SystemExit(_run())
