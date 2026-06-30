from __future__ import annotations

import time

from .scene_geometry import (
    make_box_wall_opening_obstacles,
    make_container_obstacles,
    pair_static_object_ids,
    selected_box_object_ids,
)


class PlanningSceneMixin:
    async def _prepare_planning_scene_for_stage(self, pair, state: str) -> tuple[bool, int, str]:
        if not self._manage_world_collision_objects:
            return True, 0, ""

        obstacles = []
        obstacles.extend(make_container_obstacles(self.config.get, self._planning_frame))
        if self._stage_uses_static_wall_obstacles(state):
            obstacles.extend(make_box_wall_opening_obstacles(pair, self.config.get, self._planning_frame))
        remove_ids = []
        if self._remove_target_box_world_objects and self._stage_requires_open_target_slots(state):
            remove_ids = selected_box_object_ids(pair)
        return await self._apply_world_collision_objects(
            obstacles=obstacles,
            remove_ids=remove_ids,
            label=f"planning scene for {state}",
        )

    async def _check_robot_state_valid_for_planning(self) -> tuple[bool, int, str]:
        from fsm_core.error_code import ErrorCode
        from moveit_msgs.srv import GetStateValidity

        if self._self_collision_require_dual_arm_group and str(self._moveit_group) != str(self._self_collision_required_group_name):
            return (
                False,
                int(ErrorCode.E_PLAN_COLLISION_DETECTED),
                f"MoveIt planning group must be {self._self_collision_required_group_name} for whole-body collision checking",
            )
        if not self._self_collision_check_current_state:
            return True, 0, ""
        joint_state = getattr(self, "_last_joint_state", None)
        if joint_state is None:
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "joint_states unavailable for MoveIt state validity check"
        received_at = getattr(self, "_last_joint_state_received_monotonic", None)
        if received_at is None or (time.monotonic() - float(received_at)) > max(float(self._joint_state_max_age_sec), 0.01):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "joint_states stale for MoveIt state validity check"
        if not self._state_validity_client.wait_for_service(timeout_sec=max(float(self._state_validity_timeout_sec), 0.1)):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt check_state_validity service unavailable"

        request = GetStateValidity.Request()
        request.group_name = str(self._moveit_group)
        request.robot_state.is_diff = True
        request.robot_state.joint_state = joint_state
        done, response = await self._wait_future(
            self._state_validity_client.call_async(request),
            max(float(self._state_validity_timeout_sec), 0.1),
            "MoveIt check_state_validity",
        )
        if not done or response is None:
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt check_state_validity timeout"
        if bool(response.valid):
            return True, 0, ""
        return (
            False,
            int(ErrorCode.E_PLAN_COLLISION_DETECTED),
            f"current robot state is in collision: {self._format_state_validity_contacts(response.contacts)}",
        )

    @staticmethod
    def _format_state_validity_contacts(contacts) -> str:
        pairs = []
        for contact in list(contacts)[:5]:
            pairs.append(f"{contact.contact_body_1}<->{contact.contact_body_2}")
        return ", ".join(pairs) if pairs else "no contact details"

    async def _cleanup_pair_planning_scene(self, pair) -> None:
        if not self._manage_world_collision_objects:
            return
        owned_ids = [object_id for object_id in pair_static_object_ids(pair) if object_id in self._world_collision_object_ids]
        await self._remove_world_collision_objects_best_effort(owned_ids)

    async def _apply_attached_boxes(self, pair, attach: bool) -> tuple[bool, int, str]:
        from fsm_core.error_code import ErrorCode
        from moveit_msgs.srv import ApplyPlanningScene

        if not self._attach_box_to_planning_scene:
            return True, 0, ""
        if not self._planning_scene_client.wait_for_service(timeout_sec=max(self._moveit_wait_sec, 0.1)):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt apply_planning_scene service unavailable"

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        left_active, right_active = self._active_arms(pair)
        for arm, active, link_name in (
            ("left", left_active, self._left_tip),
            ("right", right_active, self._right_tip),
        ):
            if not active:
                continue
            request.scene.robot_state.attached_collision_objects.append(self._make_attached_object(pair, arm, link_name, attach))

        done, response = await self._wait_future(
            self._planning_scene_client.call_async(request),
            max(self._moveit_wait_sec, 0.1),
            "MoveIt apply_planning_scene",
        )
        if not done or response is None or not bool(response.success):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt apply_planning_scene failed"
        if attach:
            for attached in request.scene.robot_state.attached_collision_objects:
                self._attached_object_ids.add(str(attached.object.id))
        else:
            for attached in request.scene.robot_state.attached_collision_objects:
                self._attached_object_ids.discard(str(attached.object.id))
        return True, 0, ""

    def _make_attached_object(self, pair, arm: str, link_name: str, attach: bool):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import AttachedCollisionObject
        from shape_msgs.msg import SolidPrimitive

        attached = AttachedCollisionObject()
        attached.link_name = str(link_name)
        attached.object.id = self._attached_object_id(pair, arm)
        attached.object.header.frame_id = str(link_name)
        if attach:
            size = pair.left_box_size if arm == "left" else pair.right_box_size
            depth = self._box_depth(size)
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [
                depth,
                self._dimension_or_default(size, "y", "business.box_size.width"),
                self._dimension_or_default(size, "z", "business.box_size.height"),
            ]
            primitive_pose = Pose()
            primitive_pose.position.x = -depth * 0.5
            primitive_pose.orientation.w = 1.0
            attached.object.primitives.append(primitive)
            attached.object.primitive_poses.append(primitive_pose)
            attached.object.operation = attached.object.ADD
            touch_links = {str(link_name), *[str(item) for item in self._attached_box_touch_links]}
            if str(link_name).startswith("left_"):
                touch_links.update({"left_v5_link6", "left_v5_link5"})
            elif str(link_name).startswith("right_"):
                touch_links.update({"right_v5_link6", "right_v5_link5"})
            attached.touch_links = sorted(touch_links)
            attached.weight = float(self._attached_box_weight_kg)
        else:
            attached.object.operation = attached.object.REMOVE
        return attached

    async def _apply_world_collision_objects(self, obstacles, remove_ids: list[str], label: str) -> tuple[bool, int, str]:
        from fsm_core.error_code import ErrorCode
        from moveit_msgs.srv import ApplyPlanningScene

        if not obstacles and not remove_ids:
            return True, 0, ""
        if not self._planning_scene_client.wait_for_service(timeout_sec=max(self._moveit_wait_sec, 0.1)):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), "MoveIt apply_planning_scene service unavailable"

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        for object_id in remove_ids:
            request.scene.world.collision_objects.append(self._make_world_box_collision_object(object_id, None, None, remove=True))
        for obstacle in obstacles:
            request.scene.world.collision_objects.append(
                self._make_world_box_collision_object(obstacle.object_id, obstacle.center, obstacle.size, remove=False)
            )

        done, response = await self._wait_future(
            self._planning_scene_client.call_async(request),
            max(self._moveit_wait_sec, 0.1),
            label,
        )
        if not done or response is None or not bool(response.success):
            return False, int(ErrorCode.E_PLAN_TRAJ_FAIL), f"MoveIt apply_planning_scene failed: {label}"
        for object_id in remove_ids:
            self._world_collision_object_ids.discard(str(object_id))
        for obstacle in obstacles:
            self._world_collision_object_ids.add(str(obstacle.object_id))
        return True, 0, ""

    def _make_world_box_collision_object(self, object_id: str, center, size, remove: bool):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive

        obj = CollisionObject()
        obj.header.frame_id = self._planning_frame
        obj.id = str(object_id)
        if remove:
            obj.operation = CollisionObject.REMOVE
            return obj

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(size[0]), float(size[1]), float(size[2])]
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])
        pose.orientation.w = 1.0
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def _stage_uses_static_wall_obstacles(self, state: str) -> bool:
        return state in {
            "PLAN_PREGRASP",
            "MOVE_TO_PREGRASP",
            "APPROACH_AND_CONTACT",
            "PLAN_EXTRACT",
            "EXECUTE_EXTRACT",
        }

    def _stage_requires_open_target_slots(self, state: str) -> bool:
        return state in {
            "APPROACH_AND_CONTACT",
            "PLAN_EXTRACT",
            "EXECUTE_EXTRACT",
            "PLAN_CARRY",
            "EXECUTE_CARRY",
        }

    async def _clear_attached_boxes_best_effort(self) -> None:
        if not self._attached_object_ids or not self._attach_box_to_planning_scene:
            return
        from moveit_msgs.msg import AttachedCollisionObject
        from moveit_msgs.srv import ApplyPlanningScene

        if not self._planning_scene_client.wait_for_service(timeout_sec=0.2):
            return
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        for object_id in sorted(self._attached_object_ids):
            attached = AttachedCollisionObject()
            attached.object.id = str(object_id)
            attached.object.operation = attached.object.REMOVE
            request.scene.robot_state.attached_collision_objects.append(attached)
        done, response = await self._wait_future(self._planning_scene_client.call_async(request), 0.5, "clear attached boxes")
        if done and response is not None and bool(response.success):
            self._attached_object_ids.clear()

    async def _remove_world_collision_objects_best_effort(self, object_ids) -> None:
        object_ids = [str(object_id) for object_id in object_ids if str(object_id)]
        if not object_ids:
            return
        if not self._planning_scene_client.wait_for_service(timeout_sec=0.2):
            return
        request = self._make_remove_world_objects_request(object_ids)
        done, response = await self._wait_future(request and self._planning_scene_client.call_async(request), 0.5, "clear world collision objects")
        if done and response is not None and bool(response.success):
            for object_id in object_ids:
                self._world_collision_object_ids.discard(object_id)

    def _make_remove_world_objects_request(self, object_ids):
        from moveit_msgs.srv import ApplyPlanningScene

        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        for object_id in object_ids:
            request.scene.world.collision_objects.append(self._make_world_box_collision_object(object_id, None, None, remove=True))
        return request

    def _attached_object_id(self, pair, arm: str) -> str:
        slot_id = pair.left_slot_id if arm == "left" else pair.right_slot_id
        return f"{pair.pair_id}_{slot_id or arm}_box"
