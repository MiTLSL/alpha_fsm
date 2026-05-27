from __future__ import annotations


class PlanningSceneMixin:
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
            attached.touch_links = sorted({str(link_name), *[str(item) for item in self._attached_box_touch_links]})
            attached.weight = float(self._attached_box_weight_kg)
        else:
            attached.object.operation = attached.object.REMOVE
        return attached

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

    def _attached_object_id(self, pair, arm: str) -> str:
        slot_id = pair.left_slot_id if arm == "left" else pair.right_slot_id
        return f"{pair.pair_id}_{slot_id or arm}_box"
