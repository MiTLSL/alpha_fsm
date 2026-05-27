from __future__ import annotations

from .geometry import normalized_pose, offset_pose_along_local_x


class GraspTargetBuilderMixin:
    def _stage_targets(self, pair, state: str) -> list[tuple[str, object]]:
        left_active, right_active = self._active_arms(pair)
        targets = []
        if left_active:
            targets.append((self._left_tip, self._target_pose_for_arm(pair, "left", state)))
        if right_active:
            targets.append((self._right_tip, self._target_pose_for_arm(pair, "right", state)))
        return [(link, pose) for link, pose in targets if pose is not None]

    def _target_pose_for_arm(self, pair, arm: str, state: str):
        if state in ("PLAN_PREGRASP", "MOVE_TO_PREGRASP"):
            return offset_pose_along_local_x(self._contact_pose(pair, arm), self._pregrasp_offset_x, self._planning_frame)
        if state == "APPROACH_AND_CONTACT":
            return self._contact_pose(pair, arm)
        if state in ("PLAN_EXTRACT", "EXECUTE_EXTRACT"):
            return offset_pose_along_local_x(self._contact_pose(pair, arm), self._extract_offset_x, self._planning_frame)
        if state in ("PLAN_CARRY", "EXECUTE_CARRY"):
            return self._place_pose(pair, arm, retreat=False)
        if state == "RETREAT_SAFE":
            return self._place_pose(pair, arm, retreat=True)
        return None

    def _contact_pose(self, pair, arm: str):
        if arm == "left":
            box_pose = normalized_pose(pair.left_box_pose_robot, self._planning_frame)
            box_size = pair.left_box_size
        else:
            box_pose = normalized_pose(pair.right_box_pose_robot, self._planning_frame)
            box_size = pair.right_box_size
        if not self._input_pose_represents_box_center:
            return offset_pose_along_local_x(box_pose, self._contact_standoff_x, self._planning_frame)
        half_depth = self._box_depth(box_size) * 0.5
        return offset_pose_along_local_x(box_pose, half_depth + self._contact_standoff_x, self._planning_frame)

    def _place_pose(self, pair, arm: str, retreat: bool):
        pose = normalized_pose(pair.fixed_place_pose_robot, self._planning_frame)
        if arm == "left":
            pose.pose.position.y += abs(self._place_y_separation) * 0.5
        else:
            pose.pose.position.y -= abs(self._place_y_separation) * 0.5
        if retreat:
            pose = offset_pose_along_local_x(pose, self._retreat_offset_x, self._planning_frame)
        return pose

    def _box_depth(self, size) -> float:
        value = float(getattr(size, "x", 0.0))
        if value > 1e-6:
            return value
        return float(self.config.get("business.box_size.length", 0.4))

    def _dimension_or_default(self, size, attr: str, fallback_key: str) -> float:
        value = float(getattr(size, attr, 0.0))
        if value > 1e-6:
            return value
        return float(self.config.get(fallback_key, 0.4))
