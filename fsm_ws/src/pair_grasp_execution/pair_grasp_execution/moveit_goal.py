from __future__ import annotations

from .geometry import bounded_scaling, normalized_pose


class MoveItGoalBuilderMixin:
    def _fill_motion_plan_request(self, request, targets: list[tuple[str, object]]) -> None:
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
        from shape_msgs.msg import SolidPrimitive

        request.workspace_parameters.header.frame_id = self._planning_frame
        request.workspace_parameters.min_corner.x = float(self.config.get("business.pair_grasp_execution.planning_workspace.x_min", -1.0))
        request.workspace_parameters.min_corner.y = float(self.config.get("business.pair_grasp_execution.planning_workspace.y_min", -1.0))
        request.workspace_parameters.min_corner.z = float(self.config.get("business.pair_grasp_execution.planning_workspace.z_min", 0.0))
        request.workspace_parameters.max_corner.x = float(self.config.get("business.pair_grasp_execution.planning_workspace.x_max", 1.5))
        request.workspace_parameters.max_corner.y = float(self.config.get("business.pair_grasp_execution.planning_workspace.y_max", 1.0))
        request.workspace_parameters.max_corner.z = float(self.config.get("business.pair_grasp_execution.planning_workspace.z_max", 2.0))
        request.start_state.is_diff = True
        request.num_planning_attempts = max(int(self._moveit_num_planning_attempts), 1)
        request.allowed_planning_time = max(float(self._moveit_allowed_planning_time_sec), 0.1)
        request.max_velocity_scaling_factor = bounded_scaling(self._moveit_velocity_scaling)
        request.max_acceleration_scaling_factor = bounded_scaling(self._moveit_acceleration_scaling)

        constraints = Constraints()
        constraints.name = "pair_grasp_tcp_targets"
        stamp = self.get_clock().now().to_msg()
        position_box_size = max(float(self._position_tolerance) * 2.0, 0.001)
        for link_name, pose_stamped in targets:
            target = normalized_pose(pose_stamped, self._planning_frame)

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [position_box_size, position_box_size, position_box_size]
            region_pose = Pose()
            region_pose.position = target.pose.position
            region_pose.orientation.w = 1.0

            position = PositionConstraint()
            position.header.stamp = stamp
            position.header.frame_id = target.header.frame_id or self._planning_frame
            position.link_name = str(link_name)
            position.constraint_region.primitives.append(primitive)
            position.constraint_region.primitive_poses.append(region_pose)
            position.weight = 1.0
            constraints.position_constraints.append(position)

            orientation = OrientationConstraint()
            orientation.header.stamp = stamp
            orientation.header.frame_id = target.header.frame_id or self._planning_frame
            orientation.link_name = str(link_name)
            orientation.orientation = target.pose.orientation
            orientation.absolute_x_axis_tolerance = max(float(self._orientation_tolerance), 1e-4)
            orientation.absolute_y_axis_tolerance = max(float(self._orientation_tolerance), 1e-4)
            orientation.absolute_z_axis_tolerance = max(float(self._orientation_tolerance), 1e-4)
            orientation.parameterization = OrientationConstraint.ROTATION_VECTOR
            orientation.weight = 1.0
            constraints.orientation_constraints.append(orientation)

        request.goal_constraints.append(constraints)
