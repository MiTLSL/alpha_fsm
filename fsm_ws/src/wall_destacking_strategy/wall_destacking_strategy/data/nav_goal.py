from __future__ import annotations

from dataclasses import dataclass

from .common import PoseData


@dataclass(frozen=True)
class NavGoal:
    goal_type: str
    target_pose: PoseData
    wall_frame_pose: PoseData = PoseData.zero(frame_id="map")
    phase: int = 0
    desired_distance_to_wall: float = 0.0
    desired_yaw_to_wall: float = 0.0
    desired_lateral_offset: float = 0.0

    @classmethod
    def from_msg(cls, msg) -> "NavGoal":
        return cls(
            goal_type=msg.goal_type,
            target_pose=PoseData.from_msg(msg.target_pose),
            wall_frame_pose=PoseData.from_msg(msg.wall_frame_pose),
            phase=int(msg.phase),
            desired_distance_to_wall=float(msg.desired_distance_to_wall),
            desired_yaw_to_wall=float(msg.desired_yaw_to_wall),
            desired_lateral_offset=float(msg.desired_lateral_offset),
        )

    def to_msg(self):
        from fsm_msgs.msg import NavGoal as NavGoalMsg

        msg = NavGoalMsg()
        msg.goal_type = self.goal_type
        msg.target_pose = self.target_pose.to_msg()
        msg.wall_frame_pose = self.wall_frame_pose.to_msg()
        msg.phase = int(self.phase)
        msg.desired_distance_to_wall = float(self.desired_distance_to_wall)
        msg.desired_yaw_to_wall = float(self.desired_yaw_to_wall)
        msg.desired_lateral_offset = float(self.desired_lateral_offset)
        return msg
