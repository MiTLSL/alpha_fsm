from __future__ import annotations

from dataclasses import dataclass

from fsm_core.constants import GraspMode

from .common import PoseData, Vector3Data


@dataclass(frozen=True)
class GraspPair:
    pair_id: str
    task_id: str
    wall_index: int
    phase: int
    left_slot_id: str = ""
    right_slot_id: str = ""
    left_box_pose_robot: PoseData = PoseData.zero()
    right_box_pose_robot: PoseData = PoseData.zero()
    left_box_size: Vector3Data = Vector3Data()
    right_box_size: Vector3Data = Vector3Data()
    fixed_place_pose_robot: PoseData = PoseData.zero()
    grasp_mode: int = GraspMode.DUAL

    def __post_init__(self) -> None:
        if not self.pair_id:
            raise ValueError("pair_id must not be empty")
        if self.grasp_mode == GraspMode.DUAL:
            if not self.left_slot_id or not self.right_slot_id:
                raise ValueError("dual grasp requires both slot ids")
            if self.left_box_pose_robot.position.y <= self.right_box_pose_robot.position.y:
                raise ValueError("left box y must be greater than right box y")
        elif self.grasp_mode == GraspMode.LEFT_ONLY:
            if self.right_slot_id or not self.right_box_pose_robot.is_zero() or not self.right_box_size.is_zero():
                raise ValueError("LEFT_ONLY requires right slot, pose, and size to be empty")
        elif self.grasp_mode == GraspMode.RIGHT_ONLY:
            if self.left_slot_id or not self.left_box_pose_robot.is_zero() or not self.left_box_size.is_zero():
                raise ValueError("RIGHT_ONLY requires left slot, pose, and size to be empty")
        else:
            raise ValueError(f"unsupported grasp_mode: {self.grasp_mode}")

    @classmethod
    def from_msg(cls, msg) -> "GraspPair":
        return cls(
            pair_id=msg.pair_id,
            task_id=msg.task_id,
            wall_index=int(msg.wall_index),
            phase=int(msg.phase),
            left_slot_id=msg.left_slot_id,
            right_slot_id=msg.right_slot_id,
            left_box_pose_robot=PoseData.from_msg(msg.left_box_pose_robot),
            right_box_pose_robot=PoseData.from_msg(msg.right_box_pose_robot),
            left_box_size=Vector3Data.from_msg(msg.left_box_size),
            right_box_size=Vector3Data.from_msg(msg.right_box_size),
            fixed_place_pose_robot=PoseData.from_msg(msg.fixed_place_pose_robot),
            grasp_mode=int(msg.grasp_mode),
        )

    def to_msg(self):
        from fsm_msgs.msg import GraspPair as GraspPairMsg

        msg = GraspPairMsg()
        msg.pair_id = self.pair_id
        msg.task_id = self.task_id
        msg.wall_index = int(self.wall_index)
        msg.phase = int(self.phase)
        msg.left_slot_id = self.left_slot_id
        msg.right_slot_id = self.right_slot_id
        msg.left_box_pose_robot = self.left_box_pose_robot.to_msg()
        msg.right_box_pose_robot = self.right_box_pose_robot.to_msg()
        self.left_box_size.fill_msg(msg.left_box_size)
        self.right_box_size.fill_msg(msg.right_box_size)
        msg.fixed_place_pose_robot = self.fixed_place_pose_robot.to_msg()
        msg.grasp_mode = int(self.grasp_mode)
        return msg
