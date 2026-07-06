from __future__ import annotations

from dataclasses import dataclass

from .common import HeaderData, PoseData, Vector3Data


@dataclass(frozen=True)
class BoxDetection:
    header: HeaderData
    detection_id: str
    pose: PoseData
    size: Vector3Data
    confidence: float
    class_label: str = "box"
    pose_valid: bool = True

    @classmethod
    def from_msg(cls, msg) -> "BoxDetection":
        return cls(
            header=HeaderData.from_msg(msg.header),
            detection_id=msg.detection_id,
            pose=PoseData.from_msg(msg.pose),
            size=Vector3Data.from_msg(msg.size),
            confidence=float(msg.confidence),
            class_label=msg.class_label,
            pose_valid=bool(msg.pose_valid),
        )

    def to_msg(self):
        from fsm_msgs.msg import BoxDetection as BoxDetectionMsg

        msg = BoxDetectionMsg()
        self.header.fill_msg(msg.header)
        msg.detection_id = self.detection_id
        msg.pose = self.pose.to_msg()
        self.size.fill_msg(msg.size)
        msg.confidence = float(self.confidence)
        msg.class_label = self.class_label
        msg.pose_valid = bool(self.pose_valid)
        return msg
