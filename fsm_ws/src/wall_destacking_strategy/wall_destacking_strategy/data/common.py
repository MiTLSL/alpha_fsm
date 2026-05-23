from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TimeData:
    sec: int = 0
    nanosec: int = 0

    @classmethod
    def from_msg(cls, msg) -> "TimeData":
        return cls(sec=int(getattr(msg, "sec", 0)), nanosec=int(getattr(msg, "nanosec", 0)))

    def fill_msg(self, msg):
        msg.sec = int(self.sec)
        msg.nanosec = int(self.nanosec)
        return msg


@dataclass(frozen=True)
class HeaderData:
    frame_id: str = ""
    stamp: TimeData = TimeData()

    @classmethod
    def from_msg(cls, msg) -> "HeaderData":
        return cls(frame_id=getattr(msg, "frame_id", ""), stamp=TimeData.from_msg(getattr(msg, "stamp", None)))

    def fill_msg(self, msg):
        msg.frame_id = self.frame_id
        self.stamp.fill_msg(msg.stamp)
        return msg


@dataclass(frozen=True)
class Vector3Data:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_msg(cls, msg) -> "Vector3Data":
        return cls(float(getattr(msg, "x", 0.0)), float(getattr(msg, "y", 0.0)), float(getattr(msg, "z", 0.0)))

    def fill_msg(self, msg):
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.z = float(self.z)
        return msg

    def is_zero(self) -> bool:
        return self.x == 0.0 and self.y == 0.0 and self.z == 0.0


@dataclass(frozen=True)
class QuaternionData:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0

    @classmethod
    def from_msg(cls, msg) -> "QuaternionData":
        return cls(
            float(getattr(msg, "x", 0.0)),
            float(getattr(msg, "y", 0.0)),
            float(getattr(msg, "z", 0.0)),
            float(getattr(msg, "w", 1.0)),
        )

    def fill_msg(self, msg):
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.z = float(self.z)
        msg.w = float(self.w)
        return msg


@dataclass(frozen=True)
class PoseData:
    header: HeaderData = HeaderData()
    position: Vector3Data = Vector3Data()
    orientation: QuaternionData = QuaternionData()

    @classmethod
    def zero(cls, frame_id: str = "base_link") -> "PoseData":
        return cls(header=HeaderData(frame_id=frame_id), position=Vector3Data(), orientation=QuaternionData())

    @classmethod
    def from_msg(cls, msg) -> "PoseData":
        return cls(
            header=HeaderData.from_msg(getattr(msg, "header", None)),
            position=Vector3Data.from_msg(getattr(getattr(msg, "pose", None), "position", None)),
            orientation=QuaternionData.from_msg(getattr(getattr(msg, "pose", None), "orientation", None)),
        )

    def to_msg(self):
        from geometry_msgs.msg import PoseStamped

        msg = PoseStamped()
        self.header.fill_msg(msg.header)
        self.position.fill_msg(msg.pose.position)
        self.orientation.fill_msg(msg.pose.orientation)
        return msg

    def is_zero(self) -> bool:
        return self.position.is_zero() and self.orientation == QuaternionData()
