class GoalType:
    OBSERVATION = "OBSERVATION"
    LEFT_PHASE = "LEFT_PHASE"
    RIGHT_PHASE = "RIGHT_PHASE"
    NEXT_WALL = "NEXT_WALL"
    SAFE = "SAFE"
    RECOVERY = "RECOVERY"


class Phase:
    LEFT = 0
    RIGHT = 1


class GraspMode:
    DUAL = 0
    LEFT_ONLY = 1
    RIGHT_ONLY = 2


class SlotStatus:
    UNKNOWN = 0
    OCCUPIED = 1
    REMOVED = 2
    FAILED = 3
    BLOCKED = 4


class ResultCode:
    SUCCESS_BOTH = 0
    SUCCESS_LEFT_ONLY = 1
    SUCCESS_RIGHT_ONLY = 2
    FAILED_BOTH = 3
    ESTOP = 4
    CANCELLED = 5


class ArmResult:
    SUCCESS = 0
    FAIL = 1
    NOT_ATTEMPTED = 2


class FailedStage:
    PLAN = "PLAN"
    MOVE = "MOVE"
    VACUUM = "VACUUM"
    EXTRACT = "EXTRACT"
    CARRY = "CARRY"
    PLACE = "PLACE"
    RELEASE = "RELEASE"


class TaskCommand:
    START = "start"
    PAUSE = "pause"
    RESUME = "resume"
    CANCEL = "cancel"


class ClearErrorStage:
    NONE = 0
    ESTOP_RELEASED = 1
    ACTIONS_CANCELED = 2
    FAULT_RESET = 3
    CHASSIS_ENABLED = 4
    SELF_CHECK = 5
