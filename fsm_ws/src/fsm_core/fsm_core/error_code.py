from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class ErrorLevel(IntEnum):
    INFO = 0
    WARN = 1
    ERROR = 2
    FATAL = 3
    ESTOP = 4


class ErrorSource(IntEnum):
    SYSTEM = 0
    TASK = 1
    WALL_STRATEGY = 2
    WALL_MAPPING = 3
    PHASE_PERCEPTION = 4
    PAIR_SELECTION = 5
    NAVIGATION = 6
    PAIR_GRASP = 7
    IO_VACUUM = 8
    SAFETY = 9
    PERCEPTION_EXT = 10
    COMMUNICATION = 11
    MANUAL = 12


class RecoveryAction(IntEnum):
    NONE = 0
    RETRY_CURRENT_STATE = 1
    REPLAN = 2
    REPERCEPTION = 3
    SWITCH_TARGET = 4
    SWITCH_PHASE = 5
    MOVE_BASE = 6
    RETREAT_SAFE = 7
    RELOCALIZE = 8
    REBUILD_GRID = 9
    WAIT_MANUAL_RECOVERY = 10
    ABORT_TASK = 11
    E_STOP = 12


class ErrorCode(IntEnum):
    NO_ERROR = 0

    E_SYS_BOOT_FAIL = 1000
    E_SYS_HARDWARE_NOT_READY = 1001
    E_SYS_CONFIG_INVALID = 1002
    E_SYS_TF_TIMEOUT = 1003
    E_SYS_ACTION_CANCEL_TIMEOUT = 1004
    E_SYS_SELF_CHECK_REENTRY_FAIL = 1005
    E_SYS_MODE_TRANSITION_INVALID = 1010
    E_STATE_TIMEOUT_SYSTEM = 1090
    E_SYS_UNKNOWN = 1099

    E_TASK_VALIDATE_FAIL = 2000
    E_TASK_PRECONDITION_FAIL = 2001
    E_TASK_PAUSE_TIMEOUT = 2002
    E_TASK_CANCELLED = 2003
    E_TASK_CHILD_FAILED = 2010
    E_TASK_CHILD_TIMEOUT = 2011
    E_STATE_TIMEOUT_TASK = 2090

    E_WALL_STATE_TIMEOUT = 3000
    E_WALL_INVALID_TRANSITION = 3001
    E_WALL_PHASE_NOT_COMPLETE_BUT_NO_PAIR = 3010
    E_WALL_EMPTY_VERIFY_CONFLICT = 3020
    E_WALL_TASK_COMPLETE_TIMEOUT = 3030
    E_STATE_TIMEOUT_WALL_STRATEGY = 3090
    E_WALL_UNKNOWN = 3099

    E_MAP_GLOBAL_SCAN_FAIL = 3100
    E_MAP_GLOBAL_SCAN_TIMEOUT = 3101
    E_MAP_NO_DETECTION = 3110
    E_MAP_INSUFFICIENT_DETECTION = 3111
    E_MAP_WALL_FRAME_FAIL = 3120
    E_MAP_WALL_FRAME_LOW_CONFIDENCE = 3121
    E_MAP_GRID_INCOMPLETE = 3130
    E_MAP_GRID_BUILD_FAIL = 3131
    E_MAP_NO_NEW_WALL = 3140
    E_STATE_TIMEOUT_MAPPING = 3190

    E_PERC_LOCAL_SCAN_FAIL = 3200
    E_PERC_LOCAL_SCAN_TIMEOUT = 3201
    E_PERC_NO_LOCAL_DETECTION = 3210
    E_PERC_ASSOCIATION_FAIL = 3220
    E_PERC_TOO_MANY_FALSE_POSITIVE = 3221
    E_PERC_SLOT_POSE_OUTLIER = 3230
    E_STATE_TIMEOUT_PHASE_PERCEPTION = 3290

    E_PAIR_NO_CANDIDATE = 3300
    E_PAIR_NO_REACHABLE = 3310
    E_PAIR_DUAL_CONFLICT = 3320
    E_PAIR_ARM_ASSIGN_INVALID = 3330
    E_PAIR_SINGLE_NOT_ALLOWED = 3340
    E_STATE_TIMEOUT_PAIR_SELECTION = 3390

    E_NAV_GOAL_REJECTED = 4000
    E_NAV_GOAL_TIMEOUT = 4001
    E_NAV_GOAL_CANCELLED = 4002
    E_NAV_LOCALIZATION_LOST = 4010
    E_NAV_LOCALIZATION_LOW_CONFIDENCE = 4011
    E_NAV_PATH_PLAN_FAIL = 4020
    E_NAV_OBSTACLE = 4021
    E_NAV_STUCK = 4022
    E_NAV_GOAL_UNREACHABLE = 4030
    E_NAV_FINE_ALIGN_FAIL = 4040
    E_NAV_WORKPOSE_INVALID = 4041
    E_NAV_FINE_ALIGN_NO_FEEDBACK = 4042
    E_NAV_LIFECYCLE_NOT_ACTIVE = 4050
    E_CHASSIS_ENABLE_FAIL = 4060
    E_CHASSIS_FAULT_RESET_FAIL = 4061
    E_STATE_TIMEOUT_NAVIGATION = 4090
    E_NAV_UNKNOWN = 4099

    E_GRASP_GOAL_REJECTED = 5000
    E_GRASP_GOAL_TIMEOUT = 5001
    E_GRASP_INVALID_PAIR = 5010
    E_STATE_TIMEOUT_PAIR_GRASP = 5090
    E_GRASP_UNKNOWN = 5099

    E_VAC_NO_CONTACT = 5100
    E_VAC_NO_BUILDUP = 5101
    E_VAC_LEAKAGE = 5102
    E_VAC_LOST_DURING_CARRY = 5103
    E_VAC_RELEASE_FAIL = 5104
    E_VAC_NOT_REACHED = 5105
    E_VAC_UNILATERAL_FAIL = 5106
    E_VAC_SENSOR_OFFLINE = 5107
    E_VAC_TIMEOUT = 5108

    E_PLAN_IK_FAIL = 5200
    E_PLAN_TRAJ_FAIL = 5201
    E_PLAN_COLLISION_DETECTED = 5210
    E_PLAN_PREGRASP_INVALID = 5220

    E_MOT_EXEC_FAIL = 5300
    E_MOT_FORCE_LIMIT = 5301
    E_MOT_DROP_BOX = 5310
    E_MOT_PLACE_FAIL = 5320

    E_IO_PUMP_OFFLINE = 6000
    E_IO_VALVE_TIMEOUT = 6001
    E_IO_PRESSURE_SENSOR_FAIL = 6010

    E_SAFETY_ESTOP_HW = 7000
    E_SAFETY_ESTOP_SW = 7001
    E_SAFETY_ZONE_VIOLATED = 7010
    E_SAFETY_COLLISION_RISK = 7011
    E_SAFETY_COMM_LOST = 7020
    E_SAFETY_WATCHDOG = 7030
    E_SAFETY_ESTOP_LOCK_STUCK = 7040

    E_EXT_PERC_OFFLINE = 8000
    E_EXT_PERC_INTERNAL = 8001
    E_EXT_PERC_CAMERA_FAIL = 8010
    E_EXT_PERC_LIDAR_FAIL = 8011
    E_EXT_PERC_YOLO_FAIL = 8012
    E_EXT_PERC_RATE_LOW = 8020

    E_COMM_NODE_OFFLINE = 9000
    E_COMM_SERVICE_TIMEOUT = 9001
    E_COMM_ACTION_TIMEOUT = 9002
    E_COMM_TF_LOOKUP_FAIL = 9010

    E_MAN_PAUSED = 9900
    E_MAN_RESUMED = 9901
    E_MAN_CANCELLED = 9902
    E_MAN_OVERRIDE = 9910

    E_STATE_TIMEOUT_GENERIC = 1090


@dataclass(frozen=True)
class ErrorMeta:
    code: int
    name: str
    level: ErrorLevel
    source: ErrorSource
    default_recovery: RecoveryAction
    description: str


def _meta(
    code: ErrorCode,
    level: ErrorLevel,
    source: ErrorSource,
    recovery: RecoveryAction,
    description: str,
) -> ErrorMeta:
    return ErrorMeta(int(code), code.name, level, source, recovery, description)


ERROR_TABLE: dict[int, ErrorMeta] = {
    0: _meta(ErrorCode.NO_ERROR, ErrorLevel.INFO, ErrorSource.SYSTEM, RecoveryAction.NONE, "无错误"),
    1000: _meta(ErrorCode.E_SYS_BOOT_FAIL, ErrorLevel.FATAL, ErrorSource.SYSTEM, RecoveryAction.WAIT_MANUAL_RECOVERY, "启动自检失败"),
    1001: _meta(ErrorCode.E_SYS_HARDWARE_NOT_READY, ErrorLevel.FATAL, ErrorSource.SYSTEM, RecoveryAction.WAIT_MANUAL_RECOVERY, "硬件节点未上线"),
    1002: _meta(ErrorCode.E_SYS_CONFIG_INVALID, ErrorLevel.FATAL, ErrorSource.SYSTEM, RecoveryAction.WAIT_MANUAL_RECOVERY, "YAML 参数无效"),
    1003: _meta(ErrorCode.E_SYS_TF_TIMEOUT, ErrorLevel.ERROR, ErrorSource.SYSTEM, RecoveryAction.RETRY_CURRENT_STATE, "tf 查询超时"),
    1004: _meta(ErrorCode.E_SYS_ACTION_CANCEL_TIMEOUT, ErrorLevel.ERROR, ErrorSource.SYSTEM, RecoveryAction.WAIT_MANUAL_RECOVERY, "clear_error 阶段 cancel outstanding Action 超时"),
    1005: _meta(ErrorCode.E_SYS_SELF_CHECK_REENTRY_FAIL, ErrorLevel.FATAL, ErrorSource.SYSTEM, RecoveryAction.WAIT_MANUAL_RECOVERY, "clear_error 完成后无法重新进入 SELF_CHECK"),
    1010: _meta(ErrorCode.E_SYS_MODE_TRANSITION_INVALID, ErrorLevel.WARN, ErrorSource.SYSTEM, RecoveryAction.NONE, "非法的模式切换请求"),
    1090: _meta(ErrorCode.E_STATE_TIMEOUT_SYSTEM, ErrorLevel.ERROR, ErrorSource.SYSTEM, RecoveryAction.RETRY_CURRENT_STATE, "System FSM 状态超时"),
    1099: _meta(ErrorCode.E_SYS_UNKNOWN, ErrorLevel.ERROR, ErrorSource.SYSTEM, RecoveryAction.WAIT_MANUAL_RECOVERY, "系统兜底错误"),
    2000: _meta(ErrorCode.E_TASK_VALIDATE_FAIL, ErrorLevel.ERROR, ErrorSource.TASK, RecoveryAction.ABORT_TASK, "任务参数无效"),
    2001: _meta(ErrorCode.E_TASK_PRECONDITION_FAIL, ErrorLevel.ERROR, ErrorSource.TASK, RecoveryAction.RETRY_CURRENT_STATE, "任务前置条件未满足"),
    2002: _meta(ErrorCode.E_TASK_PAUSE_TIMEOUT, ErrorLevel.WARN, ErrorSource.TASK, RecoveryAction.NONE, "暂停超时未恢复"),
    2003: _meta(ErrorCode.E_TASK_CANCELLED, ErrorLevel.INFO, ErrorSource.TASK, RecoveryAction.NONE, "用户主动取消"),
    2010: _meta(ErrorCode.E_TASK_CHILD_FAILED, ErrorLevel.ERROR, ErrorSource.TASK, RecoveryAction.ABORT_TASK, "WallDestackingFSM 上报失败"),
    2011: _meta(ErrorCode.E_TASK_CHILD_TIMEOUT, ErrorLevel.ERROR, ErrorSource.TASK, RecoveryAction.ABORT_TASK, "WallDestackingFSM Action 超时"),
    2090: _meta(ErrorCode.E_STATE_TIMEOUT_TASK, ErrorLevel.ERROR, ErrorSource.TASK, RecoveryAction.RETRY_CURRENT_STATE, "Task FSM 状态超时"),
    3000: _meta(ErrorCode.E_WALL_STATE_TIMEOUT, ErrorLevel.ERROR, ErrorSource.WALL_STRATEGY, RecoveryAction.RETRY_CURRENT_STATE, "Wall 父 FSM 某个状态超时"),
    3001: _meta(ErrorCode.E_WALL_INVALID_TRANSITION, ErrorLevel.FATAL, ErrorSource.WALL_STRATEGY, RecoveryAction.ABORT_TASK, "状态跳转表非法"),
    3010: _meta(ErrorCode.E_WALL_PHASE_NOT_COMPLETE_BUT_NO_PAIR, ErrorLevel.ERROR, ErrorSource.WALL_STRATEGY, RecoveryAction.REPERCEPTION, "phase 未完成但选不出 pair"),
    3020: _meta(ErrorCode.E_WALL_EMPTY_VERIFY_CONFLICT, ErrorLevel.WARN, ErrorSource.WALL_STRATEGY, RecoveryAction.REBUILD_GRID, "网格空但视觉仍看到箱"),
    3030: _meta(ErrorCode.E_WALL_TASK_COMPLETE_TIMEOUT, ErrorLevel.WARN, ErrorSource.WALL_STRATEGY, RecoveryAction.NONE, "多帧无箱确认阶段超时"),
    3090: _meta(ErrorCode.E_STATE_TIMEOUT_WALL_STRATEGY, ErrorLevel.ERROR, ErrorSource.WALL_STRATEGY, RecoveryAction.RETRY_CURRENT_STATE, "Wall Strategy FSM 状态超时"),
    3099: _meta(ErrorCode.E_WALL_UNKNOWN, ErrorLevel.ERROR, ErrorSource.WALL_STRATEGY, RecoveryAction.WAIT_MANUAL_RECOVERY, "Wall 策略兜底错误"),
    3100: _meta(ErrorCode.E_MAP_GLOBAL_SCAN_FAIL, ErrorLevel.ERROR, ErrorSource.WALL_MAPPING, RecoveryAction.RETRY_CURRENT_STATE, "全局感知开窗失败或检测流不可用"),
    3101: _meta(ErrorCode.E_MAP_GLOBAL_SCAN_TIMEOUT, ErrorLevel.ERROR, ErrorSource.WALL_MAPPING, RecoveryAction.RETRY_CURRENT_STATE, "全局扫描超时"),
    3110: _meta(ErrorCode.E_MAP_NO_DETECTION, ErrorLevel.WARN, ErrorSource.WALL_MAPPING, RecoveryAction.RETRY_CURRENT_STATE, "全局扫描结果为空"),
    3111: _meta(ErrorCode.E_MAP_INSUFFICIENT_DETECTION, ErrorLevel.WARN, ErrorSource.WALL_MAPPING, RecoveryAction.RETRY_CURRENT_STATE, "检测数低于阈值"),
    3120: _meta(ErrorCode.E_MAP_WALL_FRAME_FAIL, ErrorLevel.ERROR, ErrorSource.WALL_MAPPING, RecoveryAction.RETRY_CURRENT_STATE, "拟合墙面失败"),
    3121: _meta(ErrorCode.E_MAP_WALL_FRAME_LOW_CONFIDENCE, ErrorLevel.WARN, ErrorSource.WALL_MAPPING, RecoveryAction.RETRY_CURRENT_STATE, "墙面置信度低"),
    3130: _meta(ErrorCode.E_MAP_GRID_INCOMPLETE, ErrorLevel.WARN, ErrorSource.WALL_MAPPING, RecoveryAction.NONE, "网格部分 slot 没匹配上"),
    3131: _meta(ErrorCode.E_MAP_GRID_BUILD_FAIL, ErrorLevel.FATAL, ErrorSource.WALL_MAPPING, RecoveryAction.WAIT_MANUAL_RECOVERY, "完全无法建出有效网格"),
    3140: _meta(ErrorCode.E_MAP_NO_NEW_WALL, ErrorLevel.INFO, ErrorSource.WALL_MAPPING, RecoveryAction.NONE, "看不到新墙"),
    3190: _meta(ErrorCode.E_STATE_TIMEOUT_MAPPING, ErrorLevel.ERROR, ErrorSource.WALL_MAPPING, RecoveryAction.RETRY_CURRENT_STATE, "WallMapping FSM 状态超时"),
    3200: _meta(ErrorCode.E_PERC_LOCAL_SCAN_FAIL, ErrorLevel.ERROR, ErrorSource.PHASE_PERCEPTION, RecoveryAction.RETRY_CURRENT_STATE, "局部感知开窗失败"),
    3201: _meta(ErrorCode.E_PERC_LOCAL_SCAN_TIMEOUT, ErrorLevel.ERROR, ErrorSource.PHASE_PERCEPTION, RecoveryAction.RETRY_CURRENT_STATE, "局部扫描超时"),
    3210: _meta(ErrorCode.E_PERC_NO_LOCAL_DETECTION, ErrorLevel.WARN, ErrorSource.PHASE_PERCEPTION, RecoveryAction.RETRY_CURRENT_STATE, "局部检测为空"),
    3220: _meta(ErrorCode.E_PERC_ASSOCIATION_FAIL, ErrorLevel.WARN, ErrorSource.PHASE_PERCEPTION, RecoveryAction.RETRY_CURRENT_STATE, "检测匹配不到 slot"),
    3221: _meta(ErrorCode.E_PERC_TOO_MANY_FALSE_POSITIVE, ErrorLevel.WARN, ErrorSource.PHASE_PERCEPTION, RecoveryAction.NONE, "误检过多"),
    3230: _meta(ErrorCode.E_PERC_SLOT_POSE_OUTLIER, ErrorLevel.WARN, ErrorSource.PHASE_PERCEPTION, RecoveryAction.NONE, "slot 实测 pose 偏离预期"),
    3290: _meta(ErrorCode.E_STATE_TIMEOUT_PHASE_PERCEPTION, ErrorLevel.ERROR, ErrorSource.PHASE_PERCEPTION, RecoveryAction.RETRY_CURRENT_STATE, "PhasePerception FSM 状态超时"),
    3300: _meta(ErrorCode.E_PAIR_NO_CANDIDATE, ErrorLevel.INFO, ErrorSource.PAIR_SELECTION, RecoveryAction.NONE, "当前 phase 无可抓 slot"),
    3310: _meta(ErrorCode.E_PAIR_NO_REACHABLE, ErrorLevel.ERROR, ErrorSource.PAIR_SELECTION, RecoveryAction.MOVE_BASE, "候选都不可达"),
    3320: _meta(ErrorCode.E_PAIR_DUAL_CONFLICT, ErrorLevel.WARN, ErrorSource.PAIR_SELECTION, RecoveryAction.SWITCH_TARGET, "双臂明显干涉"),
    3330: _meta(ErrorCode.E_PAIR_ARM_ASSIGN_INVALID, ErrorLevel.ERROR, ErrorSource.PAIR_SELECTION, RecoveryAction.RETRY_CURRENT_STATE, "左右手分配模糊"),
    3340: _meta(ErrorCode.E_PAIR_SINGLE_NOT_ALLOWED, ErrorLevel.INFO, ErrorSource.PAIR_SELECTION, RecoveryAction.SWITCH_PHASE, "只剩单箱但不允许单臂"),
    3390: _meta(ErrorCode.E_STATE_TIMEOUT_PAIR_SELECTION, ErrorLevel.ERROR, ErrorSource.PAIR_SELECTION, RecoveryAction.RETRY_CURRENT_STATE, "PairSelection FSM 状态超时"),
    4000: _meta(ErrorCode.E_NAV_GOAL_REJECTED, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.RETRY_CURRENT_STATE, "导航 Action 拒绝 goal"),
    4001: _meta(ErrorCode.E_NAV_GOAL_TIMEOUT, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.RETRY_CURRENT_STATE, "导航 timeout"),
    4002: _meta(ErrorCode.E_NAV_GOAL_CANCELLED, ErrorLevel.INFO, ErrorSource.NAVIGATION, RecoveryAction.NONE, "导航 goal 被上层取消"),
    4010: _meta(ErrorCode.E_NAV_LOCALIZATION_LOST, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.RELOCALIZE, "定位丢失"),
    4011: _meta(ErrorCode.E_NAV_LOCALIZATION_LOW_CONFIDENCE, ErrorLevel.WARN, ErrorSource.NAVIGATION, RecoveryAction.RELOCALIZE, "定位置信度低"),
    4020: _meta(ErrorCode.E_NAV_PATH_PLAN_FAIL, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.REPLAN, "路径规划失败"),
    4021: _meta(ErrorCode.E_NAV_OBSTACLE, ErrorLevel.WARN, ErrorSource.NAVIGATION, RecoveryAction.RETRY_CURRENT_STATE, "障碍物"),
    4022: _meta(ErrorCode.E_NAV_STUCK, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.RETREAT_SAFE, "底盘卡住超时"),
    4030: _meta(ErrorCode.E_NAV_GOAL_UNREACHABLE, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.WAIT_MANUAL_RECOVERY, "目标点不可达"),
    4040: _meta(ErrorCode.E_NAV_FINE_ALIGN_FAIL, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.RETRY_CURRENT_STATE, "对墙微调失败"),
    4041: _meta(ErrorCode.E_NAV_WORKPOSE_INVALID, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.RETRY_CURRENT_STATE, "到位但偏差超阈值"),
    4042: _meta(ErrorCode.E_NAV_FINE_ALIGN_NO_FEEDBACK, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.REPERCEPTION, "对墙微调无有效反馈"),
    4050: _meta(ErrorCode.E_NAV_LIFECYCLE_NOT_ACTIVE, ErrorLevel.FATAL, ErrorSource.NAVIGATION, RecoveryAction.WAIT_MANUAL_RECOVERY, "Nav2 lifecycle manager 未 active"),
    4060: _meta(ErrorCode.E_CHASSIS_ENABLE_FAIL, ErrorLevel.FATAL, ErrorSource.NAVIGATION, RecoveryAction.WAIT_MANUAL_RECOVERY, "底盘 enable 失败"),
    4061: _meta(ErrorCode.E_CHASSIS_FAULT_RESET_FAIL, ErrorLevel.FATAL, ErrorSource.NAVIGATION, RecoveryAction.WAIT_MANUAL_RECOVERY, "底盘 reset_fault 失败"),
    4090: _meta(ErrorCode.E_STATE_TIMEOUT_NAVIGATION, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.RETRY_CURRENT_STATE, "Navigation FSM 状态超时"),
    4099: _meta(ErrorCode.E_NAV_UNKNOWN, ErrorLevel.ERROR, ErrorSource.NAVIGATION, RecoveryAction.WAIT_MANUAL_RECOVERY, "导航兜底错误"),
    5000: _meta(ErrorCode.E_GRASP_GOAL_REJECTED, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETRY_CURRENT_STATE, "抓取 Action 拒绝"),
    5001: _meta(ErrorCode.E_GRASP_GOAL_TIMEOUT, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETRY_CURRENT_STATE, "抓取整体超时"),
    5010: _meta(ErrorCode.E_GRASP_INVALID_PAIR, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.SWITCH_TARGET, "入参 pair 不合法"),
    5090: _meta(ErrorCode.E_STATE_TIMEOUT_PAIR_GRASP, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETRY_CURRENT_STATE, "PairGrasp FSM 状态超时"),
    5099: _meta(ErrorCode.E_GRASP_UNKNOWN, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.WAIT_MANUAL_RECOVERY, "抓取兜底错误"),
    5100: _meta(ErrorCode.E_VAC_NO_CONTACT, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETRY_CURRENT_STATE, "接触失败"),
    5101: _meta(ErrorCode.E_VAC_NO_BUILDUP, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETRY_CURRENT_STATE, "真空建立失败"),
    5102: _meta(ErrorCode.E_VAC_LEAKAGE, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.SWITCH_TARGET, "真空漏气"),
    5103: _meta(ErrorCode.E_VAC_LOST_DURING_CARRY, ErrorLevel.FATAL, ErrorSource.PAIR_GRASP, RecoveryAction.WAIT_MANUAL_RECOVERY, "搬运中真空丢失"),
    5104: _meta(ErrorCode.E_VAC_RELEASE_FAIL, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETRY_CURRENT_STATE, "释放失败"),
    5105: _meta(ErrorCode.E_VAC_NOT_REACHED, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETRY_CURRENT_STATE, "建压超时后未达到吸附阈值"),
    5106: _meta(ErrorCode.E_VAC_UNILATERAL_FAIL, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.SWITCH_TARGET, "双臂模式一侧建压失败"),
    5107: _meta(ErrorCode.E_VAC_SENSOR_OFFLINE, ErrorLevel.FATAL, ErrorSource.PAIR_GRASP, RecoveryAction.WAIT_MANUAL_RECOVERY, "压力传感器离线"),
    5108: _meta(ErrorCode.E_VAC_TIMEOUT, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETRY_CURRENT_STATE, "真空检查阶段整体超时"),
    5200: _meta(ErrorCode.E_PLAN_IK_FAIL, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.SWITCH_TARGET, "IK 解不出来"),
    5201: _meta(ErrorCode.E_PLAN_TRAJ_FAIL, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.REPLAN, "轨迹规划失败"),
    5210: _meta(ErrorCode.E_PLAN_COLLISION_DETECTED, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.REPLAN, "碰撞检查失败"),
    5220: _meta(ErrorCode.E_PLAN_PREGRASP_INVALID, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.SWITCH_TARGET, "预抓取 pose 不可达"),
    5300: _meta(ErrorCode.E_MOT_EXEC_FAIL, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETREAT_SAFE, "运动执行失败"),
    5301: _meta(ErrorCode.E_MOT_FORCE_LIMIT, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETREAT_SAFE, "力矩超限"),
    5310: _meta(ErrorCode.E_MOT_DROP_BOX, ErrorLevel.FATAL, ErrorSource.PAIR_GRASP, RecoveryAction.WAIT_MANUAL_RECOVERY, "掉箱"),
    5320: _meta(ErrorCode.E_MOT_PLACE_FAIL, ErrorLevel.ERROR, ErrorSource.PAIR_GRASP, RecoveryAction.RETRY_CURRENT_STATE, "放置失败"),
    6000: _meta(ErrorCode.E_IO_PUMP_OFFLINE, ErrorLevel.FATAL, ErrorSource.IO_VACUUM, RecoveryAction.WAIT_MANUAL_RECOVERY, "真空泵离线"),
    6001: _meta(ErrorCode.E_IO_VALVE_TIMEOUT, ErrorLevel.ERROR, ErrorSource.IO_VACUUM, RecoveryAction.RETRY_CURRENT_STATE, "阀响应超时"),
    6010: _meta(ErrorCode.E_IO_PRESSURE_SENSOR_FAIL, ErrorLevel.ERROR, ErrorSource.IO_VACUUM, RecoveryAction.WAIT_MANUAL_RECOVERY, "压力传感器异常"),
    7000: _meta(ErrorCode.E_SAFETY_ESTOP_HW, ErrorLevel.ESTOP, ErrorSource.SAFETY, RecoveryAction.E_STOP, "硬件急停按钮按下"),
    7001: _meta(ErrorCode.E_SAFETY_ESTOP_SW, ErrorLevel.ESTOP, ErrorSource.SAFETY, RecoveryAction.E_STOP, "软件急停"),
    7010: _meta(ErrorCode.E_SAFETY_ZONE_VIOLATED, ErrorLevel.FATAL, ErrorSource.SAFETY, RecoveryAction.RETREAT_SAFE, "安全区违反"),
    7011: _meta(ErrorCode.E_SAFETY_COLLISION_RISK, ErrorLevel.ERROR, ErrorSource.SAFETY, RecoveryAction.RETREAT_SAFE, "检测到碰撞风险"),
    7020: _meta(ErrorCode.E_SAFETY_COMM_LOST, ErrorLevel.FATAL, ErrorSource.SAFETY, RecoveryAction.E_STOP, "关键通信丢失"),
    7030: _meta(ErrorCode.E_SAFETY_WATCHDOG, ErrorLevel.FATAL, ErrorSource.SAFETY, RecoveryAction.E_STOP, "看门狗超时"),
    7040: _meta(ErrorCode.E_SAFETY_ESTOP_LOCK_STUCK, ErrorLevel.FATAL, ErrorSource.SAFETY, RecoveryAction.WAIT_MANUAL_RECOVERY, "急停锁未释放"),
    8000: _meta(ErrorCode.E_EXT_PERC_OFFLINE, ErrorLevel.FATAL, ErrorSource.PERCEPTION_EXT, RecoveryAction.WAIT_MANUAL_RECOVERY, "perception_node 进程离线"),
    8001: _meta(ErrorCode.E_EXT_PERC_INTERNAL, ErrorLevel.ERROR, ErrorSource.PERCEPTION_EXT, RecoveryAction.RETRY_CURRENT_STATE, "YOLO 推理崩溃或重启中"),
    8010: _meta(ErrorCode.E_EXT_PERC_CAMERA_FAIL, ErrorLevel.FATAL, ErrorSource.PERCEPTION_EXT, RecoveryAction.WAIT_MANUAL_RECOVERY, "RGB 相机故障"),
    8011: _meta(ErrorCode.E_EXT_PERC_LIDAR_FAIL, ErrorLevel.FATAL, ErrorSource.PERCEPTION_EXT, RecoveryAction.WAIT_MANUAL_RECOVERY, "雷达点云故障"),
    8012: _meta(ErrorCode.E_EXT_PERC_YOLO_FAIL, ErrorLevel.ERROR, ErrorSource.PERCEPTION_EXT, RecoveryAction.REPERCEPTION, "box_perception result 超时或 fallback"),
    8020: _meta(ErrorCode.E_EXT_PERC_RATE_LOW, ErrorLevel.WARN, ErrorSource.PERCEPTION_EXT, RecoveryAction.NONE, "感知发布频率低"),
    9000: _meta(ErrorCode.E_COMM_NODE_OFFLINE, ErrorLevel.FATAL, ErrorSource.COMMUNICATION, RecoveryAction.WAIT_MANUAL_RECOVERY, "关键节点离线"),
    9001: _meta(ErrorCode.E_COMM_SERVICE_TIMEOUT, ErrorLevel.ERROR, ErrorSource.COMMUNICATION, RecoveryAction.RETRY_CURRENT_STATE, "Service 超时"),
    9002: _meta(ErrorCode.E_COMM_ACTION_TIMEOUT, ErrorLevel.ERROR, ErrorSource.COMMUNICATION, RecoveryAction.RETRY_CURRENT_STATE, "Action 超时"),
    9010: _meta(ErrorCode.E_COMM_TF_LOOKUP_FAIL, ErrorLevel.ERROR, ErrorSource.COMMUNICATION, RecoveryAction.RETRY_CURRENT_STATE, "tf 查询失败"),
    9900: _meta(ErrorCode.E_MAN_PAUSED, ErrorLevel.INFO, ErrorSource.MANUAL, RecoveryAction.NONE, "用户暂停"),
    9901: _meta(ErrorCode.E_MAN_RESUMED, ErrorLevel.INFO, ErrorSource.MANUAL, RecoveryAction.NONE, "用户恢复"),
    9902: _meta(ErrorCode.E_MAN_CANCELLED, ErrorLevel.INFO, ErrorSource.MANUAL, RecoveryAction.ABORT_TASK, "用户取消"),
    9910: _meta(ErrorCode.E_MAN_OVERRIDE, ErrorLevel.WARN, ErrorSource.MANUAL, RecoveryAction.NONE, "人工接管"),
}


def get_error_meta(code: int | ErrorCode) -> ErrorMeta:
    return ERROR_TABLE.get(int(code), ERROR_TABLE[int(ErrorCode.E_SYS_UNKNOWN)])
