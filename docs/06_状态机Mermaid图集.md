# 06 状态机 Mermaid 图集

> 7 张 Mermaid 状态图，与 02 状态规格手册一一对应。GitHub/VSCode 直接渲染。改图后必须同步改 02 文档。

---

## 0. 阅读说明

- 实线 → 正常跳转
- 异常跳转：用箭头描述文字体现（兼容旧渲染器）
- `[*]` 表示初始/终态
- 节点名严格对应代码中的 BaseState 子类的 `name` 属性
- 图中只画"状态拓扑"，不展示 timeout/retry/error_code（在 02 文档里查）

---

## 1. RobotSystemFSM

```mermaid
stateDiagram
    [*] --> BOOTING

    BOOTING --> SELF_CHECK: 节点齐
    BOOTING --> FAULT: 心跳超时

    SELF_CHECK --> STANDBY: 自检通过
    SELF_CHECK --> FAULT: 任意项失败

    STANDBY --> AUTO_MODE: /task/start
    STANDBY --> MANUAL_MODE: 手动切换
    STANDBY --> SHUTDOWN: 关机

    AUTO_MODE --> STANDBY: 任务完成
    AUTO_MODE --> PAUSED: /task/pause
    AUTO_MODE --> FAULT: FATAL
    AUTO_MODE --> E_STOP: 急停

    MANUAL_MODE --> STANDBY: 切回自动

    PAUSED --> AUTO_MODE: /task/resume
    PAUSED --> FAULT: 暂停超时

    FAULT --> SELF_CHECK: /clear_error 五阶段完成

    E_STOP --> SELF_CHECK: 急停解除+/clear_error

    SHUTDOWN --> [*]
```

---

## 2. TaskFSM

```mermaid
stateDiagram
    [*] --> WAIT_TASK

    WAIT_TASK --> ACCEPT_TASK: /task/start
    ACCEPT_TASK --> VALIDATE_TASK: 入参解析成功
    ACCEPT_TASK --> FAIL_TASK: 入参错误

    VALIDATE_TASK --> PREPARE_TASK: 校验通过
    VALIDATE_TASK --> FAIL_TASK: 前置条件不满足

    PREPARE_TASK --> RUN_TASK: 准备完毕
    PREPARE_TASK --> FAIL_TASK: 准备失败

    RUN_TASK --> VERIFY_TASK_RESULT: WallDestacking 完成
    RUN_TASK --> CANCEL_TASK: /task/cancel
    RUN_TASK --> FAIL_TASK: WallDestacking 失败/超时

    VERIFY_TASK_RESULT --> COMPLETE_TASK: 结果合法
    VERIFY_TASK_RESULT --> FAIL_TASK: 结果不合法

    COMPLETE_TASK --> WAIT_TASK
    FAIL_TASK --> WAIT_TASK
    CANCEL_TASK --> WAIT_TASK
```

---

## 3. WallDestackingFSM（核心）

```mermaid
stateDiagram
    [*] --> INIT_WALL_TASK

    INIT_WALL_TASK --> NAVIGATE_TO_OBSERVATION_POSE

    NAVIGATE_TO_OBSERVATION_POSE --> RUN_WALL_MAPPING: 到位
    NAVIGATE_TO_OBSERVATION_POSE --> WALL_ERROR_HANDLE: 导航失败

    RUN_WALL_MAPPING --> CHECK_WALL_VALID: 建网成功
    RUN_WALL_MAPPING --> WALL_ERROR_HANDLE: 建网失败

    CHECK_WALL_VALID --> SELECT_PHASE: grid 有效
    CHECK_WALL_VALID --> VERIFY_TASK_COMPLETE: 无新墙
    CHECK_WALL_VALID --> WALL_ERROR_HANDLE: grid 不可用

    SELECT_PHASE --> NAVIGATE_TO_PHASE_WORKPOSE
    SELECT_PHASE --> DECIDE_NEXT_WALL: 两 phase 都已完成

    NAVIGATE_TO_PHASE_WORKPOSE --> RUN_PHASE_PERCEPTION: 到位+对齐
    NAVIGATE_TO_PHASE_WORKPOSE --> WALL_ERROR_HANDLE: 失败

    RUN_PHASE_PERCEPTION --> RUN_PAIR_SELECTION: 感知更新完成
    RUN_PHASE_PERCEPTION --> WALL_ERROR_HANDLE: 失败

    RUN_PAIR_SELECTION --> DISPATCH_PAIR_GRASP: 选出 pair
    RUN_PAIR_SELECTION --> DECIDE_NEXT_PHASE: 无候选(正常完成信号)
    RUN_PAIR_SELECTION --> WALL_ERROR_HANDLE: 选取失败

    DISPATCH_PAIR_GRASP --> WAIT_PAIR_GRASP_RESULT: goal 已发
    DISPATCH_PAIR_GRASP --> WALL_ERROR_HANDLE: goal 拒绝

    WAIT_PAIR_GRASP_RESULT --> UPDATE_GRID_AFTER_GRASP: result 到达

    UPDATE_GRID_AFTER_GRASP --> DECIDE_NEXT_PAIR
    UPDATE_GRID_AFTER_GRASP --> WALL_ERROR_HANDLE: 掉箱等 FATAL

    DECIDE_NEXT_PAIR --> RUN_PHASE_PERCEPTION: 当前 phase 还有
    DECIDE_NEXT_PAIR --> DECIDE_NEXT_PHASE: 当前 phase 完

    DECIDE_NEXT_PHASE --> NAVIGATE_TO_PHASE_WORKPOSE: LEFT 完→RIGHT
    DECIDE_NEXT_PHASE --> DECIDE_NEXT_WALL: RIGHT 完

    DECIDE_NEXT_WALL --> NAVIGATE_TO_OBSERVATION_POSE: 进下一面墙
    DECIDE_NEXT_WALL --> VERIFY_TASK_COMPLETE: 达到 max_walls

    VERIFY_TASK_COMPLETE --> WALL_DONE: 多帧无箱
    VERIFY_TASK_COMPLETE --> RUN_WALL_MAPPING: 仍看到箱(重扫)
    VERIFY_TASK_COMPLETE --> WALL_ERROR_HANDLE: 超时

    WALL_DONE --> [*]

    WALL_ERROR_HANDLE --> NAVIGATE_TO_OBSERVATION_POSE: recovery 决策
    WALL_ERROR_HANDLE --> NAVIGATE_TO_PHASE_WORKPOSE: recovery 决策
    WALL_ERROR_HANDLE --> RUN_PHASE_PERCEPTION: recovery 决策
    WALL_ERROR_HANDLE --> RUN_PAIR_SELECTION: recovery 决策
    WALL_ERROR_HANDLE --> [*]: ABORT
```

> WallDestackingFSM 主流程线性，但 WALL_ERROR_HANDLE 可跳回多个状态——具体由 WallRecoveryFSM 决定。

---

## 4. WallMappingFSM

```mermaid
stateDiagram
    [*] --> START_WINDOW

    START_WINDOW --> COLLECT_FRAMES

    COLLECT_FRAMES --> FILTER_DETECTIONS: 帧数足够
    COLLECT_FRAMES --> MAPPING_ERROR: 超时/无检测

    FILTER_DETECTIONS --> ESTIMATE_WALL_FRAME: 滤波后 ≥ min_valid
    FILTER_DETECTIONS --> MAPPING_ERROR: 不足

    ESTIMATE_WALL_FRAME --> BUILD_5X5_GRID: 拟合成功
    ESTIMATE_WALL_FRAME --> MAPPING_ERROR: 拟合失败

    BUILD_5X5_GRID --> INIT_GRID_SLOTS

    INIT_GRID_SLOTS --> VALIDATE_GRID

    VALIDATE_GRID --> CHECK_NEW_WALL: grid 合法
    VALIDATE_GRID --> MAPPING_ERROR: 不合法

    CHECK_NEW_WALL --> REPORT

    REPORT --> [*]
    MAPPING_ERROR --> [*]
```

---

## 5. PhasePerceptionFSM

```mermaid
stateDiagram
    [*] --> START_LOCAL_WINDOW

    START_LOCAL_WINDOW --> COLLECT_LOCAL_FRAMES

    COLLECT_LOCAL_FRAMES --> FILTER_LOCAL: 帧数足够
    COLLECT_LOCAL_FRAMES --> PERCEPTION_ERROR: 超时/无检测

    FILTER_LOCAL --> TRANSFORM_TO_ROBOT

    TRANSFORM_TO_ROBOT --> MATCH_TO_SLOTS: tf OK
    TRANSFORM_TO_ROBOT --> PERCEPTION_ERROR: tf 失败

    MATCH_TO_SLOTS --> UPDATE_SLOT_POSES: 至少 1 个匹配
    MATCH_TO_SLOTS --> PERCEPTION_ERROR: 全部不匹配

    UPDATE_SLOT_POSES --> MARK_UNSEEN

    MARK_UNSEEN --> REPORT

    REPORT --> [*]
    PERCEPTION_ERROR --> [*]
```

---

## 6. PairSelectionFSM

```mermaid
stateDiagram
    [*] --> FILTER_AVAILABLE

    FILTER_AVAILABLE --> SORT_BY_PRIORITY: 候选 ≥ 1
    FILTER_AVAILABLE --> PAIR_SELECTION_ERROR: 无候选(NO_CANDIDATE)

    SORT_BY_PRIORITY --> BUILD_CANDIDATES

    BUILD_CANDIDATES --> VALIDATE_REACHABILITY

    VALIDATE_REACHABILITY --> CHECK_DUAL_ARM_CONFLICT: 选出可达
    VALIDATE_REACHABILITY --> PAIR_SELECTION_ERROR: 全不可达

    CHECK_DUAL_ARM_CONFLICT --> ASSIGN_ARMS_BY_Y: 无冲突
    CHECK_DUAL_ARM_CONFLICT --> VALIDATE_REACHABILITY: 有冲突,换组
    CHECK_DUAL_ARM_CONFLICT --> PAIR_SELECTION_ERROR: 候选耗尽

    ASSIGN_ARMS_BY_Y --> BUILD_GRASP_PAIR

    BUILD_GRASP_PAIR --> REPORT

    REPORT --> [*]
    PAIR_SELECTION_ERROR --> [*]
```

---

## 7. WallRecoveryFSM

```mermaid
stateDiagram
    [*] --> RECEIVE_ERROR

    RECEIVE_ERROR --> CLASSIFY

    CLASSIFY --> CHECK_RETRY_LIMIT

    CHECK_RETRY_LIMIT --> SELECT_RECOVERY_ACTION: 未超上限
    CHECK_RETRY_LIMIT --> WAIT_MANUAL_RECOVERY: 超上限

    SELECT_RECOVERY_ACTION --> EXECUTE_RECOVERY

    EXECUTE_RECOVERY --> REPORT: 成功
    EXECUTE_RECOVERY --> WAIT_MANUAL_RECOVERY: 失败

    WAIT_MANUAL_RECOVERY --> REPORT: /clear_error
    WAIT_MANUAL_RECOVERY --> [*]: ABORT(终态 FAILURE)

    REPORT --> [*]
```

---

## 8. SafetyMonitorFSM

```mermaid
stateDiagram
    [*] --> NORMAL

    NORMAL --> WARNING: 通信丢失/区域违反/碰撞风险
    NORMAL --> EMERGENCY: 急停(硬件/软件)

    WARNING --> NORMAL: 原因解除
    WARNING --> EMERGENCY: 升级

    EMERGENCY --> NORMAL: 急停解除+/clear_error
```

---

## 9. BaseNavigationFSM 推荐骨架（A 自定义）

```mermaid
stateDiagram
    [*] --> RECEIVE_GOAL

    RECEIVE_GOAL --> CHECK_PRECONDITION
    CHECK_PRECONDITION --> CHECK_LOCALIZATION

    CHECK_LOCALIZATION --> PLAN_PATH: 定位 OK
    CHECK_LOCALIZATION --> RELOCALIZE: 定位丢失

    RELOCALIZE --> CHECK_LOCALIZATION
    RELOCALIZE --> NAV_ERROR

    PLAN_PATH --> EXECUTE_BASE_NAV
    PLAN_PATH --> NAV_ERROR

    EXECUTE_BASE_NAV --> MONITOR_NAV
    MONITOR_NAV --> EXECUTE_BASE_NAV: 继续
    MONITOR_NAV --> CHECK_GOAL_REACHED: 到位
    MONITOR_NAV --> NAV_ERROR: 卡住/障碍

    CHECK_GOAL_REACHED --> FINE_ALIGN_TO_WALL: require_fine_alignment
    CHECK_GOAL_REACHED --> CHECK_WORKPOSE_VALID: 不需要

    FINE_ALIGN_TO_WALL --> CHECK_WORKPOSE_VALID
    FINE_ALIGN_TO_WALL --> NAV_ERROR

    CHECK_WORKPOSE_VALID --> REPORT_NAV_RESULT: 偏差合格
    CHECK_WORKPOSE_VALID --> NAV_ERROR: 偏差超阈值

    REPORT_NAV_RESULT --> [*]
    NAV_ERROR --> [*]
```

> 上图为推荐骨架，A 在实现时可调整内部状态名与跳转，但必须保证 Action Goal/Result 语义符合 03 接口契约。

---

## 10. PairGraspExecutionFSM 推荐骨架（B 自定义）

```mermaid
stateDiagram
    [*] --> RECEIVE_PAIR

    RECEIVE_PAIR --> CHECK_PAIR_VALID

    CHECK_PAIR_VALID --> PLAN_PREGRASP: 合法
    CHECK_PAIR_VALID --> EXECUTION_ERROR: 不合法

    PLAN_PREGRASP --> MOVE_TO_PREGRASP: 规划成功
    PLAN_PREGRASP --> EXECUTION_ERROR: 规划失败

    MOVE_TO_PREGRASP --> APPROACH_AND_CONTACT: 到位
    MOVE_TO_PREGRASP --> EXECUTION_ERROR: 运动失败

    APPROACH_AND_CONTACT --> CHECK_VACUUM
    APPROACH_AND_CONTACT --> EXECUTION_ERROR: 接触失败

    CHECK_VACUUM --> ATTACH_BOX_MODEL: 真空建立
    CHECK_VACUUM --> EXECUTION_ERROR: 真空失败/泄漏

    ATTACH_BOX_MODEL --> PLAN_EXTRACT
    PLAN_EXTRACT --> EXECUTE_EXTRACT: 规划成功
    PLAN_EXTRACT --> EXECUTION_ERROR

    EXECUTE_EXTRACT --> PLAN_CARRY
    EXECUTE_EXTRACT --> EXECUTION_ERROR: 抬离失败

    PLAN_CARRY --> EXECUTE_CARRY
    EXECUTE_CARRY --> RELEASE_BOX
    EXECUTE_CARRY --> EXECUTION_ERROR: 搬运失败/掉箱(FATAL)

    RELEASE_BOX --> RETREAT_SAFE: 释放成功
    RELEASE_BOX --> EXECUTION_ERROR: 释放失败

    RETREAT_SAFE --> REPORT_EXECUTION_RESULT
    RETREAT_SAFE --> EXECUTION_ERROR

    REPORT_EXECUTION_RESULT --> [*]
    EXECUTION_ERROR --> [*]
```

> 同上，B 可调整内部状态。Result 必须符合 03 接口契约里 PairGraspResult 的字段约束。

---

## 11. 三层 FSM 调用关系（系统视图）

```mermaid
graph TD
    SystemFSM["RobotSystemFSM\ntask_manager_node"]
    TaskFSM["TaskFSM\ntask_manager_node"]
    WallFSM["WallDestackingFSM\nwall_destacking_strategy_node"]

    Mapping[WallMappingFSM]
    Phase[PhasePerceptionFSM]
    Pair[PairSelectionFSM]
    Recovery[WallRecoveryFSM]

    NavFSM["BaseNavigationFSM\nnavigation_manager_node"]
    GraspFSM["PairGraspExecutionFSM\npair_grasp_execution_node"]

    Safety["SafetyMonitorFSM\nsafety_monitor_node"]
    PercAdapter["perception_adapter_node\n适配层"]
    VacuumIO["vacuum_io_node\nmock/hardware"]
    Perception["perception_node\n外部既有"]

    SystemFSM -->|同进程类调用| TaskFSM
    TaskFSM -->|Action /run_wall_destacking| WallFSM

    WallFSM -->|同进程类调用| Mapping
    WallFSM -->|同进程类调用| Phase
    WallFSM -->|同进程类调用| Pair
    WallFSM -->|同进程类调用| Recovery

    WallFSM -->|Action /navigate_to_pose| NavFSM
    WallFSM -->|Action /execute_pair_grasp| GraspFSM

    Perception -->|/box_perception/result| PercAdapter
    PercAdapter -->|/perception/box_detections| Mapping
    PercAdapter -->|/perception/box_detections| Phase
    PercAdapter -->|/perception/health| Safety

    GraspFSM -->|/vacuum/cmd| VacuumIO
    VacuumIO -->|/vacuum/pressure_raw / health| GraspFSM

    Safety -->|/safety/estop pub| SystemFSM
    Safety -->|/safety/estop pub| WallFSM
    Safety -->|/safety/estop pub| NavFSM
    Safety -->|/safety/estop pub| GraspFSM
```

---

## 12. 数据流（一次完整 pair 抓取）

```mermaid
sequenceDiagram
    autonumber
    participant U as "User/UI"
    participant TM as task_manager_node
    participant ST as strategy_node
    participant NV as navigation_manager_node
    participant GR as pair_grasp_execution_node
    participant PA as perception_adapter_node
    participant PE as perception_node
    participant VI as vacuum_io_node

    U->>TM: /task/start
    TM->>ST: Action: run_wall_destacking
    ST->>NV: Action: navigate(OBSERVATION)
    NV-->>ST: Result(success)
    PE-->>PA: Topic: /box_perception/result (持续)
    PA-->>ST: Topic: /perception/box_detections (持续)
    Note over ST: WallMappingFSM 开窗收 N 帧
    ST->>NV: Action: navigate(LEFT_PHASE)
    NV-->>ST: Result(success)
    Note over ST: PhasePerceptionFSM 开窗收 5 帧 + 匹配
    Note over ST: PairSelectionFSM 选 pair
    ST->>GR: Action: execute_pair_grasp(pair)
    GR->>VI: Topic: /vacuum/cmd
    VI-->>GR: Topic: /vacuum/pressure_raw
    GR-->>ST: Feedback(stage=PLAN/MOVE/VACUUM/...)
    GR-->>ST: Result(SUCCESS_BOTH)
    Note over ST: UPDATE_GRID + DECIDE_NEXT_PAIR
    ST->>ST: 继续下一 pair...
    Note over ST: 直到当前 phase 完成
    ST->>NV: Action: navigate(RIGHT_PHASE)
    Note over ST: 重复直到 wall 完成
    Note over ST: 多帧无箱确认
    ST-->>TM: Action Result(success)
    TM-->>U: /task/start response
```

---

## 13. 版本

- v1.0 初版 2026-05-23
- 状态名变更必须同步改 02 状态规格手册与本文档的 Mermaid
