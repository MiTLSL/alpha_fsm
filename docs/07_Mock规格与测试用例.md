# 07 Mock 规格与测试用例

> 本文档定义所有 mock 节点的行为契约、测试分层与必跑用例。**目标是让 FSM 在硬件未就位、A/B 既有代码未对接前就能完成端到端闭环测试与错误恢复演练**。
>
> 配套阅读：
> - 03 接口契约（msg/srv/action 字段）
> - 04 错误码（每个 mock 必须能按需注入这些码）
> - 05 配置（mock 阈值、QoS、launch profile）
> - `codes/IK、视觉雷达感知与底盘导航代码导览.md`（上游真实模样，mock 设计参考）

---

## 0. 设计原则

```text
1. mock 与真实节点共享同一份 fsm_msgs 与 03 接口契约。FSM 业务代码不区分对接的是 mock 还是真实。
2. mock 一律放进独立包 fsm_test/，不污染生产包。
3. 每个 mock 对外接口必须 1:1 对齐真实适配节点。差异只在内部实现。
4. 每个 mock 必须支持"故障注入"：通过参数或 Service 切换返回失败、超时、错误码。
5. 单测优先（dataclass、状态类）；集成测试覆盖跨节点；端到端只做关键 happy path + clear_error。
6. 每条 P0 错误码必须至少有一条测试用例覆盖恢复路径。
```

---

## 1. 测试分层

```text
L0  纯单元测试（pytest，无 ROS）
    - 状态类的 execute() 输入输出
    - dataclass 的 to_msg/from_msg
    - ErrorHandler 的 recovery 选择逻辑
    - 网格匹配/Pair 选择/可达性算法

L1  节点单元测试（launch_pytest，单节点）
    - 节点起来后是否声明所有 declared_parameters
    - 必须的 Topic/Service/Action 是否注册
    - 状态广播是否符合 QoS 与频率

L2  双节点对接测试（launch_pytest，2-3 节点）
    - 策略 ↔ navigation_manager（mock）：导航 happy / fail / fine_align
    - 策略 ↔ pair_grasp_execution（mock）：抓取 happy / vacuum_fail / drop_box
    - 策略 ↔ perception_adapter（mock）：感知 happy / lidar_offline
    - clear_error 五阶段的逐阶段失败注入

L3  端到端测试（bringup_with_mock）
    - 从 /task/start 到 /task 完成的完整链路
    - 急停 → clear_error → 重新跑任务

L4  现场联调（bringup_full）
    - 真实 perception / Nav2 / chassis / 机械臂
    - 不在本期文档范围（属于上线流程）
```

每层用例必须**独立可跑**，不依赖上一层。L0/L1 在 CI 跑；L2/L3 本地或在带显示的 runner 跑。

---

## 2. mock 节点总览

| mock 节点 | 替代的真实节点 | 主要用途 | 故障注入 Service |
|---|---|---|---|
| `mock_perception_adapter_node` | perception_adapter_node + 上游 box_perception | 模拟连续 box 检测流 + health | `~/inject_failure` |
| `mock_navigation_manager_node` | navigation_manager_node + Nav2 + AMCL + chassis | 模拟 NavigateToPose Action + base_recovery | `~/inject_failure` |
| `mock_pair_grasp_execution_node` | pair_grasp_execution_node + MoveIt + 控制器 | 模拟 ExecutePairGrasp Action | `~/inject_failure` |
| `mock_vacuum_io_node` | vacuum_io_node 真实硬件后端 | 真实 vacuum_io_node 默认就是 mock first，本节专列其行为 | `~/inject_failure` |
| `mock_chassis_status_publisher` | chassis_node 的 `/chassis_node/status` | 用于测 navigation_manager 自身（不替代它） | `~/set_status` |
| `mock_safety_button` | 硬件急停按钮 | 推 `/estop_button` 升降沿，测 EMERGENCY → clear_error | `~/press` / `~/release` |

> mock 节点全部放在 `fsm_test/fsm_test/mocks/` 包内，由 `bringup_with_mock.launch.py` 通过 LaunchArguments 选择启用。

---

## 3. mock_perception_adapter_node 规格

### 3.1 对外接口（与真实 perception_adapter_node 一致）

| 端口 | 类型 | Dir | 行为 |
|---|---|---|---|
| `/perception/box_detections` | `fsm_msgs/BoxDetectionArray` | P | 默认 10Hz；frame_id="base_link"；`detections[]` 内容由 mode 决定 |
| `/perception/health` | `fsm_msgs/PerceptionHealth` | P | 默认 1Hz，全部 ok=true，error_code=0 |

### 3.2 mode 参数（启动 / 运行时切换）

```yaml
mock_perception_adapter:
  ros__parameters:
    mode: "OBSERVATION"                   # 见下方枚举
    publish_rate_hz: 10.0
    health_rate_hz: 1.0
    deterministic_seed: 42                # 复现
```

| mode | 输出语义 | 用途 |
|---|---|---|
| `OBSERVATION` | 模拟 5×5 完整墙面：25 个 box 在 base_link 下（X≈1.5m 处），加 ±2cm 噪声 | WallMappingFSM happy |
| `LEFT_PHASE` | 仅 left_phase_cols 内的 box | PhasePerceptionFSM happy |
| `RIGHT_PHASE` | 仅 right_phase_cols 内的 box | PhasePerceptionFSM happy |
| `EMPTY` | `detections[]` 为空 | VERIFY_TASK_COMPLETE happy |
| `PARTIAL` | 仅给前 row 的 5 个 box | E_MAP_INSUFFICIENT_DETECTION 边界 |
| `JITTER` | 在 OBSERVATION 基础上每帧随机抖动 ±15cm | 验证 PhasePerceptionFSM 多帧融合 |

mode 切换通过参数 `mode` reload 或 `~/set_mode` Service。

### 3.3 故障注入

`~/inject_failure` Service（`fsm_msgs/InjectFailure` 自定义，见 §9）：

| failure | 行为 |
|---|---|
| `STOP_PUBLISHING` | 停发 `/perception/box_detections` N 秒 |
| `CAMERA_OFFLINE` | health.camera_ok=false、error_code=8010 |
| `LIDAR_OFFLINE` | health.lidar_ok=false、error_code=8011 |
| `YOLO_FALLBACK` | health.yolo_ok=false、error_code=8012；继续发 detections 但 confidence 全部 < 阈值 |
| `TF_TIMEOUT` | health.tf_ok=false、error_code=9010 |
| `RATE_LOW` | 频率降到 1Hz，health.error_code=8020（WARN） |
| `INVALID_FRAME` | header.frame_id 设成 `body`（错误 frame，触发 strategy 端校验） |
| `NONE` | 恢复正常 |

**复位**：Service 调 `failure=NONE` 或 60s 自动复位（避免测试用例污染）。

### 3.4 与真实 adapter 的差异

```text
真实 adapter 做：上游 BoxResult → 6D pose 反推 → frame 转 → 重新打包
mock     adapter 做：直接按 grid_shape + box_size 合成 BoxDetection

行为对策略层完全等价。区别只在没有真实图像/点云延迟。
```

---

## 4. mock_navigation_manager_node 规格

### 4.1 对外接口

`/navigate_to_pose` Action server（`fsm_msgs/NavigateToPose`）。Goal 字段全部按 03 §2.3.2 处理。

`/nav/base_recovery` Service server（`fsm_msgs/BaseRecoveryCommand`）。

`/fsm/nav_health` Topic 1Hz 心跳，默认 True。

### 4.2 默认 happy path

收到 goal → Feedback 阶段序列（每阶段间隔 200ms）：

```text
RECEIVE_GOAL → CHECK_PRECONDITION → CHECK_LOCALIZATION
  → PLAN_PATH → EXECUTE → MONITOR
  [ require_fine_alignment=true 才进 ] → FINE_ALIGN
  → VERIFY → REPORT
```

总耗时由 `nominal_duration_sec` 参数控制（默认 3s OBSERVATION、5s LEFT/RIGHT_PHASE）。

Result：

```python
success = True
actual_base_pose = goal.target_pose（不加噪声 / 加 ±2cm 噪声 by mock_pose_noise param）
position_error = 0.01
yaw_error = 0.005
alignment_error = 0.005 if require_fine_alignment else NaN
workpose_valid = True
error_code = 0
```

### 4.3 故障注入

| failure | 触发的内部行为 | 返回的 error_code |
|---|---|---|
| `GOAL_REJECTED` | 直接 reject goal | 4000 (E_NAV_GOAL_REJECTED) |
| `GOAL_TIMEOUT` | feedback 跑到 EXECUTE 后停在该状态不发 result，直到 client 端超时 | 4001 |
| `LOCALIZATION_LOST` | `/fsm/nav_health=False`；result.error_code=4010 | 4010 |
| `PATH_PLAN_FAIL` | 进 PLAN_PATH 后立即失败 | 4020 |
| `STUCK` | EXECUTE 期间 `/fsm/nav_health=True` 但 result 长时间不返回；最终 4022 | 4022 |
| `FINE_ALIGN_FAIL` | 进 FINE_ALIGN 后 alignment_error_current 一直 > tol，超时返回 4040 | 4040 |
| `FINE_ALIGN_NO_FEEDBACK` | 类似上面但 error_code 为 4042 | 4042 |
| `LIFECYCLE_INACTIVE` | `is_active` 内部状态返回 False，`/fsm/nav_health=False` | 4050 |
| `CHASSIS_ENABLE_FAIL` | `BaseRecoveryCommand(ENABLE_CHASSIS)` 返回 success=false、stage_reached=4 | 4060 |
| `CHASSIS_FAULT_RESET_FAIL` | `BaseRecoveryCommand(RESET_FAULT)` 返回 success=false、stage_reached=3 | 4061 |
| `ESTOP_LOCK_STUCK` | `BaseRecoveryCommand(RELEASE_ESTOP)` 返回 success=false、stage_reached=1 | 7040 |

### 4.4 BaseRecoveryCommand mock 行为

| command | happy path | 故障注入 |
|---|---|---|
| `RELEASE_ESTOP` | 立刻发 `/estop=False`，0.5s 后返回 success=true、stage=1 | `ESTOP_LOCK_STUCK` 时返回 success=false |
| `RESET_FAULT` | 立刻"调"虚假 reset_fault，1s 后返回 success=true、stage=3 | `CHASSIS_FAULT_RESET_FAIL` 时返回 success=false |
| `ENABLE_CHASSIS` | 1s 后返回 success=true、stage=4 | `CHASSIS_ENABLE_FAIL` 时返回 success=false |

mock 内部维护一个 `_chassis_enabled` 布尔，跟随 ENABLE/DISABLE 命令切换；测试用例可读 `~/state` Service 检查。

### 4.5 cancel 行为

收到 cancel → 在 ≤2s 内返回 result（success=false、error_code=4002 `E_NAV_GOAL_CANCELLED`）。用户取消本身由 task_manager 记录 `E_TASK_CANCELLED` / `E_MAN_CANCELLED`，navigation mock 不复用 2xxx/99xx。

### 4.6 急停响应

订 `/safety/estop`（Bool）。`True` 升降沿 → 当前 active goal 立即 cancel + result.error_code=7000 系列；`False` 不自动重置任何东西，由 clear_error 协议驱动。

---

## 5. mock_pair_grasp_execution_node 规格

### 5.1 对外接口

`/execute_pair_grasp` Action server（`fsm_msgs/ExecutePairGrasp`）。

订 `/vacuum/pressure_raw`（来自 mock_vacuum_io_node 或真实），转发为 `/vacuum/pressure`。

### 5.2 默认 happy path

收到 pair → Feedback 阶段序列（每阶段 300ms）：

```text
RECEIVE_PAIR → CHECK_PAIR_VALID → PLAN_PREGRASP → MOVE_TO_PREGRASP
  → APPROACH_AND_CONTACT → CHECK_VACUUM
  [ 此处发 /vacuum/cmd 给 mock_vacuum_io_node，等 pressure ≤ -50kPa ]
  → ATTACH_BOX_MODEL → PLAN_EXTRACT → EXECUTE_EXTRACT
  → PLAN_CARRY → EXECUTE_CARRY → RELEASE_BOX → RETREAT_SAFE → REPORT
```

`vacuum_left_kpa` / `vacuum_right_kpa` 在 feedback 中实时填，取自最近一帧 `/vacuum/pressure_raw`。

Result（happy）：`SUCCESS_BOTH`，`vacuum_left_kpa = vacuum_right_kpa = -60.0`，`execution_time_sec = 累计 feedback 时长`。

### 5.3 故障注入

| failure | 行为 | result_code | error_code |
|---|---|---|---|
| `GOAL_REJECTED` | reject | - | 5000 |
| `IK_FAIL` | 进 PLAN_PREGRASP 后失败 | FAILED_BOTH | 5200 |
| `TRAJ_FAIL` | 进 PLAN_EXTRACT 后失败 | FAILED_BOTH | 5201 |
| `COLLISION` | PLAN_CARRY 后失败 | FAILED_BOTH | 5210 |
| `MOVE_FAIL` | EXECUTE_EXTRACT 失败 | FAILED_BOTH | 5300 |
| `VACUUM_NOT_REACHED` | 让 mock_vacuum 不要建压（或注入 LEAK） | FAILED_BOTH | 5105 |
| `VACUUM_UNILATERAL` | 让 mock_vacuum 仅一侧建压 | SUCCESS_LEFT_ONLY 或 RIGHT_ONLY 或 FAILED_BOTH（按 grasp_mode） | 5106 |
| `VACUUM_LOST_DURING_CARRY` | EXECUTE_CARRY 中途让 mock_vacuum 释放 | FAILED_BOTH | 5103 |
| `DROP_BOX` | EXECUTE_CARRY 后期 → FATAL | FAILED_BOTH | 5310 |
| `PLACE_FAIL` | RELEASE_BOX 失败 | FAILED_BOTH | 5320 |
| `TIMEOUT` | 任意阶段不返回，触发 client 端超时 | - | 5001 |

### 5.4 dry_run

`Goal.dry_run=true` 时：

- 走完所有 Feedback 阶段
- **不发** `/vacuum/cmd`（不触发 mock_vacuum_io_node）
- Result 按 happy / 注入决定，但 `vacuum_*_kpa = 0.0`、`execution_time_sec = 模拟规划耗时`

### 5.5 cancel 与急停

cancel：当前阶段后立即停，发 `/vacuum/cmd=[false, false]`（除非 `business.vacuum.release_on_cancel=false`），返回 `result.result_code = CANCELLED`、success=false（与 03 §6.5 一致）。

急停：同 cancel，但 `result.error_code=7000` 系列，且 `/vacuum/cmd` 是否释放看 `business.vacuum.hold_on_estop`（默认 true → 不释放）。

---

## 6. vacuum_io_node mock first 规格

> 本节点真实生产版本的 P0 实现就是这里描述的 mock。硬件后端到位后**保持本节接口不变**，只换内部驱动。

### 6.1 对外接口（生产版与 mock 完全一致）

| 端口 | 类型 | Dir | 行为 |
|---|---|---|---|
| `/vacuum/cmd` | `fsm_msgs/VacuumCommand` | Sub | 由 pair_grasp_execution_node 发，10Hz；`left_on / right_on` 控开关 |
| `/vacuum/pressure_raw` | `std_msgs/Float32MultiArray` | P | `[left_kpa, right_kpa]`，20Hz |
| `/vacuum/health` | `std_msgs/Bool` | P | 1Hz，sensor 在线即 True |

### 6.2 压力曲线模型

参数（`business.yaml` § vacuum）：

```yaml
vacuum:
  attach_threshold_kpa: -50.0
  hold_ms: 150
  buildup_timeout_ms: 800
  sensor_timeout_ms: 200
  pressure_publish_rate_hz: 20.0
  mock_buildup_time_ms: 150
  mock_release_time_ms: 100
```

模型（每路独立）：

```text
on_cmd[t] 从 false → true：
  pressure[t] = -60.0 * (1 - exp(-(t - t_on) / tau_buildup))
  其中 tau_buildup = mock_buildup_time_ms / 3   (取 3τ ≈ 充满)

on_cmd[t] 从 true → false：
  pressure[t] = pressure[t_off] * exp(-(t - t_off) / tau_release)
  其中 tau_release = mock_release_time_ms / 3

正常稳态：约 -60 kPa；on→off 后 ~300ms 回到 0
```

启动时 `pressure = 0.0`、`/vacuum/health = True`。

### 6.3 故障注入

| failure | 行为 |
|---|---|
| `LEFT_NEVER_BUILDUP` | left 路 cmd=true 时 pressure 卡在 -10 kPa（远不到阈值） |
| `RIGHT_NEVER_BUILDUP` | 同上但右路 |
| `LEAK_AFTER_ATTACH` | 建压成功后 500ms 缓慢漏到 -20 kPa |
| `SENSOR_OFFLINE` | 停止发 `/vacuum/pressure_raw`、`/vacuum/health=False` |
| `RELEASE_TOO_SLOW` | release 时间从 100ms → 2s |
| `NONE` | 复位 |

### 6.4 命令源审计

`/vacuum/cmd.command_source` 字段（03 §2.1.6a）允许观察是谁在发：

- `SOURCE_PAIR_GRASP=0`：正常抓取
- `SOURCE_SAFETY=1`：safety_monitor 急停时切（如果 `hold_on_estop=false`）
- `SOURCE_MANUAL=2`：UI / CLI 手动
- `SOURCE_TEST=3`：测试脚本

mock_vacuum_io_node 不基于 source 区别行为，但记录到自己 log 便于复盘。

---

## 7. mock_safety_button 规格

最简单的一个 mock，用来在测试中触发 SafetyMonitorFSM 的 EMERGENCY。

| 端口 | 类型 | Dir | 行为 |
|---|---|---|---|
| `/estop_button` | `std_msgs/Bool` | P | 由 Service 控制升降沿 |

Service：

- `~/press` (`std_srvs/Trigger`)：发 `/estop_button=True`
- `~/release` (`std_srvs/Trigger`)：发 `/estop_button=False`

测试用例直接 `ros2 service call` 即可。

> 注意：`/estop_button` 不直接进 twist_mux；safety_monitor_node 订它后再合成 `/safety/estop` 与 `/estop`。

---

## 8. 测试用例集（必跑）

### 8.1 L0 单元测试（pytest）

| 用例 ID | 模块 | 测的内容 |
|---|---|---|
| L0-FSM-01 | `fsm_core.fsm_engine` | tick 一步：状态返回 SUCCESS 后切 next_state，触发 on_exit/on_enter |
| L0-FSM-02 | `fsm_core.fsm_engine` | 超时：超过 timeout_sec 时调 on_timeout 并切到失败分支 |
| L0-FSM-03 | `fsm_core.fsm_engine` | abort：request_abort 后下一 tick 走 on_abort，标记 FSM 结束 |
| L0-ERR-01 | `fsm_core.error_code` | 每个 ErrorCode 在 ERROR_TABLE 都有 meta，code 与 key 一致 |
| L0-ERR-02 | `fsm_core.recovery_policy` | error_codes.yaml overrides 优先于默认 recovery |
| L0-DAT-01 | `wall_grid` | 25 个 slot row-major 索引一致；4 个角的 (row,col) 正确 |
| L0-DAT-02 | `grasp_pair` | 单臂模式下未使用 slot 字段全 0；左手 Y > 右手 Y 不变式 |
| L0-MAP-01 | WallMappingFSM 算法 | RANSAC 拟合 wall_frame：给定 25 个标准箱中心点能拟出朝向 |
| L0-MAP-02 | 同上 | 缺失 5 个 box（INSUFFICIENT_DETECTION）时拟合失败返 3111 |
| L0-PAIR-01 | PairSelectionFSM | 给定 5×5 grid、左 phase 满，按 row 升序选出第一对 |
| L0-PAIR-02 | 同上 | 仅剩 1 个箱 + allow_single=false → 返回 3340 |
| L0-PAIR-03 | 同上 | 候选都不可达 → 返回 3310 |
| L0-CLR-01 | `clear_error` 协议 | 五阶段顺序的状态机：每阶段失败时 stage_reached 正确 |

### 8.2 L1 节点单元测试（launch_pytest，单节点）

| 用例 ID | 节点 | 测的内容 |
|---|---|---|
| L1-NODE-01 | task_manager_node | 启动后声明所有 declared_parameters，无 KeyError |
| L1-NODE-02 | task_manager_node | `/task/start` Service handler 注册；`/clear_error` Service handler 注册 |
| L1-NODE-03 | strategy_node | `/run_wall_destacking` Action server 注册；订阅 `/perception/box_detections` |
| L1-NODE-04 | navigation_manager_node | `/navigate_to_pose` Action server + `/nav/base_recovery` Service server 注册 |
| L1-NODE-05 | perception_adapter_node | 上游 `/box_perception/result` 不发时，1Hz 仍发 `/perception/health(camera_ok=false 等)` |
| L1-QOS-01 | broadcaster | `/fsm/system_state` QoS = RELIABLE + TRANSIENT_LOCAL + Depth=1 |
| L1-QOS-02 | broadcaster | 晚启动的订阅者能收到最新一帧（验证 TRANSIENT_LOCAL 实际生效）|

### 8.3 L2 双节点对接测试

| 用例 ID | 范围 | 步骤 / 预期 |
|---|---|---|
| L2-NAV-01 | strategy ↔ mock_navigation_manager | strategy 发 `goal_type=OBSERVATION` → mock 默认 happy 返回 success；strategy 进 RUN_WALL_MAPPING |
| L2-NAV-02 | 同上 | 注入 `LOCALIZATION_LOST` → strategy 收到 4010 → 跳 WallRecoveryFSM → recovery_action=RELOCALIZE |
| L2-NAV-03 | 同上 | `goal_type=LEFT_PHASE` + `require_fine_alignment=true` → mock feedback 必须出现 FINE_ALIGN 状态名；alignment_error_current 不为 NaN |
| L2-NAV-04 | 同上 | 注入 `FINE_ALIGN_FAIL` → strategy 收到 4040 → recovery=RETRY_CURRENT_STATE |
| L2-NAV-05 | 同上 | 注入 `STUCK` → 4022 → recovery=RETREAT_SAFE |
| L2-NAV-06 | 同上 | 注入 `GOAL_TIMEOUT` → strategy 超时 cancel goal，上报 4001 |
| L2-NAV-07 | 同上 | 注入 `PATH_PLAN_FAIL` → strategy 收到 4020 → recovery=REPLAN |
| L2-NAV-08 | 同上 | 注入 `LIFECYCLE_INACTIVE` → `/fsm/nav_health=false`，strategy / safety 收到 4050 |
| L2-PERC-01 | strategy ↔ mock_perception_adapter | mode=OBSERVATION → WallMappingFSM 完整跑完，grid.slots 25 个 OCCUPIED |
| L2-PERC-02 | 同上 | mode=PARTIAL → INSUFFICIENT_DETECTION (3111) |
| L2-PERC-03 | 同上 | 注入 `LIDAR_OFFLINE` → strategy 收 8011 → 上报 ERROR |
| L2-PERC-04 | 同上 | 注入 `INVALID_FRAME` → strategy 端 frame 校验拒绝该帧（写日志，不上报） |
| L2-PERC-05 | 同上 | 注入 `STOP_PUBLISHING` → WallMappingFSM 开窗失败，strategy 上报 3100 |
| L2-PERC-06 | 同上 | mode=EMPTY 且当前 phase 仍应有箱 → PhasePerceptionFSM 上报 3210 |
| L2-PERC-07 | 同上 | 注入 `CAMERA_OFFLINE` → strategy / safety 收到 8010 |
| L2-GRASP-01 | strategy ↔ mock_pair_grasp + mock_vacuum_io | 完整 happy → SUCCESS_BOTH，pressure 峰值 ≤ -50 kPa |
| L2-GRASP-02 | 同上 | mock_vacuum 注入 `LEFT_NEVER_BUILDUP` → result_code=FAILED_BOTH 或 SUCCESS_RIGHT_ONLY（按 grasp_mode） |
| L2-GRASP-03 | 同上 | mock_pair_grasp 注入 `DROP_BOX` → FATAL，strategy 上报 5310 |
| L2-GRASP-04 | 同上 | dry_run=true → result.success=true，未触发 vacuum mock |
| L2-GRASP-05 | 同上 | mock_pair_grasp 注入 `VACUUM_UNILATERAL` → result.error_code=5106，strategy 进入 SWITCH_TARGET |
| L2-GRASP-06 | 同上 | mock_pair_grasp 注入 `VACUUM_LOST_DURING_CARRY` → result.error_code=5103，strategy 上报 FATAL |
| L2-CLR-01 | task_manager + mock_navigation_manager | E_STOP 进入后调 `/clear_error`：① mock 全 happy → 五阶段都成 → 进 SELF_CHECK |
| L2-CLR-02 | 同上 | 注入 `ESTOP_LOCK_STUCK` → cleared=false、stage_reached=1、error_code=7040 |
| L2-CLR-03 | 同上 | 注入 `CHASSIS_FAULT_RESET_FAIL` → stage_reached=3、error_code=4061 |
| L2-CLR-04 | 同上 | 注入 `CHASSIS_ENABLE_FAIL` → stage_reached=4、error_code=4060 |
| L2-CLR-05 | 同上 | 阶段 2 cancel 全部超时 → stage_reached=2、error_code=1004 |
| L2-SAFETY-01 | safety_monitor + mock_safety_button | 按下按钮 → 0.5s 内 SafetyStatus.estop=true，`/safety/estop` latched |
| L2-SAFETY-02 | 同上 | 释放按钮 + 调 `/clear_error` → 五阶段后回 NORMAL |

### 8.4 L3 端到端用例（bringup_with_mock）

| 用例 ID | 场景 | 步骤 / 预期 |
|---|---|---|
| L3-E2E-01 | 单墙 happy（左 phase 完成、右 phase 完成、wall 完成）| `/task/start` → 完整跑；mock_perception 在 mode=OBSERVATION→LEFT_PHASE→RIGHT_PHASE→EMPTY 之间切换；最后任务 success=true |
| L3-E2E-02 | 中途单 pair 失败 + 重试成功 | mock_pair_grasp 第一次注入 IK_FAIL，第二次 happy → 最终 wall 完成 |
| L3-E2E-03 | 中途急停 + 清错 + 续跑 | 任务进行中按急停 → strategy abort → /clear_error 五阶段 → SELF_CHECK → STANDBY → /task/start 重新跑 |
| L3-E2E-04 | 中途任务取消 | `/task/cancel` → strategy cancel pair_grasp_action → 任务返回 CANCELLED |
| L3-E2E-05 | 视觉中途断流 | mock_perception 注入 `STOP_PUBLISHING` → strategy 在 RUN_PHASE_PERCEPTION 超时 → recovery=REPERCEPTION 一次后失败 → FATAL |
| L3-E2E-06 | 多墙连续 | mode 序列：W0 OBSERVATION→...→EMPTY、W1 OBSERVATION→...→EMPTY；wall_index 0→1，max_walls=2 后正常完成 |
| L3-E2E-07 | dry_run | `/task/start` 带 `params_json={"dry_run": true}` → 全程 ExecutePairGrasp 跑 dry_run，不触发 vacuum mock，最终 success=true |

### 8.5 错误码覆盖矩阵

下列 P0 错误码必须至少有一条 L2 或 L3 用例命中并验证 recovery：

| code | 用例 |
|---|---|
| 1004 (action cancel timeout) | L2-CLR-05 |
| 3100 (mapping scan fail) | L2-PERC-05 |
| 3111 (insufficient detection) | L2-PERC-02 |
| 3210 (no local detection) | L2-PERC-06 |
| 3300 (no candidate) | L3-E2E-01（每个 phase 收尾时） |
| 4001 (nav timeout) | L2-NAV-06 |
| 4010 (localization lost) | L2-NAV-02 |
| 4020 (path plan fail) | L2-NAV-07 |
| 4022 (stuck) | L2-NAV-05 |
| 4040 (fine align fail) | L2-NAV-04 |
| 4050 (lifecycle inactive) | L2-NAV-08 |
| 4060 (chassis enable fail) | L2-CLR-04 |
| 4061 (chassis fault reset) | L2-CLR-03 |
| 5103 (vacuum lost in carry) | L2-GRASP-06 |
| 5105 (vacuum not reached) | L2-GRASP-02 |
| 5106 (unilateral fail) | L2-GRASP-05 |
| 5200 (IK fail) | L3-E2E-02 |
| 5310 (drop box) | L2-GRASP-03 |
| 7000/7001 (estop) | L2-SAFETY-01 + L3-E2E-03 |
| 7040 (estop lock stuck) | L2-CLR-02 |
| 8010 (camera fail) | L2-PERC-07 |
| 8011 (lidar fail) | L2-PERC-03 |

---

## 9. fsm_test 包结构

```text
fsm_test/
├── package.xml
├── setup.py
├── fsm_test/
│   ├── __init__.py
│   ├── mocks/
│   │   ├── mock_perception_adapter_node.py
│   │   ├── mock_navigation_manager_node.py
│   │   ├── mock_pair_grasp_execution_node.py
│   │   ├── mock_vacuum_io_node.py
│   │   ├── mock_chassis_status_publisher.py
│   │   └── mock_safety_button.py
│   ├── inject/
│   │   └── inject_failure_srv.py        # 共享的 InjectFailure 客户端工具
│   ├── fixtures/
│   │   ├── grids.py                     # 标准 5×5 网格、缺角网格、单列网格
│   │   ├── pairs.py                     # 标准 GraspPair 样例
│   │   └── nav_results.py
│   └── helpers/
│       ├── action_client_helpers.py
│       ├── topic_assertions.py          # 断言"在 N 秒内出现/不出现某 Topic"
│       └── tf_helpers.py
├── launch/
│   ├── bringup_with_mock.launch.py
│   ├── bringup_strategy_only.launch.py
│   └── e2e_single_wall_happy.launch.py
└── test/
    ├── test_unit_*.py                   # L0 用例
    ├── test_node_*.py                   # L1 用例
    ├── test_integration_*.py            # L2 用例
    └── test_e2e_*.py                    # L3 用例
```

### 9.1 InjectFailure.srv（自定义到 fsm_msgs）

```text
# fsm_msgs/srv/InjectFailure.srv
string failure_name                       # 由各 mock 节点定义的字符串集；"NONE" 表示恢复
float32 duration_sec                      # 0 = 永久；>0 = 自动复位
string params_json                        # 参数化注入（如 LEAK_AFTER_ATTACH 的延迟、抖动幅度）
---
bool   accepted
string message
string current_failure                    # 注入前活跃的 failure 名
```

每个 mock 节点 expose 自己的 `~/inject_failure` Service，使用本 srv。

---

## 10. CI 集成

### 10.1 跑哪些层

| 触发 | L0 | L1 | L2 | L3 |
|---|---|---|---|---|
| 每 PR | ✅ | ✅ | ✅（核心 5 条）| ❌ |
| 主分支 push | ✅ | ✅ | ✅（全部）| ✅（核心 3 条 happy）|
| 每日定时 | ✅ | ✅ | ✅ | ✅（全部）|

L4 现场联调不进 CI。

### 10.2 失败时的产物

- L0/L1：pytest 报告
- L2/L3：rosbag2 录所有 `/fsm/*`、`/perception/*`、`/safety/*`、`/vacuum/*` Topic；JSON 日志；时间戳化保留 7 天

### 10.3 性能基准（L3）

- L3-E2E-01 总耗时 ≤ 60s（mock 默认时长，CPU 中位数 < 30%）
- 急停响应：从按下按钮到 strategy 写出"abort 完成"日志 ≤ 200ms
- clear_error happy path：从 `/clear_error` 调用到回 SELF_CHECK ≤ 6s（每阶段 ≤ 2s + 切换 1s）

性能未达标：CI 标黄但不阻塞，告警到日志频道。

---

## 11. 与 04 错误码的协同

mock 与真实节点共用同一个 `ErrorCode` enum 与 `ERROR_TABLE`。mock 的故障注入只是**让某个 code 出现在 result/feedback 里**，不去重定义。这样：

- 测试用例可以直接断言 `result.error_code == ErrorCode.E_NAV_LOCALIZATION_LOST.value`
- mock 不会"造出仅 mock 期才有的错误码"
- 切到真机后，同一条 recovery 路径直接复用

新增 P0 错误码的标准流程：

```text
1. 在 04 §5 加一行（含 level + recovery）
2. 在 fsm_core/error_code.py 的 ERROR_TABLE 加一条
3. 在 mock 的 inject_failure 字符串集里加一项映射到该 code
4. 加一条 L2/L3 用例
```

---

## 12. 待补充

- [ ] mock_chassis_status_publisher 与真实 chassis_node DiagnosticArray 的 KeyValue 字段对齐校验
- [ ] mock_pair_grasp 在 dry_run 时的"规划耗时分布"模型（让 metric 测试有意义）
- [ ] L4 现场联调的检查表（启动顺序、TF 桥接、twist_mux align 通道是否生效）
- [ ] 与 UI（08 文档）联动：mock_perception 的 mode 切换暴露给 UI 调试面板
- [ ] 长稳测试（L3 跑 24h，看是否有内存泄漏 / log 溢出）

---

## 13. 版本

- v1.0 初版 2026-05-23：建立 mock 节点对齐 03 接口契约；P0 错误码全覆盖
- 后续每加一个 P0 错误码，必须同步更新 §8.5 矩阵
