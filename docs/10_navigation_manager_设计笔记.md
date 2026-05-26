# navigation_manager 设计笔记

版本：2026-05-26

## 1. 当前 M1 边界

`navigation_manager_node` 在 M1 只完成端口骨架，不接真实 Nav2。

当前已具备：

- `/navigate_to_pose` Action server 注册。
- `/nav/base_recovery` Service 注册。
- `/fsm/nav_health` 1Hz 发布。
- 启动、参数、心跳、状态广播基础能力。

当前未实现：

- Nav2 `NavigateToPose` action client。
- AMCL / localization health 聚合。
- FINE_ALIGN 闭环控制。
- twist_mux `/estop`、chassis enable/reset 的真实调用。

因此 M1 阶段真实任务闭环必须使用 `mock_navigation_manager_node`，`navigation_manager_node` 仅作为 M2 真实适配层占位。

## 2. 对外接口

### Action：`/navigate_to_pose`

输入来自 `wall_destacking_strategy_node`，包括：

- `goal_type`：观察位、左 phase 工位、右 phase 工位。
- `target_pose`
- `phase`
- `require_fine_alignment`
- `timeout_sec`

输出必须统一到 `fsm_msgs/action/NavigateToPose`：

- `success`
- `actual_base_pose`
- `position_error`
- `yaw_error`
- `alignment_error`
- `workpose_valid`
- `error_code`
- `failure_reason`

### Service：`/nav/base_recovery`

仅允许 `task_manager_node` 在 `/clear_error` 五阶段协议中调用。

命令映射：

- `RELEASE_ESTOP`：释放底盘 / twist_mux 急停锁。
- `RESET_FAULT`：调用底盘故障复位。
- `ENABLE_CHASSIS`：底盘重新使能。

### Topic：`/fsm/nav_health`

M2 中应由以下信号合成：

- Nav2 lifecycle active。
- AMCL / localization 可用。
- chassis enabled。
- FINE_ALIGN 反馈链路可用。

## 3. M2 对接计划

1. 接 Nav2 action client。
   - `goal_type=OBSERVATION/LEFT_PHASE/RIGHT_PHASE` 转为 Nav2 pose goal。
   - Nav2 action 超时映射到 `E_NAV_GOAL_TIMEOUT=4001`。
   - goal reject 映射到 `E_NAV_GOAL_REJECTED=4000`。

2. 接 FINE_ALIGN。
   - Nav2 到粗工位后进入 fine align。
   - 发布 `/cmd_vel_align`。
   - 订阅视觉或激光对墙误差反馈。
   - 收敛失败映射到 `E_NAV_FINE_ALIGN_FAIL=4040`。

3. 接底盘恢复。
   - `/clear_error` 第 1/3/4 阶段实际动作在本节点执行。
   - 每阶段必须有超时和错误码，不允许直接返回 success。

4. 接 health 聚合。
   - lifecycle inactive 映射 `E_NAV_LIFECYCLE_NOT_ACTIVE=4050`。
   - localization lost 映射 `E_NAV_LOCALIZATION_LOST=4010`。

## 4. 错误码约定

| 场景 | 错误码 | recovery |
|---|---:|---|
| Nav goal rejected | 4000 | RETRY_CURRENT_STATE |
| Nav goal timeout | 4001 | RETRY_CURRENT_STATE |
| localization lost | 4010 | RELOCALIZE |
| path plan fail | 4020 | REPLAN |
| chassis stuck | 4022 | RETREAT_SAFE |
| goal unreachable | 4030 | WAIT_MANUAL_RECOVERY |
| fine align fail | 4040 | RETRY_CURRENT_STATE |
| lifecycle inactive | 4050 | WAIT_MANUAL_RECOVERY |

## 5. 风险

- FINE_ALIGN 反馈源尚未冻结：可能来自 perception_adapter 的墙面法向，也可能来自激光最近点。
- `/clear_error` 第 1 阶段需要和 twist_mux 实际锁语义对齐，不能只发 `/estop=False` 就认为成功。
- 真机 Nav2 的 action result 与当前 mock 阶段序列不一定一致，M2 需要补状态映射表。
