# pair_grasp_execution 设计笔记

版本：2026-05-26

## 1. 当前 M1 边界

`pair_grasp_execution_node` 在 M1 只完成端口骨架，不接真实 MoveIt / 控制器。

当前已具备：

- `/execute_pair_grasp` Action server 注册。
- `/vacuum/cmd` publisher。
- `/vacuum/pressure_raw` subscriber。
- `/vacuum/pressure` forward publisher。
- 启动、参数、心跳、状态广播基础能力。

当前未实现：

- IK 求解。
- 轨迹规划。
- 碰撞检查。
- 双臂同步执行。
- 真空吸附闭环。
- 放置与退回动作。

因此 M1 阶段抓取闭环必须使用 `mock_pair_grasp_execution_node`，真实抓取适配在 M2/M3 落地。

## 2. 对外接口

### Action：`/execute_pair_grasp`

输入：

- `GraspPair`
- `timeout_sec`
- `dry_run`

输出：

- `success`
- `PairGraspResult`
- `failed_stage`
- `error_code`

反馈阶段建议保持与 mock 一致，方便 UI 和 smoke 复用：

```text
RECEIVE_PAIR
CHECK_PAIR_VALID
PLAN_PREGRASP
MOVE_TO_PREGRASP
APPROACH_AND_CONTACT
CHECK_VACUUM
ATTACH_BOX_MODEL
PLAN_EXTRACT
EXECUTE_EXTRACT
PLAN_CARRY
EXECUTE_CARRY
RELEASE_BOX
RETREAT_SAFE
REPORT
```

### Vacuum IO

`pair_grasp_execution_node` 不直接读硬件泵阀，统一通过 `vacuum_io_node`：

- 发 `/vacuum/cmd`
- 读 `/vacuum/pressure_raw`
- 对外转发 `/vacuum/pressure`

这样 mock 与真机可以共用同一 action 行为边界。

## 3. M2 对接计划

1. GraspPair 校验。
   - 校验 slot id、左右 pose、box size、grasp_mode。
   - 失败映射 `E_GRASP_INVALID_PAIR=5010`。

2. IK / 轨迹规划。
   - 左右臂分别求 IK。
   - 双臂模式做同步规划。
   - IK 失败映射 `E_PLAN_IK_FAIL=5200`。
   - 轨迹失败映射 `E_PLAN_TRAJ_FAIL=5201`。
   - 碰撞失败映射 `E_PLAN_COLLISION_DETECTED=5210`。

3. 运动执行。
   - pregrasp、approach、extract、carry、retreat 分阶段执行。
   - 控制器超时或失败映射 `E_MOT_EXEC_FAIL=5300`。
   - 掉箱映射 `E_MOT_DROP_BOX=5310`。

4. 真空闭环。
   - `CHECK_VACUUM` 阶段要求压力达到 `business.vacuum.attach_threshold_kpa`。
   - 建压失败映射 `E_VAC_NOT_REACHED=5105`。
   - 单侧失败映射 `E_VAC_UNILATERAL_FAIL=5106`。
   - 搬运中丢真空映射 `E_VAC_LOST_DURING_CARRY=5103`。

5. cancel / estop。
   - action cancel 必须停止当前轨迹。
   - estop 时必须立即停止执行，并按 `hold_on_estop` 决定是否保持真空。

## 4. 错误码约定

| 场景 | 错误码 | recovery |
|---|---:|---|
| action rejected | 5000 | RETRY_CURRENT_STATE |
| action timeout | 5001 | RETRY_CURRENT_STATE |
| invalid pair | 5010 | SWITCH_TARGET |
| vacuum not reached | 5105 | RETRY_CURRENT_STATE |
| unilateral vacuum fail | 5106 | SWITCH_TARGET |
| vacuum lost during carry | 5103 | WAIT_MANUAL_RECOVERY |
| IK fail | 5200 | SWITCH_TARGET |
| trajectory fail | 5201 | REPLAN |
| collision detected | 5210 | REPLAN |
| motion exec fail | 5300 | RETREAT_SAFE |
| drop box | 5310 | WAIT_MANUAL_RECOVERY |
| place fail | 5320 | RETRY_CURRENT_STATE |

## 5. 风险

- 双臂同步规划和单臂 fallback 的决策边界还需要真机工作空间标定。
- 真空压力阈值必须按真实泵、吸盘、箱体材质重新标定，不能沿用 mock 默认值。
- `dry_run` 真机语义必须冻结：建议只规划和运动到安全预抓位，不吸附、不搬运。
