# pair_grasp_execution 设计笔记

版本：2026-05-27

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

## 1.1 M2-preintegration 当前状态

当前 `pair_grasp_execution_node` 已不再只是 M1 骨架：

- `/execute_pair_grasp` 已实现 14 阶段主线反馈，阶段名与 mock 保持一致。
- 已实现 GraspPair 入参校验，结构化返回 `E_GRASP_INVALID_PAIR=5010`。
- `backend_mode=dry_run`：只跑状态主线和接口反馈，不调用真实控制器。
- `backend_mode=fake_real`：使用内部 fake MoveIt 后端注入 `IK_FAIL`、`TRAJ_FAIL`、`COLLISION`、`MOVE_FAIL`、`VACUUM_NOT_REACHED`。
- `backend_mode=real`：已接 `moveit_msgs/action/MoveGroup` client，action 名默认 `interfaces.actions.moveit_move_group=/move_action`；当前是 MoveIt 接入骨架，真实约束构造和控制器执行仍需要 L3-SIM-03 轻量物理仿真 / 真机 dry-run 验证。
- cancel / estop 已在主线内处理；cancel 返回 `CANCELLED`，estop 返回 `ESTOP` 并按 `hold_on_estop` 决定真空保持策略。
- 当前仍不实现真实吸盘硬件后端，只保留 `/vacuum/cmd` 和 `/vacuum/pressure` 转发。

离线验收：

- `scripts/m2_pre_grasp_fake_real_smoke.py`

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

## 3.1 M2-SIM 验收轨道

M2-SIM / L3-SIM 用 MoveIt 2 + ros2_control 验证 `pair_grasp_execution_node` 的 IK、轨迹规划和 dry_run 阶段划分。L3-SIM-03 第一版进一步引入 Gazebo 轻量物理场景，验证机器人、场地、纸箱和控制链路能共同启动并完成规划/预抓主线。它仍不证明真实吸附可靠性；“接触条件 + 固定约束”的仿真抓取桥接器放在阶段 B。

| 用例 | 组合 | 验收点 |
|---|---|---|
| L3-SIM-03 | 真实 `pair_grasp_execution_node` + MoveIt 2 + ros2_control + Gazebo 轻量物理场景 | 标准 pair 能完成规划、接近、预抓 dry_run；工作空间边界 pair 能触发 5200/5201/5210 中对应错误 |
| L3-SIM-03-B | L3-SIM-03 + 仿真抓取桥接器 | TCP / box 满足接触或距离条件后创建固定约束；释放时解除约束；失败映射既有抓取错误码 |
| L3-SIM-04 | sim perception + real navigation adapter + real grasp dry_run | 抓取 adapter 能在完整任务上下文中接收 strategy 选出的 GraspPair，反馈阶段与 mock 保持一致 |

实现约束：

- `dry_run=true` 时只允许规划和运动到安全预抓/验证姿态，不吸附、不搬运真实箱体。
- MoveIt mock hardware、Gazebo 仿真控制链和仿真抓取桥接器只在测试 launch 中启用，生产节点仍通过既定 MoveIt / 控制器 / vacuum 接口工作。
- L3-SIM-03 通过后，M2-C03 仍必须跑 L3-M2-GRASP-DRY；仿真只提前暴露 IK、规划、场景碰撞和控制链路配置边界。

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
- L3-SIM-03 能提前发现 IK/规划不可达、场景碰撞和控制链路配置问题，但不能证明真实控制器跟随、吸附接触和碰撞监控可靠。
- 真空压力阈值必须按真实泵、吸盘、箱体材质重新标定，不能沿用 mock 默认值。
- `dry_run` 真机语义必须冻结：建议只规划和运动到安全预抓位，不吸附、不搬运。
