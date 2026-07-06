# navigation_manager 设计笔记

版本：2026-05-27

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

## 1.1 M2-preintegration 当前状态

当前 `navigation_manager_node` 已不再只是 M1 骨架：

- `/navigate_to_pose` 会转发到内部 Nav2 `NavigateToPose` Action，内部 action 名默认 `interfaces.actions.nav2_navigate_to_pose=/nav2/navigate_to_pose`，避免和 FSM 对外 `/navigate_to_pose` 冲突。
- 已接 `/lifecycle_manager_navigation/is_active`、`/lifecycle_manager_localization/is_active`，失败映射 `E_NAV_LIFECYCLE_NOT_ACTIVE=4050`。
- 已订 `/amcl_pose`，按 `business.navigation_manager.amcl_max_covariance` 和 `amcl_timeout_sec` 判断定位健康，失败映射 `E_NAV_LOCALIZATION_LOST=4010`。
- 已订 `/chassis_node/status`，可按 `business.navigation_manager.require_chassis_ready` 启用底盘 enabled / fault / heartbeat 检查；运行中底盘故障会 cancel Nav2 并映射 `E_NAV_STUCK=4022`。
- 已封装 local/global costmap clear client。
- `/nav/base_recovery` 已调用 `/estop=False`、`/chassis_node/reset_fault`、`/chassis_node/enable`，并映射 4060/4061。
- 已支持上层 cancel / `/safety/estop` 透传到 Nav2 goal，并显式发布零 `/cmd_vel_align`。
- `require_fine_alignment=true` 时使用 `/perception/box_detections` 做视觉闭环，按距离误差和 yaw 误差发布 `/cmd_vel_align`；控制律吸收了旧 `alfa_nav_control/path_follower_node` 的限速、最小速度、死区和停发零速度思路，但不直接发 `/cmd_vel`。
- 已从 `old_codes/robot_navigation` 抽取边界策略：Fast-LIO / Nav2 / 2D 投影 / 地图发布作为外部导航 workspace 或 overlay 运行，当前仓库只维护 `navigation_manager_node` 适配层；旧 `path_follower_node` 和 `goal_resolver` 不作为 FSM 入口复用。
- L3-ISAAC 中 `navigation_manager_node` 仍只认 Nav2、AMCL、`/chassis_node/status`、`/cmd_vel_align` 和 `/nav/base_recovery` 契约；Isaac 底盘状态由 `isaac_chassis_bridge_node` 模拟，底盘运动桥后续作为下游 `/cmd_vel` 执行器接入。

离线验收：

- `scripts/m2_pre_nav_fake_real_smoke.py`

## 2. 对外接口

### Action：`/navigate_to_pose`

输入来自 `wall_destacking_strategy_node`，包括：

- `goal_type`：观察位、左作业位、右作业位（接口枚举仍为 `LEFT_PHASE` / `RIGHT_PHASE`）。
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
   - V1 已选定视觉反馈源：`/perception/box_detections` 中有效箱体 pose 的距离和 yaw 均值；激光最近点作为 V2。

3. 接底盘恢复。
   - `/clear_error` 第 1/3/4 阶段实际动作在本节点执行。
   - 每阶段必须有超时和错误码，不允许直接返回 success。

4. 接 health 聚合。
   - lifecycle inactive 映射 `E_NAV_LIFECYCLE_NOT_ACTIVE=4050`。
   - localization lost 映射 `E_NAV_LOCALIZATION_LOST=4010`。

## 3.1 M2-SIM 验收轨道

M2-SIM 用来在真机 Nav2 / chassis 未稳定前，先验证 navigation_manager 的 FINE_ALIGN 控制律和 TF 使用方式。

| 用例 | 组合 | 验收点 |
|---|---|---|
| L3-SIM-02 | 真实 `navigation_manager_node` + `fake_nav2_base_node` + `sim_world_node` | LEFT/RIGHT_PHASE 初始偏差 ±10cm / ±5° 时，`alignment_error_current` 在 `fine_alignment.timeout_sec` 内收敛 |
| L3-SIM-04 | 真实 `navigation_manager_node` + sim perception + grasp dry_run | `/cmd_vel_align` 与 coarse navigation 不竞争；Action feedback 阶段名能定位到 FINE_ALIGN |

实现约束：

- `fake_nav2_base_node` 只在测试 profile 中模拟 coarse goal 到位和 `/cmd_vel_align` 运动学积分，不能进入生产 launch。
- navigation_manager 的生产代码不得订阅 `/sim/*`，只读既有 `/perception/box_detections`、TF 和 Nav2/chassis 接口。
- L3-SIM-02 通过后，M2-B02 仍必须在真实视觉反馈下跑 L3-M2-NAV；仿真只证明控制律和 TF 链路合理。

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

- FINE_ALIGN 反馈源 V1 已冻结为 perception_adapter 的箱体/墙面检测；风险在于真实视觉延迟、漏检和箱面 yaw 噪声，需 L3-M2-NAV 验证。
- M2-SIM 能提前验证 FINE_ALIGN 控制律，但无法覆盖真实视觉漏检、延迟和噪声；L3-M2-NAV 仍是 M2 完成判据。
- `/clear_error` 第 1 阶段需要和 twist_mux 实际锁语义对齐，不能只发 `/estop=False` 就认为成功。
- 真机 Nav2 的 action result 与当前 mock 阶段序列不一定一致，M2 需要补状态映射表。

## 6. 从 old_codes/robot_navigation 抽取的取舍

| 旧模块 | 当前处理 | 原因 |
|---|---|---|
| `fastlio` / `FAST_LIO_LOCALIZATION2` | 外部 overlay 启动 | 属于定位/建图栈，不进入 FSM 业务包 |
| `alfa_nav_mapping/body_2d_projection_node` | 作为外部 TF 桥保留 | Nav2 需要 yaw-only `2d_body`；FSM 不发布 TF |
| `alfa_nav_planning` Nav2 参数 | 复用参数思路 | `robot_base_frame=2d_body`、footprint、costmap 配置对真机仍有价值 |
| `alfa_nav_control/path_follower_node` | 只抽控制律，不作为生产执行链 | 旧节点直接发 `/cmd_vel`，与当前 `/cmd_vel_align` / Nav2 边界冲突 |
| `alfa_nav_behavior/behavior_goal_translator` | 抽箱面法向生成目标 pose 思路 | 当前作业位由策略层和 `business.yaml` 管理，不新增旧行为接口 |
| `alfa_nav_goal_resolver` / `semantic_manager` | 暂不接入 | 当前 FSM 已有观察位/作业位配置；语义地标可作为后续运维工具 |

## 7. Isaac Sim 接入边界

Isaac Sim 全流程仿真不改变 `navigation_manager_node` 的职责。它仍作为策略层唯一导航入口：

```text
strategy_node → /navigate_to_pose → navigation_manager_node → Nav2 / chassis / twist_mux
```

L3-ISAAC 第一阶段已经提供 `isaac_chassis_bridge_node`，用于发布 `/chassis_node/status` 并响应 reset/enable。待 Isaac 安装后，再补底盘运动桥接节点消费 twist_mux 输出 `/cmd_vel`，并让 `/cmd_vel_align` 按当前 FINE_ALIGN 通道参与速度仲裁。
