# alpha_fsm × Isaac Sim 接口契约表

> 目的：梳理 alpha_fsm 状态机系统与 Isaac Sim 之间的全部交互接口，作为「在 Isaac Sim 中实现数字孪生」的对接基准。
> 范围：仅覆盖 ROS2 ↔ Isaac Sim 边界。FSM 内部节点间接口见 [`alpha_fsm/docs/03_接口契约.md`](alpha_fsm/docs/03_接口契约.md)，本表不重复。
> 字段：名称 / 类型 / msg 定义 / 频率 / QoS / 方向。

---

## 0. 总览

alpha_fsm 的设计原则是 **FSM 业务层只依赖三个适配节点的抽象接口**，从不直接对接硬件或仿真：

```
                 ┌─────────────── FSM 业务层（不感知仿真/真机）───────────────┐
                 │ task_manager / wall_destacking_strategy / safety_monitor  │
                 └───────┬──────────────┬───────────────────┬───────────────┘
                         │ Action       │ Action            │ Topic
              navigation_manager   pair_grasp_execution   perception_adapter
                         │              │                   │
        ════════════════ 接口边界（下游可换 真机 / Gazebo / Isaac Sim）════════════════
                         │              │                   │
                   底盘/Nav2/TF    ros2_control/关节/吸盘   感知真值/点云
                         └──────────────┴───────────────────┘
                                        ▲
                                  Isaac Sim 在此实现
```

**结论**：Isaac Sim 作为数字孪生后端，需要 **实现/消费三个适配节点下游的那组 ROS2 接口**，并补齐时钟、TF、关节状态等仿真基础设施。FSM 业务层与 fsm_msgs 冻结内容完全不变。

> 注：现有 L3-SIM-03 方案（见 [`docs/13`](alpha_fsm/docs/13_L3-SIM-03_MoveIt资料清单.md)）用的是 Gazebo。Isaac Sim 走同一组接口契约即可平替，**FSM 不依赖任何 `/sim/*` 私有调试 Topic 或仿真器内部 Topic**。

---

## 1. 接口分类总览

按数据流向分四类：

| 类   | 方向         | 子系统       | 说明                                                         |
| ---- | ------------ | ------------ | ------------------------------------------------------------ |
| A    | Isaac → ROS2 | 仿真基础设施 | `/clock`、TF、`/joint_states`                                |
| B    | ROS2 → Isaac | 控制下行     | 关节轨迹、底盘速度、吸盘命令                                 |
| C    | Isaac → ROS2 | 传感上行     | 相机、点云、底盘状态、吸盘压力、感知真值                     |
| D    | 双向         | 闭环对接     | MoveIt 规划链、Nav2 链（Isaac 提供物理与传感，ROS2 侧负责规划） |

单位约定沿用契约：长度 m、角度 rad、线速度 m/s、角速度 rad/s、压力 kPa、时间 `builtin_interfaces/Time`。

---

## 2. A 类：仿真基础设施（Isaac → ROS2）

| 名称            | 类型  | msg 定义                 | 频率                     | QoS                                            | 方向       | 备注                                                         |
| --------------- | ----- | ------------------------ | ------------------------ | ---------------------------------------------- | ---------- | ------------------------------------------------------------ |
| `/clock`        | Topic | `rosgraph_msgs/Clock`    | = 物理步频（建议 ≥60Hz） | RELIABLE / VOLATILE / KEEP_LAST / 1            | Isaac→ROS2 | 仿真时间源；所有节点须 `use_sim_time=true`，否则定量对比时间错位 |
| `/tf`           | Topic | `tf2_msgs/TFMessage`     | 动态链路 ≥30Hz           | RELIABLE / VOLATILE / KEEP_LAST / 100          | Isaac→ROS2 | Isaac 发 `odom→base_link` 等动态 TF；**不发 `map→odom`**（留给 AMCL/定位） |
| `/tf_static`    | Topic | `tf2_msgs/TFMessage`     | latched 一次             | RELIABLE / **TRANSIENT_LOCAL** / KEEP_LAST / 1 | Isaac→ROS2 | URDF 静态 TF（`base_link→camera_link`、`*_tool0` 等）        |
| `/joint_states` | Topic | `sensor_msgs/JointState` | 50~250Hz（建议≥100）     | BEST_EFFORT / VOLATILE / KEEP_LAST / 1         | Isaac→ROS2 | 15 关节快照，见 §6 关节清单；遥操作镜像与孪生同步的核心      |

> **frame 约定**（对齐 03 契约 §1.1）：Isaac 物理 TF 用 `body` / `2d_body` / `camera_init` 时，由 launch 静态 TF 桥接到逻辑名 `base_link` / `2d_base_link`。FSM 业务字段一律写逻辑名。

---

## 3. B 类：控制下行（ROS2 → Isaac）

Isaac Sim 订阅这些指令，驱动孪生体的关节与底盘。

| 名称                                              | 类型   | msg 定义                             | 频率                  | QoS                                        | 方向       | 备注                                                         |
| ------------------------------------------------- | ------ | ------------------------------------ | --------------------- | ------------------------------------------ | ---------- | ------------------------------------------------------------ |
| `/dual_v5_arm_controller/follow_joint_trajectory` | Action | `control_msgs/FollowJointTrajectory` | 按规划触发            | Action 默认                                | ROS2→Isaac | 双臂 12 关节轨迹落地；Isaac 侧 ros2_control 或等效控制器执行 |
| `/torso_controller/follow_joint_trajectory`       | Action | `control_msgs/FollowJointTrajectory` | 按规划触发            | Action 默认                                | ROS2→Isaac | 升降柱 `updown` 单独控制                                     |
| `/cmd_vel`                                        | Topic  | `geometry_msgs/Twist`                | ≥10Hz（timeout 0.3s） | RELIABLE / VOLATILE / KEEP_LAST / 10       | ROS2→Isaac | twist_mux 输出；Isaac 履带底盘只用 `linear.x` `angular.z`    |
| `/cmd_vel_align`                                  | Topic  | `geometry_msgs/Twist`                | FINE_ALIGN 期间 ~20Hz | RELIABLE / VOLATILE / KEEP_LAST / 10       | ROS2→Isaac | 对墙微调专用；经 twist_mux 优先级 50 汇入 `/cmd_vel`         |
| `/vacuum/cmd`                                     | Topic  | `fsm_msgs/VacuumCommand`             | 10Hz                  | RELIABLE / VOLATILE / KEEP_LAST / 10       | ROS2→Isaac | 左右吸盘开关；Isaac 侧抓取桥接器据此建立/解除固定约束        |
| `/estop`                                          | Topic  | `std_msgs/Bool`                      | 事件触发 + latched    | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST / 1 | ROS2→Isaac | 急停；Isaac 收到 True 应冻结底盘运动                         |

> **关键提醒（履带平台）**：`/cmd_vel` 的 `linear.x`/`angular.z` → 履带差速。Isaac PhysX 无原生履带，需用「多轮近似」或「表面速度近似」实现差速到履带的映射，且这是 sim-to-real 定量误差的主要来源（打滑），建议优先做技术验证。

---

## 4. C 类：传感上行（Isaac → ROS2）

Isaac 产出等价传感数据，供感知/导航/安全链消费。

| 名称                             | 类型  | msg 定义                                  | 频率                | QoS                                        | 方向                              | 备注                                                         |
| -------------------------------- | ----- | ----------------------------------------- | ------------------- | ------------------------------------------ | --------------------------------- | ------------------------------------------------------------ |
| `/camera/camera/color/image_raw` | Topic | `sensor_msgs/Image`                       | 10~30Hz             | BEST_EFFORT / VOLATILE / KEEP_LAST / 5     | Isaac→ROS2                        | RGB；perception_adapter 仅用于 health 帧间隔判断             |
| `/camera/camera_info`            | Topic | `sensor_msgs/CameraInfo`                  | latched / 随图      | RELIABLE / TRANSIENT_LOCAL / KEEP_LAST / 1 | Isaac→ROS2                        | 内参；Isaac 相机标定参数需与真机一致                         |
| `/cloud_registered_body`         | Topic | `sensor_msgs/PointCloud2`                 | = 雷达帧率（~10Hz） | BEST_EFFORT / VOLATILE / KEEP_LAST / 5     | Isaac→ROS2                        | RTX Lidar 输出，frame=`body`；fast_lio 等价输入              |
| `/box_perception/result`         | Topic | `box_perception_msgs/BoxPerceptionResult` | 5~10Hz              | BEST_EFFORT / VOLATILE / KEEP_LAST / 5     | Isaac→ROS2                        | **可选**：Isaac 用真值+扰动模拟上游感知，frame=`body`；或走 adapter_bypass 直接发 `/perception/box_detections` |
| `/chassis_node/status`           | Topic | `diagnostic_msgs/DiagnosticArray`         | 5~10Hz              | RELIABLE / VOLATILE / KEEP_LAST / 1        | Isaac→ROS2                        | 模拟底盘 enabled / fault / heartbeat；navigation_manager 监控 |
| `/vacuum/pressure_raw`           | Topic | `std_msgs/Float32MultiArray`              | ≥20Hz               | BEST_EFFORT / VOLATILE / KEEP_LAST / 10    | Isaac→ROS2                        | `[left_kpa, right_kpa]`；可由 vacuum_io_node mock 模型产出（150ms 内 0→-60kPa） |
| `/amcl_pose`                     | Topic | `geometry_msgs/PoseWithCovarianceStamped` | ~静态/重定位        | RELIABLE / VOLATILE / KEEP_LAST / 1        | （AMCL发，依赖 Isaac 的 scan/TF） | Isaac 提供雷达与 TF，AMCL 在 ROS2 侧运行                     |

> 感知接入有两种 profile（对齐 sim.yaml `sensor.output_mode`）：
>
> - `adapter_input`：Isaac 模拟上游 `/box_perception/result`，经 perception_adapter 转成 `/perception/box_detections`（保留完整真机数据链）
> - `adapter_bypass`：Isaac 直接发 `/perception/box_detections`（`fsm_msgs/BoxDetectionArray`），跳过 adapter（仅测试用）

---

## 5. D 类：闭环对接链（双向，规划在 ROS2 / 物理在 Isaac）

这些不是单条 topic，而是 Isaac 与既有 ROS2 组件协同的链路。Isaac 提供物理与传感，ROS2 侧组件（MoveIt / Nav2 / ros2_control）负责规划决策。

| 链路         | ROS2 侧组件                            | Isaac 侧职责                                             | 关键接口                                     |
| ------------ | -------------------------------------- | -------------------------------------------------------- | -------------------------------------------- |
| 双臂运动规划 | MoveIt2 `move_group`（`/move_action`） | 提供 `/joint_states` 反馈 + 执行 `FollowJointTrajectory` | `dual_v5_arm_with_base` group；IK=`bio_ik`   |
| 底盘导航     | Nav2（bt_navigator / costmap / AMCL）  | 提供雷达 `/scan` 或点云 + `odom` TF + 执行 `/cmd_vel`    | navigation_manager 适配，FSM 不直连 Nav2     |
| 关节控制     | ros2_control / controller_manager      | 运行控制器后端，驱动 articulation                        | `joint_state_broadcaster` 发 `/joint_states` |
| 抓取约束     | pair_grasp_execution（吸盘逻辑）       | 接 `/vacuum/cmd` 判定接触 → 建立/解除箱体-末端固定约束   | 见 §3 `/vacuum/cmd`                          |

---

## 6. 关节清单（articulation 配置依据）

来自 [`joint_names_alfa_robot_v2_arm_v6.yaml`](alpha_fsm/fsm_ws/src/alfa_robot_v2_arm_v6/config/joint_names_alfa_robot_v2_arm_v6.yaml)，`/joint_states` 与 `FollowJointTrajectory` 的关节名必须严格对齐：

```text
躯干/升降：  pitch, turn, updown
左臂 6 关节：leftjoint1, leftjoint2, leftjoint3, leftjoint4, leftjoint5, leftjoint6
右臂 6 关节：rightjoint1, rightjoint2, rightjoint3, rightjoint4, rightjoint5, rightjoint6
```

| 项                  | 值                                                   |
| ------------------- | ---------------------------------------------------- |
| planning group      | `dual_v5_arm_with_base`                              |
| 左 TCP              | `left_v5_tool0`（`left_v5_link6` + xyz `0 0 0.1`）   |
| 右 TCP              | `right_v5_tool0`（`right_v5_link6` + xyz `0 0 0.1`） |
| root link           | `base_link`                                          |
| IK solver           | `bio_ik/BioIKKinematicsPlugin`                       |
| dual_arm controller | `/dual_v5_arm_controller/follow_joint_trajectory`    |
| torso controller    | `/torso_controller/follow_joint_trajectory`          |

> ⚠️ 真机关节命名顺序 / 单位 / 零点未必与 Isaac USD 一致，需在适配层做 remap（对齐 03 契约的接口适配层职责）。

---

## 7. QoS 速查（汇总）

| QoS 档             | Reliability | Durability      | History   | Depth | 用于                                                         |
| ------------------ | ----------- | --------------- | --------- | ----- | ------------------------------------------------------------ |
| 状态/心跳          | RELIABLE    | TRANSIENT_LOCAL | KEEP_LAST | 1     | `/tf_static`、`/camera_info`、`/estop`、`/chassis_status`、`/amcl_pose` |
| 高频流（丢旧取新） | BEST_EFFORT | VOLATILE        | KEEP_LAST | 1~10  | `/joint_states`、`/cloud_registered_body`、`/image_raw`、`/vacuum/pressure_raw` |
| 控制指令           | RELIABLE    | VOLATILE        | KEEP_LAST | 10    | `/cmd_vel`、`/cmd_vel_align`、`/vacuum/cmd`                  |
| 时钟               | RELIABLE    | VOLATILE        | KEEP_LAST | 1     | `/clock`                                                     |

> 遥操作/孪生同步类高频流用 BEST_EFFORT + depth=1，确保「永远拿最新帧、旧帧丢弃」，避免 RELIABLE 重传累积延迟。

---

## 8. 来源

- [`alpha_fsm/docs/03_接口契约.md`](alpha_fsm/docs/03_接口契约.md) — FSM 内部接口冻结契约（v1.3）
- [`alpha_fsm/fsm_ws/src/fsm_config/params/interfaces.yaml`](alpha_fsm/fsm_ws/src/fsm_config/params/interfaces.yaml) — topic/service/action/frame/QoS 名集中表
- [`alpha_fsm/fsm_ws/src/fsm_config/params/sim.yaml`](alpha_fsm/fsm_ws/src/fsm_config/params/sim.yaml) — 仿真 profile 配置
- [`alpha_fsm/docs/13_L3-SIM-03_MoveIt资料清单.md`](alpha_fsm/docs/13_L3-SIM-03_MoveIt资料清单.md) — 轻量物理仿真方案（Gazebo 版，Isaac 平替参照）
- [`alpha_fsm/fsm_ws/src/alfa_robot_v2_arm_v6/config/joint_names_alfa_robot_v2_arm_v6.yaml`](alpha_fsm/fsm_ws/src/alfa_robot_v2_arm_v6/config/joint_names_alfa_robot_v2_arm_v6.yaml) — 关节命名

---

## 9. 状态机迁移表

> 来源：[`alpha_fsm/docs/02_状态规格手册.md`](alpha_fsm/docs/02_状态规格手册.md)（规格卡）+ [`docs/06_状态机Mermaid图集.md`](alpha_fsm/docs/06_状态机Mermaid图集.md)（跳转校核）。
> 系统为三层 FSM：根 FSM（task_manager）→ 核心子 FSM（wall_destacking_strategy）→ 适配 FSM（navigation_manager / pair_grasp_execution），外加并行的 SafetyMonitorFSM。
> 列含义：`From → To` 为状态跳转；`Event` 为触发（success/failure/timeout/abort/stay 或具体信号）；`Timeout` 为该 From 状态的超时；`异常码` 为该跳转可能携带的 error_code。`[*]` 表示初始/终态。

### 9.0 三层调用关系

```text
RobotSystemFSM (task_manager_node)          根：整机生命周期与模式
  └─ TaskFSM (task_manager_node)            根：单次任务编排
       └─ WallDestackingFSM (strategy_node) 核心：拆垛主循环
            ├─ WallMappingFSM               子：建 5×5 网格
            ├─ PhasePerceptionFSM           子：局部感知更新
            ├─ PairSelectionFSM             子：选左右抓取对
            ├─ WallRecoveryFSM              子：错误恢复决策
            ├─ BaseNavigationFSM (nav_manager_node)        适配：底盘导航
            └─ PairGraspExecutionFSM (pair_grasp_node)     适配：双臂抓取
SafetyMonitorFSM (safety_monitor_node)      并行：全程安全监控
```

### 9.1 RobotSystemFSM（根，task_manager_node）

整机生命周期：`BOOTING→SELF_CHECK→STANDBY→AUTO_MODE`，可分支 MANUAL_MODE/PAUSED/SHUTDOWN，异常进 FAULT/E_STOP，清错后回 SELF_CHECK。

| From → To             | Event        | 条件                                 | Timeout | 异常码                                                       |
| --------------------- | ------------ | ------------------------------------ | ------- | ------------------------------------------------------------ |
| `[*]` → BOOTING       | init         | 节点启动初始状态                     | —       | —                                                            |
| BOOTING → SELF_CHECK  | success      | 所有底层节点心跳齐                   | 30s     | —                                                            |
| BOOTING → FAULT       | failure      | 底层节点启动失败                     | 30s     | E_SYS_BOOT_FAIL, E_SYS_HARDWARE_NOT_READY, E_COMM_NODE_OFFLINE |
| BOOTING → FAULT       | timeout      | 30s 内心跳超时未齐                   | 30s     | 同上                                                         |
| SELF_CHECK → STANDBY  | success      | 自检全通过且底盘已使能               | 20s     | —                                                            |
| SELF_CHECK → FAULT    | failure      | 任一自检项失败                       | 20s     | E_SYS_BOOT_FAIL, E_SYS_CONFIG_INVALID, E_EXT_PERC_OFFLINE, E_EXT_PERC_CAMERA_FAIL, E_EXT_PERC_LIDAR_FAIL, E_NAV_LIFECYCLE_NOT_ACTIVE, E_CHASSIS_ENABLE_FAIL |
| SELF_CHECK → FAULT    | timeout      | 20s 内未完成自检                     | 20s     | 同上                                                         |
| STANDBY → AUTO_MODE   | /task/start  | 收到任务启动 Service                 | —       | —                                                            |
| STANDBY → MANUAL_MODE | 手动切换     | 收到手动模式切换请求                 | —       | —                                                            |
| STANDBY → SHUTDOWN    | 关机         | 收到关机命令                         | —       | —                                                            |
| AUTO_MODE → STANDBY   | success      | TaskFSM 任务完成                     | —       | —                                                            |
| AUTO_MODE → PAUSED    | /task/pause  | 用户暂停                             | —       | —                                                            |
| AUTO_MODE → FAULT     | failure      | FATAL 错误                           | —       | E_TASK_CHILD_FAILED, E_TASK_CHILD_TIMEOUT                    |
| AUTO_MODE → E_STOP    | abort        | 急停触发                             | —       | —                                                            |
| MANUAL_MODE → STANDBY | success      | 切回自动模式                         | —       | —                                                            |
| MANUAL_MODE → FAULT   | failure      | 手动操作异常                         | —       | E_MAN_OVERRIDE                                               |
| PAUSED → AUTO_MODE    | /task/resume | 收到恢复请求                         | 300s    | —                                                            |
| PAUSED → FAULT        | timeout      | 暂停超时（参数化 300s）              | 300s    | E_TASK_PAUSE_TIMEOUT                                         |
| FAULT → SELF_CHECK    | success      | /clear_error 五阶段全部完成          | —       | E_SAFETY_ESTOP_LOCK_STUCK, E_SYS_ACTION_CANCEL_TIMEOUT, E_CHASSIS_FAULT_RESET_FAIL, E_CHASSIS_ENABLE_FAIL, E_SYS_SELF_CHECK_REENTRY_FAIL |
| FAULT → FAULT         | stay         | clear_error 某阶段失败，等待再次清错 | —       | 同上                                                         |
| E_STOP → SELF_CHECK   | success      | 急停解除 + clear_error 五阶段全成    | —       | E_SAFETY_ESTOP_HW, E_SAFETY_ESTOP_SW                         |
| E_STOP → E_STOP       | stay         | 急停未解除或清错阶段失败             | —       | E_SAFETY_ESTOP_HW, E_SAFETY_ESTOP_SW, E_SAFETY_ESTOP_LOCK_STUCK, E_CHASSIS_FAULT_RESET_FAIL, E_CHASSIS_ENABLE_FAIL |
| SHUTDOWN → `[*]`      | success      | 清理完毕进程退出（终态）             | 5s      | —                                                            |

### 9.2 TaskFSM（根，task_manager_node）

单次任务编排：`WAIT_TASK→ACCEPT→VALIDATE→PREPARE→RUN_TASK→VERIFY→COMPLETE`，失败入 FAIL_TASK，取消入 CANCEL_TASK，三终态均回 WAIT_TASK。

| From → To                          | Event           | 条件                                                 | Timeout | 异常码                                         |
| ---------------------------------- | --------------- | ---------------------------------------------------- | ------- | ---------------------------------------------- |
| WAIT_TASK → ACCEPT_TASK            | /task/start     | 收到任务启动请求                                     | —       | —                                              |
| ACCEPT_TASK → VALIDATE_TASK        | success         | 入参解析成功                                         | 1s      | —                                              |
| ACCEPT_TASK → FAIL_TASK            | failure         | 入参解析错误                                         | 1s      | E_TASK_VALIDATE_FAIL                           |
| VALIDATE_TASK → PREPARE_TASK       | success         | 前置条件全通过（AUTO/安全/感知/tf）                  | 5s      | —                                              |
| VALIDATE_TASK → FAIL_TASK          | failure         | 前置条件不满足                                       | 5s      | E_TASK_VALIDATE_FAIL, E_TASK_PRECONDITION_FAIL |
| PREPARE_TASK → RUN_TASK            | success         | 准备完毕                                             | 3s      | —                                              |
| PREPARE_TASK → FAIL_TASK           | failure         | 准备失败                                             | 3s      | E_TASK_PRECONDITION_FAIL                       |
| RUN_TASK → VERIFY_TASK_RESULT      | success         | WallDestacking 子 Action 完成                        | 1800s   | —                                              |
| RUN_TASK → CANCEL_TASK             | abort           | 收到 /task/cancel                                    | 1800s   | —                                              |
| RUN_TASK → FAIL_TASK               | failure/timeout | WallDestacking 失败或超时（急停直跳）                | 1800s   | E_TASK_CHILD_FAILED, E_TASK_CHILD_TIMEOUT      |
| VERIFY_TASK_RESULT → COMPLETE_TASK | success         | 结果合法                                             | 2s      | —                                              |
| VERIFY_TASK_RESULT → FAIL_TASK     | failure         | 结果不合法                                           | 2s      | E_TASK_CHILD_FAILED                            |
| COMPLETE_TASK → WAIT_TASK          | success         | 上报完毕                                             | 2s      | —                                              |
| FAIL_TASK → WAIT_TASK              | success         | 故障上报完毕（FATAL 上报 RobotSystemFSM 触发 FAULT） | 2s      | last_error                                     |
| CANCEL_TASK → WAIT_TASK            | success         | Action server cancel 完成                            | 10s     | E_MAN_CANCELLED                                |
| CANCEL_TASK → FAIL_TASK            | failure         | cancel 失败                                          | 10s     | E_MAN_CANCELLED                                |

### 9.3 WallDestackingFSM（核心子 FSM，strategy_node）

拆垛主循环：导航观察位→建网格→逐 phase 感知选 pair 抓取并更新 grid，循环至全墙无箱确认。任何 FAILURE 进 WALL_ERROR_HANDLE 由 WallRecoveryFSM 决策跳转或 ABORT。

| From → To                                         | Event              | 条件                                                 | Timeout | 异常码                                                       |
| ------------------------------------------------- | ------------------ | ---------------------------------------------------- | ------- | ------------------------------------------------------------ |
| INIT_WALL_TASK → NAVIGATE_TO_OBSERVATION_POSE     | success            | ctx 初始化完毕                                       | 1s      | E_WALL_STATE_TIMEOUT                                         |
| INIT_WALL_TASK → WALL_ERROR_HANDLE                | failure            | 初始化失败                                           | 1s      | E_WALL_STATE_TIMEOUT                                         |
| NAVIGATE_TO_OBSERVATION_POSE → RUN_WALL_MAPPING   | success            | 导航到位                                             | 60s     | E_NAV_GOAL_REJECTED, E_NAV_GOAL_TIMEOUT, E_NAV_PATH_PLAN_FAIL, E_NAV_LOCALIZATION_LOST, E_NAV_WORKPOSE_INVALID |
| NAVIGATE_TO_OBSERVATION_POSE → WALL_ERROR_HANDLE  | failure            | 导航失败（重试上限 2）                               | 60s     | 同上                                                         |
| RUN_WALL_MAPPING → CHECK_WALL_VALID               | success            | 建网成功                                             | 30s     | E_MAP_GLOBAL_SCAN_FAIL, E_MAP_NO_DETECTION, E_MAP_INSUFFICIENT_DETECTION, E_MAP_WALL_FRAME_FAIL, E_MAP_GRID_BUILD_FAIL, E_MAP_NO_NEW_WALL |
| RUN_WALL_MAPPING → WALL_ERROR_HANDLE              | failure            | 建网失败（重试 1，子 FSM error 透传）                | 30s     | 同上                                                         |
| CHECK_WALL_VALID → NAVIGATE_TO_PHASE_WORKPOSE     | success            | grid 有效合法                                        | 1s      | E_MAP_GRID_BUILD_FAIL                                        |
| CHECK_WALL_VALID → VERIFY_TASK_COMPLETE           | new_wall=false     | 无新墙                                               | 1s      | E_MAP_GRID_BUILD_FAIL                                        |
| CHECK_WALL_VALID → WALL_ERROR_HANDLE              | failure            | grid 不可用                                          | 1s      | E_MAP_GRID_BUILD_FAIL                                        |
| NAVIGATE_TO_PHASE_WORKPOSE → RUN_PHASE_PERCEPTION | success            | 导航到位 + 对墙微调成功                              | 60s     | E_NAV_*, E_NAV_FINE_ALIGN_FAIL, E_NAV_WORKPOSE_INVALID       |
| NAVIGATE_TO_PHASE_WORKPOSE → WALL_ERROR_HANDLE    | failure            | 导航/对齐失败（重试上限 2）                          | 60s     | 同上                                                         |
| RUN_PHASE_PERCEPTION → RUN_PAIR_SELECTION         | success            | 局部感知更新完成                                     | 20s     | E_PERC_NO_LOCAL_DETECTION, E_PERC_ASSOCIATION_FAIL, E_PERC_LOCAL_SCAN_TIMEOUT |
| RUN_PHASE_PERCEPTION → WALL_ERROR_HANDLE          | failure            | 感知失败（重试上限 2）                               | 20s     | 同上                                                         |
| RUN_PAIR_SELECTION → DISPATCH_PAIR_GRASP          | success            | 推荐 phase 与当前作业位相同，选出 pair               | 2s      | E_PAIR_NO_CANDIDATE, E_PAIR_NO_REACHABLE, E_PAIR_DUAL_CONFLICT, E_PAIR_SINGLE_NOT_ALLOWED |
| RUN_PAIR_SELECTION → DECIDE_NEXT_PHASE            | 推荐另一作业位     | 推荐 phase 与当前作业位不同                          | 2s      | 同上                                                         |
| RUN_PAIR_SELECTION → WALL_ERROR_HANDLE            | failure            | 无安全候选/选取失败                                  | 2s      | 同上                                                         |
| DISPATCH_PAIR_GRASP → WAIT_PAIR_GRASP_RESULT      | success            | goal 已发（accepted）                                | 2s      | E_GRASP_GOAL_REJECTED, E_GRASP_INVALID_PAIR                  |
| DISPATCH_PAIR_GRASP → WALL_ERROR_HANDLE           | failure            | goal 被拒绝                                          | 2s      | 同上                                                         |
| WAIT_PAIR_GRASP_RESULT → UPDATE_GRID_AFTER_GRASP  | success            | result 到达                                          | 120s    | E_GRASP_GOAL_TIMEOUT, 5xxx（抓取层透传）                     |
| WAIT_PAIR_GRASP_RESULT → UPDATE_GRID_AFTER_GRASP  | failure            | 失败/超时（超时先 cancel goal），失败结果也更新 grid | 120s    | 同上                                                         |
| UPDATE_GRID_AFTER_GRASP → DECIDE_NEXT_PAIR        | success            | grid 更新完毕                                        | 1s      | E_MOT_DROP_BOX, E_VAC_LOST_DURING_CARRY                      |
| UPDATE_GRID_AFTER_GRASP → WALL_ERROR_HANDLE       | failure            | 掉箱等 FATAL                                         | 1s      | 同上                                                         |
| DECIDE_NEXT_PAIR → RUN_PHASE_PERCEPTION           | wall 仍有 OCCUPIED | wall 仍有可抓箱                                      | 1s      | —                                                            |
| DECIDE_NEXT_PAIR → DECIDE_NEXT_WALL               | wall 无 OCCUPIED   | wall 已完成                                          | 1s      | —                                                            |
| DECIDE_NEXT_PHASE → NAVIGATE_TO_PHASE_WORKPOSE    | stay               | current_phase 切到推荐 phase                         | 1s      | —                                                            |
| DECIDE_NEXT_WALL → NAVIGATE_TO_OBSERVATION_POSE   | wall_index+1       | 进下一面墙                                           | 1s      | —                                                            |
| DECIDE_NEXT_WALL → VERIFY_TASK_COMPLETE           | 达到上限           | 达到 max_walls 限制                                  | 1s      | —                                                            |
| VERIFY_TASK_COMPLETE → WALL_DONE                  | success            | 连续多帧无箱确认                                     | 30s     | E_WALL_TASK_COMPLETE_TIMEOUT, E_WALL_EMPTY_VERIFY_CONFLICT   |
| VERIFY_TASK_COMPLETE → RUN_WALL_MAPPING           | failure            | 仍看到箱，重扫确认                                   | 30s     | 同上                                                         |
| VERIFY_TASK_COMPLETE → WALL_ERROR_HANDLE          | timeout            | 多帧时间窗超时                                       | 30s     | 同上                                                         |
| WALL_DONE → `[*]`                                 | terminal           | Action result.success=true 返回（终态）              | 1s      | —                                                            |
| WALL_ERROR_HANDLE → NAVIGATE_TO_OBSERVATION_POSE  | recovery 决策      | WallRecoveryFSM 推荐重导航观察位                     | 30s     | 透传                                                         |
| WALL_ERROR_HANDLE → NAVIGATE_TO_PHASE_WORKPOSE    | recovery 决策      | 推荐重导航作业位                                     | 30s     | 透传                                                         |
| WALL_ERROR_HANDLE → RUN_PHASE_PERCEPTION          | recovery 决策      | 推荐重做局部感知                                     | 30s     | 透传                                                         |
| WALL_ERROR_HANDLE → RUN_PAIR_SELECTION            | recovery 决策      | 推荐重选 pair                                        | 30s     | 透传                                                         |
| WALL_ERROR_HANDLE → `[*]`                         | abort              | ABORT，Action result.success=false（终态）           | 30s     | 透传                                                         |

### 9.4 WallMappingFSM（子 FSM，strategy_node）

从全局观察位扫描最外侧货箱墙，估计 wall_frame 并生成 5×5 网格。任一环节失败转 MAPPING_ERROR 终态。

| From → To                               | Event   | 条件                                 | Timeout                      | 异常码                                                 |
| --------------------------------------- | ------- | ------------------------------------ | ---------------------------- | ------------------------------------------------------ |
| START_WINDOW → COLLECT_FRAMES           | success | 开窗参数就绪                         | 1s                           | —                                                      |
| START_WINDOW → MAPPING_ERROR            | failure | 开窗失败                             | 1s                           | —                                                      |
| COLLECT_FRAMES → FILTER_DETECTIONS      | success | 达到 N 帧或 T 秒，帧数足够           | mapping_window_sec（默认3s） | E_MAP_NO_DETECTION, E_PERC_LOCAL_SCAN_TIMEOUT          |
| COLLECT_FRAMES → MAPPING_ERROR          | failure | 超时或无检测                         | 同上                         | 同上                                                   |
| FILTER_DETECTIONS → ESTIMATE_WALL_FRAME | success | 滤波后 ≥ min_valid 个检测            | 2s                           | E_MAP_INSUFFICIENT_DETECTION                           |
| FILTER_DETECTIONS → MAPPING_ERROR       | failure | 有效检测不足                         | 2s                           | 同上                                                   |
| ESTIMATE_WALL_FRAME → BUILD_5X5_GRID    | success | wall 平面拟合成功                    | 3s                           | E_MAP_WALL_FRAME_FAIL, E_MAP_WALL_FRAME_LOW_CONFIDENCE |
| ESTIMATE_WALL_FRAME → MAPPING_ERROR     | failure | 平面拟合失败                         | 3s                           | 同上                                                   |
| BUILD_5X5_GRID → INIT_GRID_SLOTS        | success | 25 个 slot 的 expected_pose 计算完毕 | 1s                           | E_MAP_GRID_BUILD_FAIL                                  |
| BUILD_5X5_GRID → MAPPING_ERROR          | failure | grid 构造失败                        | 1s                           | 同上                                                   |
| INIT_GRID_SLOTS → VALIDATE_GRID         | success | 每个 slot 与最近检测匹配完成         | 2s                           | E_MAP_GRID_INCOMPLETE                                  |
| INIT_GRID_SLOTS → MAPPING_ERROR         | failure | slot 初始化失败                      | 2s                           | 同上                                                   |
| VALIDATE_GRID → CHECK_NEW_WALL          | success | grid 合法（row0 与 col0 全匹配）     | 1s                           | E_MAP_GRID_BUILD_FAIL                                  |
| VALIDATE_GRID → MAPPING_ERROR           | failure | grid 不合法                          | 1s                           | 同上                                                   |
| CHECK_NEW_WALL → REPORT                 | success | 完成新墙判定                         | 1s                           | E_MAP_NO_NEW_WALL                                      |
| CHECK_NEW_WALL → MAPPING_ERROR          | failure | 判定异常                             | 1s                           | 同上                                                   |
| REPORT → `[*]` SUCCESS                  | success | 父 FSM 取走结果                      | 1s                           | —                                                      |
| MAPPING_ERROR → `[*]` FAILURE           | failure | 携带 error_code 返回父 FSM           | 1s                           | 透传                                                   |

### 9.5 PhasePerceptionFSM（子 FSM，strategy_node）

近距离局部感知采集→滤波→变换到 base_link→匹配回逻辑网格槽位→更新可见 pose→标记未见→上报。任一步失败转 PERCEPTION_ERROR 终态。

| From → To                                 | Event   | 条件                                        | Timeout          | 异常码                                               |
| ----------------------------------------- | ------- | ------------------------------------------- | ---------------- | ---------------------------------------------------- |
| START_LOCAL_WINDOW → COLLECT_LOCAL_FRAMES | success | 开窗参数就绪                                | 1s               | —                                                    |
| START_LOCAL_WINDOW → PERCEPTION_ERROR     | failure | 开窗准备失败                                | 1s               | —                                                    |
| COLLECT_LOCAL_FRAMES → FILTER_LOCAL       | success | 达到 N 帧或 T 秒，帧数足够                  | local_window_sec | —                                                    |
| COLLECT_LOCAL_FRAMES → PERCEPTION_ERROR   | failure | 采集超时或无检测                            | local_window_sec | E_PERC_NO_LOCAL_DETECTION, E_PERC_LOCAL_SCAN_TIMEOUT |
| FILTER_LOCAL → TRANSFORM_TO_ROBOT         | success | 滤波完成                                    | 1s               | —                                                    |
| FILTER_LOCAL → PERCEPTION_ERROR           | failure | 误检过多滤波失败                            | 1s               | E_PERC_TOO_MANY_FALSE_POSITIVE                       |
| TRANSFORM_TO_ROBOT → MATCH_TO_SLOTS       | success | 检测已为 base_link 且时间戳新鲜（tf OK）    | 2s               | —                                                    |
| TRANSFORM_TO_ROBOT → PERCEPTION_ERROR     | failure | tf 校验失败                                 | 2s               | E_COMM_TF_LOOKUP_FAIL                                |
| MATCH_TO_SLOTS → UPDATE_SLOT_POSES        | success | 至少 1 个 slot 匹配                         | 2s               | —                                                    |
| MATCH_TO_SLOTS → PERCEPTION_ERROR         | failure | 全部不匹配                                  | 2s               | E_PERC_ASSOCIATION_FAIL, E_PERC_SLOT_POSE_OUTLIER    |
| UPDATE_SLOT_POSES → MARK_UNSEEN           | success | slot pose 更新完毕                          | 1s               | —                                                    |
| UPDATE_SLOT_POSES → PERCEPTION_ERROR      | failure | pose 更新失败                               | 1s               | —                                                    |
| MARK_UNSEEN → REPORT                      | success | 未匹配 slot 标记 visible=false 完毕         | 1s               | —                                                    |
| MARK_UNSEEN → PERCEPTION_ERROR            | failure | 标记失败                                    | 1s               | —                                                    |
| REPORT → `[*]` SUCCESS                    | success | 携带 visible 列表返回父 FSM                 | 1s               | —                                                    |
| PERCEPTION_ERROR → `[*]` FAILURE          | failure | 携带 error_code 返回父 FSM，发布 /fsm/error | 1s               | 透传                                                 |

### 9.6 PairSelectionFSM（子 FSM，strategy_node）

同时评估左右作业位的下一抓候选，经全局列高度安全过滤、可达性与双臂冲突校验后按 Y 分配左右手生成 GraspPair。

| From → To                                       | Event   | 条件                              | Timeout | 异常码                    |
| ----------------------------------------------- | ------- | --------------------------------- | ------- | ------------------------- |
| FILTER_AVAILABLE → SORT_BY_PRIORITY             | success | 候选列表生成（候选 ≥1）           | 1s      | —                         |
| FILTER_AVAILABLE → PAIR_SELECTION_ERROR         | failure | 无候选                            | 1s      | E_PAIR_NO_CANDIDATE       |
| SORT_BY_PRIORITY → BUILD_CANDIDATES             | success | 左右作业位内按优先级排序完成      | 1s      | —                         |
| BUILD_CANDIDATES → CHECK_HEIGHT_SAFETY          | success | 候选 pair 列表生成                | 1s      | —                         |
| CHECK_HEIGHT_SAFETY → VALIDATE_REACHABILITY     | success | 产生第一个高度安全候选            | 1s      | —                         |
| CHECK_HEIGHT_SAFETY → PAIR_SELECTION_ERROR      | failure | 无高度安全候选                    | 1s      | E_PAIR_NO_CANDIDATE       |
| VALIDATE_REACHABILITY → CHECK_DUAL_ARM_CONFLICT | success | 选出可达 pair                     | 2s      | —                         |
| VALIDATE_REACHABILITY → PAIR_SELECTION_ERROR    | failure | 全部候选不可达                    | 2s      | E_PAIR_NO_REACHABLE       |
| CHECK_DUAL_ARM_CONFLICT → ASSIGN_ARMS_BY_Y      | success | 双臂无冲突                        | 1s      | —                         |
| CHECK_DUAL_ARM_CONFLICT → VALIDATE_REACHABILITY | stay    | 有冲突且仍有候选，取下一组        | 1s      | —                         |
| CHECK_DUAL_ARM_CONFLICT → PAIR_SELECTION_ERROR  | failure | 有冲突且候选耗尽                  | 1s      | E_PAIR_DUAL_CONFLICT      |
| ASSIGN_ARMS_BY_Y → BUILD_GRASP_PAIR             | success | 按 Y 值左右手分配完毕             | 1s      | —                         |
| ASSIGN_ARMS_BY_Y → PAIR_SELECTION_ERROR         | failure | 左右手分配非法                    | 1s      | E_PAIR_ARM_ASSIGN_INVALID |
| BUILD_GRASP_PAIR → REPORT                       | success | GraspPair 构造完毕                | 1s      | —                         |
| REPORT → `[*]` SUCCESS                          | success | 发布 /fsm/grasp_pair 并返回父 FSM | 1s      | —                         |
| PAIR_SELECTION_ERROR → `[*]` FAILURE            | failure | 携带 error_code 返回父 FSM        | 1s      | 透传                      |

### 9.7 WallRecoveryFSM（子 FSM，strategy_node）

错误恢复决策：`RECEIVE_ERROR→CLASSIFY→CHECK_RETRY_LIMIT→SELECT_RECOVERY_ACTION→EXECUTE_RECOVERY→REPORT`。重试超限或恢复失败转 WAIT_MANUAL_RECOVERY，人工恢复后回 REPORT，否则超时 ABORT。

| From → To                                  | Event         | 条件                         | Timeout | 异常码                       |
| ------------------------------------------ | ------------- | ---------------------------- | ------- | ---------------------------- |
| RECEIVE_ERROR → CLASSIFY                   | success       | error 接收完毕               | 1s      | —                            |
| CLASSIFY → CHECK_RETRY_LIMIT               | success       | 等级/源已分类                | 1s      | —                            |
| CHECK_RETRY_LIMIT → SELECT_RECOVERY_ACTION | success       | 未超重试上限                 | 1s      | —                            |
| CHECK_RETRY_LIMIT → WAIT_MANUAL_RECOVERY   | failure       | 达到 max_retry 上限          | 1s      | —                            |
| SELECT_RECOVERY_ACTION → EXECUTE_RECOVERY  | success       | 已决定 recovery action       | 1s      | —                            |
| EXECUTE_RECOVERY → REPORT                  | success       | 恢复动作执行成功             | 30s     | —                            |
| EXECUTE_RECOVERY → WAIT_MANUAL_RECOVERY    | failure       | 恢复动作失败，升级人工       | 30s     | —                            |
| REPORT → `[*]` SUCCESS                     | success       | 返回父 FSM 推荐 next_state   | 1s      | —                            |
| WAIT_MANUAL_RECOVERY → REPORT              | /clear_error  | 人工清错并指定 next_state    | 600s    | —                            |
| WAIT_MANUAL_RECOVERY → `[*]` FAILURE       | timeout/abort | 等待人工恢复超时，触发 ABORT | 600s    | E_WALL_TASK_COMPLETE_TIMEOUT |

### 9.8 BaseNavigationFSM（适配 FSM，navigation_manager_node）

收到导航 goal 后依次校验前置/定位、规划并执行底盘导航，到位后按需对墙微调，再校验工作位姿并上报。任一环节失败进 NAV_ERROR。**此 FSM 是 Isaac 底盘导航链的直接对接对象。**

| From → To                                 | Event                  | 条件                                                         | Timeout | 异常码                                                       |
| ----------------------------------------- | ---------------------- | ------------------------------------------------------------ | ------- | ------------------------------------------------------------ |
| `[*]` → RECEIVE_GOAL                      | start                  | FSM 启动接收 NavigateToPose goal                             | —       | —                                                            |
| RECEIVE_GOAL → CHECK_PRECONDITION         | success                | 收到导航 goal                                                | —       | —                                                            |
| CHECK_PRECONDITION → CHECK_LOCALIZATION   | success                | 前置条件满足                                                 | —       | —                                                            |
| CHECK_LOCALIZATION → PLAN_PATH            | 定位 OK                | AMCL 协方差在阈值内                                          | —       | —                                                            |
| CHECK_LOCALIZATION → RELOCALIZE           | 定位丢失               | 协方差超阈值/定位丢失                                        | —       | E_NAV_LOCALIZATION_LOST                                      |
| RELOCALIZE → CHECK_LOCALIZATION           | success                | 重定位成功                                                   | —       | —                                                            |
| RELOCALIZE → NAV_ERROR                    | failure                | 重定位失败                                                   | —       | E_NAV_LOCALIZATION_LOST                                      |
| PLAN_PATH → EXECUTE_BASE_NAV              | success                | 路径规划成功                                                 | —       | —                                                            |
| PLAN_PATH → NAV_ERROR                     | failure                | 规划失败/无可行路径                                          | —       | —                                                            |
| EXECUTE_BASE_NAV → MONITOR_NAV            | success                | 开始执行底盘导航并进入监控                                   | —       | —                                                            |
| MONITOR_NAV → EXECUTE_BASE_NAV            | 继续                   | 导航进行中尚未到位                                           | —       | —                                                            |
| MONITOR_NAV → CHECK_GOAL_REACHED          | 到位                   | 到达目标位                                                   | —       | —                                                            |
| MONITOR_NAV → NAV_ERROR                   | failure                | 卡住/遇障碍                                                  | —       | —                                                            |
| CHECK_GOAL_REACHED → FINE_ALIGN_TO_WALL   | require_fine_alignment | require_fine_alignment=true 且 goal_type∈{LEFT_PHASE,RIGHT_PHASE} | —       | —                                                            |
| CHECK_GOAL_REACHED → CHECK_WORKPOSE_VALID | 不需要                 | 不需要对墙微调                                               | —       | —                                                            |
| FINE_ALIGN_TO_WALL → CHECK_WORKPOSE_VALID | success                | 连续 N 帧法向偏差与距离误差均合格                            | 15s     | —                                                            |
| FINE_ALIGN_TO_WALL → NAV_ERROR            | failure/timeout        | 微调失败或超时（重试 1 次后仍失败）                          | 15s     | E_NAV_FINE_ALIGN_FAIL, E_NAV_FINE_ALIGN_NO_FEEDBACK, E_PERC_LOCAL_SCAN_TIMEOUT |
| CHECK_WORKPOSE_VALID → REPORT_NAV_RESULT  | 偏差合格               | 工作位姿偏差合格                                             | —       | —                                                            |
| CHECK_WORKPOSE_VALID → NAV_ERROR          | 偏差超阈值             | 工作位姿偏差超阈值                                           | —       | —                                                            |
| REPORT_NAV_RESULT → `[*]`                 | success                | 上报导航成功 result 并结束                                   | —       | —                                                            |
| NAV_ERROR → `[*]`                         | failure                | 异常终止，返回 success=false 的 result                       | —       | —                                                            |

### 9.9 PairGraspExecutionFSM（适配 FSM，pair_grasp_execution_node）

双臂抓取全流程：`RECEIVE_PAIR→CHECK_PAIR_VALID→PLAN_PREGRASP→MOVE_TO_PREGRASP→APPROACH_AND_CONTACT→CHECK_VACUUM→ATTACH_BOX_MODEL→PLAN/EXECUTE_EXTRACT→PLAN/EXECUTE_CARRY→RELEASE_BOX→RETREAT_SAFE→REPORT`。任何阶段失败统一进 EXECUTION_ERROR。**此 FSM 是 Isaac 双臂控制链 + 吸盘抓取约束的直接对接对象。**

| From → To                               | Event   | 条件                                                         | Timeout                                          | 异常码                                                       |
| --------------------------------------- | ------- | ------------------------------------------------------------ | ------------------------------------------------ | ------------------------------------------------------------ |
| RECEIVE_PAIR → CHECK_PAIR_VALID         | success | 接收到抓取对 Goal                                            | —                                                | —                                                            |
| CHECK_PAIR_VALID → PLAN_PREGRASP        | success | 抓取对合法                                                   | —                                                | —                                                            |
| CHECK_PAIR_VALID → EXECUTION_ERROR      | failure | 抓取对不合法                                                 | —                                                | —                                                            |
| PLAN_PREGRASP → MOVE_TO_PREGRASP        | success | 预抓取规划成功                                               | —                                                | —                                                            |
| PLAN_PREGRASP → EXECUTION_ERROR         | failure | 预抓取规划失败                                               | —                                                | —                                                            |
| MOVE_TO_PREGRASP → APPROACH_AND_CONTACT | success | 运动到位                                                     | —                                                | —                                                            |
| MOVE_TO_PREGRASP → EXECUTION_ERROR      | failure | 运动失败                                                     | —                                                | —                                                            |
| APPROACH_AND_CONTACT → CHECK_VACUUM     | success | 双臂吸盘贴近箱面                                             | —                                                | —                                                            |
| APPROACH_AND_CONTACT → EXECUTION_ERROR  | failure | 接触失败                                                     | —                                                | —                                                            |
| CHECK_VACUUM → ATTACH_BOX_MODEL         | success | 双臂真空压力均达标并持续 vacuum_hold_ms（150ms）             | 0.8s（buildup 800ms；sensor 200ms 无数据判离线） | E_VAC_NOT_REACHED, E_VAC_UNILATERAL_FAIL, E_VAC_SENSOR_OFFLINE, E_VAC_TIMEOUT |
| CHECK_VACUUM → EXECUTION_ERROR          | failure | 超时或 DUAL 模式一侧未达标/泄漏（重试上限 1，先 RELEASE 再微调） | 同上                                             | 同上                                                         |
| ATTACH_BOX_MODEL → PLAN_EXTRACT         | success | 箱体模型挂载完成                                             | —                                                | —                                                            |
| PLAN_EXTRACT → EXECUTE_EXTRACT          | success | 抽取规划成功                                                 | —                                                | —                                                            |
| PLAN_EXTRACT → EXECUTION_ERROR          | failure | 抽取规划失败                                                 | —                                                | —                                                            |
| EXECUTE_EXTRACT → PLAN_CARRY            | success | 抬离成功                                                     | —                                                | —                                                            |
| EXECUTE_EXTRACT → EXECUTION_ERROR       | failure | 抬离失败                                                     | —                                                | —                                                            |
| PLAN_CARRY → EXECUTE_CARRY              | success | 搬运规划成功                                                 | —                                                | —                                                            |
| EXECUTE_CARRY → RELEASE_BOX             | success | 搬运到位                                                     | —                                                | —                                                            |
| EXECUTE_CARRY → EXECUTION_ERROR         | failure | 搬运失败/掉箱（FATAL）                                       | —                                                | E_MOT_DROP_BOX                                               |
| RELEASE_BOX → RETREAT_SAFE              | success | 释放成功                                                     | —                                                | —                                                            |
| RELEASE_BOX → EXECUTION_ERROR           | failure | 释放失败                                                     | —                                                | —                                                            |
| RETREAT_SAFE → REPORT_EXECUTION_RESULT  | success | 退回安全位完成                                               | —                                                | —                                                            |
| RETREAT_SAFE → EXECUTION_ERROR          | failure | 退回安全位失败                                               | —                                                | —                                                            |
| REPORT_EXECUTION_RESULT → `[*]`         | success | 返回 PairGraspResult，正常结束                               | —                                                | —                                                            |
| EXECUTION_ERROR → `[*]`                 | abort   | 异常终止，返回 success=false + result_code/failed_stage/error_code | —                                                | —                                                            |

### 9.10 SafetyMonitorFSM（并行监控，safety_monitor_node）

全程并行运行：`NORMAL ⇄ WARNING ⇄ EMERGENCY`，监听急停、安全区与通信看门狗，异常升级、原因解除或五阶段清错后回落。**Isaac 侧需将仿真碰撞/急停事件桥接到对应 topic 才能驱动此 FSM。**

| From → To           | Event   | 条件                                          | Timeout | 异常码                                                       |
| ------------------- | ------- | --------------------------------------------- | ------- | ------------------------------------------------------------ |
| NORMAL → WARNING    | failure | WARNING 级：通信丢失/安全区违反/碰撞风险      | —       | —                                                            |
| NORMAL → EMERGENCY  | failure | ESTOP 级：硬件/软件急停                       | —       | —                                                            |
| WARNING → NORMAL    | success | 告警原因解除                                  | 30s     | E_SAFETY_ZONE_VIOLATED, E_SAFETY_COMM_LOST, E_SAFETY_COLLISION_RISK |
| WARNING → EMERGENCY | failure | 告警升级为 ESTOP 级                           | 30s     | 同上                                                         |
| WARNING → EMERGENCY | timeout | 持续 30s 未解除自动升级                       | 30s     | 同上                                                         |
| EMERGENCY → NORMAL  | success | 硬件/软件急停均释放后 /clear_error 五阶段全成 | —       | E_SAFETY_ESTOP_HW, E_SAFETY_ESTOP_SW, E_SAFETY_COMM_LOST, E_SAFETY_WATCHDOG |

> **Isaac 对接提示**：§9.8 BaseNavigationFSM 与 §9.9 PairGraspExecutionFSM 是适配层 FSM，其内部状态直接消费 Isaac 提供的传感/控制接口（见 §3、§4、§5）。Isaac 数字孪生需保证这两条链路上的接口在仿真中可用，FSM 状态才能正常推进。§9.10 SafetyMonitorFSM 依赖 Isaac 把碰撞/急停事件桥接为 `/safety/*`、`/estop_button` 等信号。

---
