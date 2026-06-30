# 13 Isaac Sim 全流程仿真联调计划

> 本文冻结 Isaac Sim 接入边界、开发顺序、验收里程碑和当前已落地的前三步。当前开发机未安装 Isaac Sim，因此本阶段只实现不依赖 Isaac SDK 的 ROS2 桥接骨架和文档计划；真实 Isaac world 接入后再补启动脚本与端到端测试。

---

## 1. 目标与边界

目标是在 Isaac Sim 中跑通拆垛任务的全流程：

```text
任务启动
  → 观察位导航
  → groundTruth 箱体输入
  → 墙面建图与 pair 选择
  → 作业位导航与 FINE_ALIGN
  → MoveIt 双臂规划执行
  → 仿真吸附 / 搬运 / 放置
  → 状态更新与下一 pair
```

传感器仿真不作为第一阶段目标。相机、雷达、YOLO 和点云提取由 Isaac groundTruth 替代，但输出仍必须走 FSM 已冻结契约：

```text
/perception/box_detections
/perception/health
```

Isaac 只替代硬件和环境，不替代 FSM 决策。`task_manager`、`wall_destacking_strategy`、`navigation_manager`、`pair_grasp_execution` 的对外接口不因 Isaac 改动。

---

## 2. 架构位置

推荐新增 `isaac_sim_bridge` 包，作为 Isaac 与当前 FSM 契约之间的桥接层：

```text
Isaac Sim world
  ├── container / box stack USD prim
  ├── robot articulation
  ├── base motion / wheel drive
  └── arm articulation
        ↓
isaac_sim_bridge
  ├── isaac_ground_truth_perception_node  → /perception/box_detections, /perception/health
  ├── isaac_chassis_bridge_node           → /chassis_node/status, /chassis_node/reset_fault, /chassis_node/enable
  ├── isaac_base_cmd_bridge_node          ← /cmd_vel 或 /cmd_vel_align
  ├── isaac_joint_trajectory_bridge_node  ← FollowJointTrajectory
  └── isaac_grasp_bridge_node             ↔ /vacuum/cmd, /vacuum/pressure_raw
        ↓
FSM 既有适配层
  navigation_manager_node
  pair_grasp_execution_node
  perception_adapter_node 可关闭或被 groundTruth 节点旁路
```

第一阶段只实现前两个节点：

- `isaac_ground_truth_perception_node`
- `isaac_chassis_bridge_node`

后续 Isaac 安装完成后，再补底盘速度桥、机械臂轨迹桥和抓取约束桥。

---

## 3. 接口约束

### 3.1 groundTruth 感知

`isaac_ground_truth_perception_node` 发布：

| Topic | 类型 | 说明 |
|---|---|---|
| `/perception/box_detections` | `fsm_msgs/BoxDetectionArray` | frame_id 固定为 `base_link`，策略层直接消费 |
| `/perception/health` | `fsm_msgs/PerceptionHealth` | camera/lidar/yolo 在 groundTruth 模式下视为 ok |

输入源分两种：

| 模式 | 当前状态 | 说明 |
|---|---|---|
| `param_truth` | 已实现 | 从参数或 YAML 文件读取箱体真值；无 Isaac 环境也可启动 |
| `isaac_sdk` | 预留 | 后续从 Isaac USD prim / ROS2 bridge 读取箱体 pose |

视野过滤第一版采用几何过滤：

```text
max_distance_m
horizontal_fov_rad
z_min / z_max
```

遮挡、raycast 和动态箱体状态同步放到第二阶段。

### 3.2 底盘状态桥

`isaac_chassis_bridge_node` 提供：

| 端口 | 类型 | 说明 |
|---|---|---|
| `/chassis_node/status` | `diagnostic_msgs/DiagnosticArray` | 供 `navigation_manager_node` 判断 enabled / fault / heartbeat |
| `/chassis_node/reset_fault` | `std_srvs/Trigger` | 清除仿真底盘 fault |
| `/chassis_node/enable` | `std_srvs/Trigger` | 使能仿真底盘 |
| `/estop` | `std_msgs/Bool` Sub | 接收 clear_error 或 safety 链路下发的底盘锁 |
| `/safety/estop` | `std_msgs/Bool` Pub | 向 navigation / grasp 广播仿真急停状态 |

该节点不控制底盘运动，只提供底盘健康和恢复协议。底盘运动后续由 `isaac_base_cmd_bridge_node` 接 `/cmd_vel` / `/cmd_vel_align` 完成。

---

## 4. 开发顺序

### S1：冻结设计与文档

输出：

- 新增本文档。
- 更新 `README`、`01_系统架构与代码骨架`、`03_接口契约`、`05_配置与参数清单`、`09_任务拆解`、`12_M2离线预集成计划`。
- 明确 Isaac 是 L3-ISAAC 轨道，不把 Isaac 私有 Topic 暴露给策略层。

完成状态：已落地。

### S2：新增 `isaac_sim_bridge` 包骨架

输出：

```text
fsm_ws/src/isaac_sim_bridge/
  package.xml
  setup.py
  setup.cfg
  resource/isaac_sim_bridge
  isaac_sim_bridge/
    __init__.py
    config.py
    scene_truth.py
    ground_truth_perception_node.py
    chassis_bridge_node.py
```

约束：

- 不在 import 阶段依赖 Isaac SDK。
- 无 Isaac 环境下 `colcon build --packages-select isaac_sim_bridge` 必须通过。
- 后续 Isaac SDK 读取逻辑只接入 `scene_truth.py` 这一层，不污染 FSM 节点。

完成状态：已落地。

### S3：实现 groundTruth 感知与 chassis bridge

输出：

- `isaac_ground_truth_perception_node`
  - 发布 `/perception/box_detections`。
  - 发布 `/perception/health`。
  - 支持 `param_truth` 数据源和简单视野过滤。
- `isaac_chassis_bridge_node`
  - 发布 `/chassis_node/status`。
  - 提供 reset / enable service。
  - 接 `/estop`，发布 `/safety/estop`。

完成状态：已落地。

### S4：接 Isaac 底盘运动

待 Isaac 安装后实施：

- 新增 `isaac_base_cmd_bridge_node`。
- 订阅 twist_mux 输出 `/cmd_vel`，并确保 `/cmd_vel_align` 在 twist_mux 中优先级高于 Nav2。
- 第一版采用运动学 root pose 控制。
- 第二版再切换到轮式 articulation 控制。

验收：

```text
L3-ISAAC-NAV-01：/navigate_to_pose 能驱动 Isaac 底盘到目标位。
L3-ISAAC-NAV-02：FINE_ALIGN 发布 /cmd_vel_align 后，箱体距离/yaw 误差收敛。
```

### S5：接 Isaac 机械臂轨迹

待 Isaac 安装和 robot USD articulation 对齐后实施：

- 新增 `isaac_joint_trajectory_bridge_node`。
- MoveIt / ros2_control 输出 FollowJointTrajectory。
- Isaac 发布 `/joint_states`。
- 关节名必须与 URDF/SRDF 一致。

验收：

```text
L3-ISAAC-GRASP-01：标准 pair 完成 MoveIt 规划并驱动 Isaac 双臂到预抓位。
L3-ISAAC-GRASP-02：本体自碰或容器碰撞映射 E_PLAN_COLLISION_DETECTED。
```

### S6：接仿真抓取约束

待 S5 通过后实施：

- 新增 `isaac_grasp_bridge_node`。
- 订阅 `/vacuum/cmd`。
- TCP 与 box 满足距离条件时创建 attach / fixed constraint。
- release 时解除约束并更新箱体真值状态。
- 发布 `/vacuum/pressure_raw`。

验收：

```text
L3-ISAAC-FULL-01：单 pair 完成吸附、搬运、放置，groundTruth 不再发布已取走箱体。
L3-ISAAC-FULL-02：急停后机械臂/底盘停止，真空按 hold_on_estop 策略保持。
```

### S7：全流程任务

最终验收：

```text
L3-ISAAC-E2E-01：单墙 happy path 完整跑通。
L3-ISAAC-E2E-02：导航失败 / 底盘 fault / clear_error 后续跑。
L3-ISAAC-E2E-03：抓取规划失败触发换 pair 或重试。
L3-ISAAC-E2E-04：急停 + clear_error + 任务恢复。
```

---

## 5. 参数入口

Isaac 相关参数放在 `fsm_config/params/sim.yaml`：

```yaml
sim:
  isaac:
    enabled: false
    use_sim_time: true

    perception:
      source_mode: "param_truth"     # param_truth | isaac_sdk
      publish_rate_hz: 10.0
      scene_truth_file: ""
      boxes_json: ""
      view:
        max_distance_m: 3.0
        horizontal_fov_rad: 2.2
        z_min: -0.2
        z_max: 2.8

    chassis:
      initial_enabled: true
      initial_fault: false
      status_publish_rate_hz: 10.0
```

`scene_truth_file` 示例：

```yaml
boxes:
  - id: box_w0_r0_c0
    frame_id: base_link
    center: {x: 0.75, y: 0.80, z: 1.80}
    size: {x: 0.40, y: 0.40, z: 0.40}
    yaw: 0.0
    confidence: 1.0
```

---

## 6. 风险与决策

| 风险 | 处理 |
|---|---|
| Isaac world 与 URDF/SRDF 关节名不一致 | S5 前必须逐项比对 joint name，禁止桥接层做隐式重命名 |
| groundTruth 直接发布所有箱体导致策略绕过“视野”约束 | 第一版已加 FOV 过滤；遮挡放第二阶段 |
| Isaac 私有 Topic 泄露到 FSM 策略层 | 只允许 `isaac_sim_bridge` 消费 Isaac 私有接口，策略层仍只订 FSM 契约 |
| 无 Isaac 环境阻塞 CI | 当前包不依赖 Isaac SDK，CI 只构建 ROS2 节点骨架 |
| 仿真通过但真机失败 | L3-ISAAC 只降低流程集成风险，不替代 L3-M2 真机/真实后端验收 |

