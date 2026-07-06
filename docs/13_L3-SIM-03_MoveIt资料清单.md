# 13 L3-SIM-03 轻量物理仿真方案与资料清单

> 目标：为 L3-SIM-03 建立一个面向业务流程验证的轻量物理仿真环境。该环境需要能承载机器人本体、场地、集装箱、纸箱、简化传感器和抓取流程，支持在浏览器侧观察状态、轨迹、错误码和流程结果。本文定义推荐方案、资料入口和验收项，不在 FSM 业务节点里直接引入机器人模型依赖。

---

## 1. 推荐方案

### 1.1 方案结论

L3-SIM-03 推荐采用以下分层方案：

```text
Gazebo Sim           : 场景、碰撞、重力、摩擦、简单传感器、纸箱/集装箱物理行为
ros2_control         : 仿真关节控制链路、controller 管理
MoveIt 2             : 双臂/升降轴运动规划、IK、碰撞检测、轨迹生成
FSM / 测试节点       : 业务流程编排、状态跳转、错误码映射、异常分支验证
Foxglove / Web 面板  : 浏览器可视化、流程按钮、日志、错误码、任务状态展示
```

Gazebo 版本建议：优先采用 Gazebo Harmonic；若当前 ROS 2 Humble 环境中的 `ros_gz` / `gz_ros2_control`
依赖未就绪，允许先用同接口的 Gazebo Sim 兼容版本或 Gazebo Classic 兜底。版本选择不能改变 FSM 对外接口。

### 1.2 方案边界

- 本方案不是高保真工业数字孪生，不追求吸盘流体模型、柔性纸箱形变、复杂传感器建模。
- 本方案的重点是“稍微真实的物理场景 + 可控的 mock 感知 + 完整的业务流程验证”。
- `MoveIt` 在此方案中承担规划层职责，不单独作为“仿真环境”。
- `Gazebo` 作为主仿真环境，负责世界、物体和力学；`MoveIt` 只负责让机器人在该世界中进行规划和避障。
- 第一版 L3-SIM-03 的阻塞验收是“物理场景 + 规划/控制链路 + mock 感知 + 错误映射”打通。
- “接触条件 + 固定约束”的抓取桥接器放入阶段 B，不作为第一版阻塞项。

### 1.3 为什么这样选

- 只用 `MoveIt + RViz` 只能验证轨迹和可达性，无法覆盖纸箱、集装箱、重力、摩擦和接触行为。
- 直接上高保真平台会显著增加环境搭建、资产转换、传感器建模和维护成本，不适合当前以 FSM 流程验证为主的目标。
- `Gazebo + MoveIt + ros2_control` 与 ROS 2 生态天然兼容，改造成本最低，也最容易把现有状态机接入。
- 浏览器侧需要的是“业务调试台”，因此应把 3D 观测、状态、错误码和按钮面板一起设计，而不是只转发一个 `RViz` 画面。

---

## 2. 必需资料

下表路径均以仓库内 `sevnova_fsm/` 目录为工程根；从仓库总根访问时需加前缀 `sevnova_fsm/`。

| 类别 | 必需文件 | 推荐落点 |
|---|---|---|
| 整机模型 | 主 `URDF` 或 `xacro` 入口，能展开完整底盘、升降柱、双臂、末端工具 | `fsm_ws/src/fsm_test/l3_sim_03_assets/robot_description/urdf/` |
| Mesh | URDF 引用到的 `dae` / `stl` / `obj` 等 mesh 文件，路径需可被 ROS package URI 解析 | `fsm_ws/src/fsm_test/l3_sim_03_assets/robot_description/meshes/` |
| SRDF | MoveIt semantic model，含 planning groups、end effectors、disabled collisions | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| Kinematics | `kinematics.yaml`，包含左右臂 group 的 IK solver 配置 | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| Joint limits | `joint_limits.yaml`，与硬件安全范围一致 | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| Planning | `ompl_planning.yaml` 或等效规划 pipeline 配置 | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| ros2_control | 仿真控制链路所需 `ros2_controllers.yaml`、controller 名称、joint list | `fsm_ws/src/fsm_test/l3_sim_03_assets/controllers/` |
| MoveIt controllers | MoveIt 到 ros2_control 的 controller 映射 | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| Gazebo world | 场地、集装箱、纸箱、静态障碍物 world/sdf 文件 | `fsm_ws/src/fsm_test/l3_sim_03_assets/worlds/` |
| Gazebo models | 场地模型、集装箱模型、纸箱模型及其 mesh / collision 资源 | `fsm_ws/src/fsm_test/l3_sim_03_assets/gazebo_models/` |
| 物理参数 | 纸箱质量、摩擦、弹性、阻尼等基础参数说明 | `fsm_ws/src/fsm_test/l3_sim_03_assets/physics/` |
| Mock 传感器 | 真值转检测结果的 mock 节点配置，如噪声、延迟、漏检参数 | `fsm_ws/src/fsm_test/l3_sim_03_assets/mock_sensors/` |
| 抓取规则 | 吸附/夹取成功判定规则、释放规则、失败条件 | `fsm_ws/src/fsm_test/l3_sim_03_assets/grasp_rules/` |
| 可视化 | `RViz` 配置、Foxglove layout、Web 面板说明 | `fsm_ws/src/fsm_test/l3_sim_03_assets/visualization/` |
| 测试目标 | 标准 pair、边界 pair 和异常注入样例 | `fsm_ws/src/fsm_test/l3_sim_03_assets/sample_goals/` |

---

## 2.1 当前已收到的机械资料

已将设计交付包作为 ROS 2 描述包放入：

```text
fsm_ws/src/alfa_robot_v2_arm_v6/
```

原因：该 URDF 使用 `package://alfa_robot_v2_arm_v6/meshes/...` 引用 mesh，作为独立 package 放在 `fsm_ws/src`
下最稳妥，避免复制到 placeholder 后路径失效。

本次未复制 `*:Zone.Identifier*` 文件；这些是 Windows 下载标记，不是 ROS 资源。

当前可用：

```text
URDF: alfa_robot_v2_arm_v6.urdf
visual/collision STL meshes: 32 refs, 0 missing
root link: base_link
leaf links: leftjoint6, rightjoint6
```

随后在最新机械臂控制代码中找到了 MoveIt 语义和控制配置，并已导入：

```text
fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/alfa_robot.srdf
fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/kinematics.yaml
fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/joint_limits.yaml
fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/moveit_controllers.yaml
fsm_ws/src/fsm_test/l3_sim_03_assets/controllers/ros2_controllers.yaml
fsm_ws/src/fsm_test/l3_sim_03_assets/robot_description/urdf/alfa_robot_l3_sim.urdf.xacro
fsm_ws/src/fsm_test/l3_sim_03_assets/rviz/moveit.rviz
```

这套配置确认：

```text
planning_group: dual_v5_arm_with_base
left_tip_link: left_v5_tool0
right_tip_link: right_v5_tool0
left_v5_tool0 offset from left_v5_link6: xyz="0 0 0.1" rpy="0 0 0"
right_v5_tool0 offset from right_v5_link6: xyz="0 0 0.1" rpy="0 0 0"
dual_v5_arm_with_base IK solver: bio_ik/BioIKKinematicsPlugin
```

当前仍缺或仍需处理：

```text
ompl_planning.yaml
Gazebo world / models / physics 资产
mock 传感器节点与配置
抓取成功/失败的简化规则定义
Foxglove layout 或浏览器面板方案
WSL 中安装 xacro / MoveIt / ros2_control / controller_manager / Gazebo
bio_ik 插件安装或从旧工程源码构建
确认 left/right TCP 的 0.1 m fixed offset 是否为最终吸盘 TCP 定义
```

注意：机械交付 URDF 原始叶子 link 仍是 `leftjoint6`、`rightjoint6`；最新控制代码通过 V5 语义 xacro
补出了 `left_v5_tool0`、`right_v5_tool0`。后续以控制代码里的 V5 语义层作为 L3-SIM-03 输入，但仍要让机械/控制确认 TCP 偏置。

## 2.2 建议新增的仿真输入

### 世界与物体

```text
场地 floor
集装箱 container
纸箱 box_* 若干
必要静态障碍物 collision_objects
```

### 物理参数

```text
gravity: 默认地球重力
paper_box.mass: 先给近似值，后续按实物修订
paper_box.friction: 先给低到中等摩擦
container.friction: 与纸箱区分配置
contact.damping / restitution: 先用保守值，避免箱体抖动
```

### mock 传感器策略

推荐先采用“真值加扰动”的轻量方案，而不是一开始就做完整视觉算法：

```text
输入: Gazebo 世界中的纸箱真值位姿
输出: FSM 依赖的目标检测结果 topic / service / action
可注入: 高斯噪声、固定偏移、延迟、丢帧、漏检、误检
```

### 抓取机制

阶段 B 推荐使用“接触条件 + 固定约束”的简化抓取规则；第一版 L3-SIM-03 只要求规划、接近和预抓 dry-run
链路成立，不把该规则作为阻塞验收：

```text
当 TCP 到达目标阈值范围内
且姿态满足要求
且接触持续时间满足阈值
则由仿真专用抓取桥接器判定 grasp success
并将纸箱与末端建立固定约束

放置阶段解除约束
若超时 / 偏差过大 / 接触不成立，则返回 grasp failed
```

---

## 3. 需要确认的名字

这些名字必须和 `business.yaml` / MoveIt config 对齐：

```text
planning_group: business.pair_grasp_execution.moveit_planning_group
left_tip_link: business.pair_grasp_execution.left_tip_link
right_tip_link: business.pair_grasp_execution.right_tip_link
moveit_action: interfaces.actions.moveit_move_group
apply_planning_scene: interfaces.services.moveit_apply_planning_scene
dual_arm_controller: /dual_v5_arm_controller/follow_joint_trajectory
torso_controller: /torso_controller/follow_joint_trajectory
```

---

## 4. L3-SIM-03 完成前检查

```text
[ ] xacro 能成功展开完整 robot_description
[ ] Gazebo world 能正常加载场地、集装箱、纸箱
[ ] 纸箱具备基础重力、碰撞和摩擦行为
[ ] robot_state_publisher 能发布 TF
[ ] move_group 启动成功，并提供 /move_action
[ ] ros2_control 仿真 controller 启动成功
[ ] joint_state_broadcaster 发布 /joint_states
[ ] MoveIt planning group 名称与 business.yaml 一致
[ ] left/right tip link 名称与 business.yaml 一致
[ ] mock 传感器能输出可控的目标检测结果
[ ] 标准 pair 能完成规划、接近和预抓 dry-run 主链路
[ ] 边界 pair 能触发 5200 / 5201 / 5210 中对应错误映射
[ ] 浏览器侧能看到机器人状态、流程状态、错误码和任务结果
```

阶段 B 增量检查：

```text
[ ] 仿真抓取桥接器能读取 TCP / box 接触或距离条件
[ ] 抓取成功时能创建纸箱到末端的固定约束
[ ] 放置阶段能解除约束
[ ] 接触超时、姿态偏差和约束失败能映射到既有抓取错误码
```

---

## 5. 开发阶段建议

### 阶段 A：先打通主链路

- 完成 `URDF/xacro + MoveIt + ros2_control + Gazebo` 的基本联通。
- 用最简单 world 放机器人、地面、1 个集装箱、1 个纸箱。
- 纸箱先只启用基本碰撞和重力，不追求真实材料参数。
- 感知先直接用真值，不加扰动。
- 抓取先停在规划、接近、预抓和错误映射，不要求接触固定约束。

### 阶段 B：补业务真实性

- 给 mock 传感器加入噪声、延迟、丢帧和漏检。
- 增加标准 pair、边界 pair、异常注入场景。
- 建立抓取成功/失败规则，并打通对应错误码。
- 增加仿真专用抓取桥接器，实现接触条件判定、固定约束和释放。

### 阶段 C：补浏览器调试台

- 在浏览器中展示任务按钮、状态机当前节点、错误码、纸箱检测结果和执行日志。
- 3D 观测优先接 Foxglove；`RViz` 保留给本地开发调试。

---

## 6. 文档修改原则

- 把“MoveIt 仿真”改成“轻量物理仿真”，避免误导读者以为只靠 `MoveIt` 就能覆盖世界和物理行为。
- 把资料清单从“机器人模型清单”扩展成“机器人 + 世界 + 物理 + 感知 + 可视化”的完整输入清单。
- 把验收项从“能规划”升级为“能跑规划/接近/预抓主链路并显示结果”；阶段 B 再覆盖接触约束抓取。
- 明确 `MoveIt`、`Gazebo`、mock 传感器和浏览器面板各自职责，后续讨论时不容易再卡在概念争论上。

---

## 7. 边界

- 这些文件只服务 L3-SIM-03 和后续 L3-M2-GRASP-DRY，不进入生产 FSM 节点依赖。
- `fsm_test/l3_sim_03_assets/` 可以放副本，也可以放指向真实机器人包的软链。
- 真机最终仍应使用机器人团队维护的正式 `robot_description` / `moveit_config` 包。
- FSM 只依赖抽象接口，不应直接依赖 `Gazebo` 内部 topic 或 world 细节；仿真环境只是接口实现之一。
