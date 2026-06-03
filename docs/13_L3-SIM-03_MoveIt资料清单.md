# 13 L3-SIM-03 MoveIt / IK 仿真资料清单

> 目标：进入 L3-SIM-03 前，把真实机器人模型、MoveIt 配置和 ros2_control mock hardware 资料收齐。本文只定义资料入口和检查项，不在 FSM 业务节点里引入机器人模型依赖。

---

## 1. 必需资料

| 类别 | 必需文件 | 推荐落点 |
|---|---|---|
| 整机模型 | 主 `URDF` 或 `xacro` 入口，能展开完整底盘、升降柱、双臂、末端工具 | `fsm_ws/src/fsm_test/l3_sim_03_assets/robot_description/urdf/` |
| Mesh | URDF 引用到的 `dae` / `stl` / `obj` 等 mesh 文件，路径需可被 ROS package URI 解析 | `fsm_ws/src/fsm_test/l3_sim_03_assets/robot_description/meshes/` |
| SRDF | MoveIt semantic model，含 planning groups、end effectors、disabled collisions | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| Kinematics | `kinematics.yaml`，包含左右臂 group 的 IK solver 配置 | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| Joint limits | `joint_limits.yaml`，与硬件安全范围一致 | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| Planning | `ompl_planning.yaml` 或等效规划 pipeline 配置 | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| ros2_control | mock hardware 所需 `ros2_controllers.yaml`、controller 名称、joint list | `fsm_ws/src/fsm_test/l3_sim_03_assets/controllers/` |
| MoveIt controllers | MoveIt 到 ros2_control 的 controller 映射 | `fsm_ws/src/fsm_test/l3_sim_03_assets/moveit_config/config/` |
| RViz | 可选的 MoveIt/RViz 配置 | `fsm_ws/src/fsm_test/l3_sim_03_assets/rviz/` |
| 测试目标 | 标准 pair 和边界 pair 的输入样例 | `fsm_ws/src/fsm_test/l3_sim_03_assets/sample_goals/` |

---

## 1.1 当前已收到的机械资料

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

当前仍缺：

```text
SRDF / planning groups / end effectors
left/right TCP tool frames
kinematics.yaml
joint_limits.yaml
ompl_planning.yaml
moveit_controllers.yaml
ros2_controllers.yaml
ros2_control mock hardware integration
MoveIt RViz config
```

注意：`business.yaml` 目前期望 `left_v5_tool0`、`right_v5_tool0`，但交付 URDF 只有 `leftjoint6`、`rightjoint6`
作为左右叶子 link。不能直接默认等价，需要由机械/控制设计确认 TCP frame 与工具偏置。

---

## 2. 需要确认的名字

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

## 3. L3-SIM-03 完成前检查

```text
[ ] xacro 能成功展开完整 robot_description
[ ] robot_state_publisher 能发布 TF
[ ] move_group 启动成功，并提供 /move_action
[ ] ros2_control mock_components 启动成功
[ ] joint_state_broadcaster 发布 /joint_states
[ ] MoveIt planning group 名称与 business.yaml 一致
[ ] left/right tip link 名称与 business.yaml 一致
[ ] 标准 pair dry_run 规划成功
[ ] 边界 pair 能触发 5200 / 5201 / 5210 中对应错误映射
```

---

## 4. 边界

- 这些文件只服务 L3-SIM-03 和后续 L3-M2-GRASP-DRY，不进入生产 FSM 节点依赖。
- `fsm_test/l3_sim_03_assets/` 可以放副本，也可以放指向真实机器人包的软链。
- 真机最终仍应使用机器人团队维护的正式 `robot_description` / `moveit_config` 包。
