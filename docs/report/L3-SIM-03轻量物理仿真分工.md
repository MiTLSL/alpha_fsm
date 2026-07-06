# L3-SIM-03 轻量物理仿真分工

> 用途：本文件用于三人会议对齐，不作为 FSM 接口规范。长期规范以 `docs/13_L3-SIM-03_MoveIt资料清单.md`、`docs/07_Mock规格与测试用例.md`、`docs/09_任务拆解.md` 为准。

---

## 1. 当前情况

L3-SIM-03 已从原来的 “MoveIt / RViz 运动学验证” 调整为 “Gazebo 轻量物理场景 + MoveIt 2 + ros2_control + mock 感知 + FSM 业务流程验证”。

当前代码和资料已经具备启动仿真工作的基础：

- FSM 主体、mock 节点、`pair_grasp_execution_node` 的 dry-run / fake-real / real MoveGroup 骨架已经存在。
- `sim_world_node`、`fake_nav2_base_node` 和 L3-SIM-01 / L3-SIM-02 smoke 已经覆盖几何真值感知与 FINE_ALIGN 收敛验证。
- 机器人 URDF / xacro、SRDF、kinematics、joint limits、MoveIt controller 映射、ros2_control controller 配置已导入到 `fsm_test/l3_sim_03_assets/`。
- `business.yaml` 中的 `planning_group=dual_v5_arm_with_base`、`left_tip_link=left_v5_tool0`、`right_tip_link=right_v5_tool0` 已和当前 V5 语义层对齐。

当前仍未完成的关键项：

- 本地 ROS 环境还缺 MoveIt、xacro、ros2_control、controller_manager、Gazebo / ros_gz、bio_ik 等运行依赖。
- 还没有 Gazebo world、场地、集装箱、纸箱、物理参数资产。
- 还没有 `bringup_with_light_physics_sim.launch.py`。
- 还没有 Gazebo 真值到 FSM 感知输入的 bridge。
- 还没有 L3-SIM-03 第一版 smoke。
- “接触条件 + 固定约束”的抓取桥接器放在阶段 B，不作为第一版阻塞项。

结论：现在可以进入 L3-SIM-03 仿真准备和第一版打通阶段，但第一目标不是完整抓取接触仿真，而是先让物理场景、规划、控制链路和 FSM 接口跑通。

---

## 2. 工作目的

L3-SIM-03 的目标不是做高保真数字孪生，而是在真机联调前提前暴露以下风险：

- 机器人模型、TCP、关节轴、关节限位是否能被 MoveIt 和 ros2_control 正确使用。
- 场地、集装箱、纸箱等外部物体是否能进入仿真世界，并提供基础碰撞、重力和摩擦行为。
- `pair_grasp_execution_node` 是否能在真实 MoveIt 2 / ros2_control 链路下完成标准 pair 的规划、接近、预抓 dry-run。
- 工作空间边界 pair 是否能稳定触发 5200 / 5201 / 5210 等既有错误码。
- FSM 是否仍只依赖抽象接口，不直接依赖 Gazebo 内部 topic 或 world 细节。
- 后续浏览器 / Foxglove 能否观察机器人状态、流程状态、错误码和任务结果。

第一版验收目标：

```text
Gazebo world + robot_description + ros2_control + MoveIt 2 + mock 真值感知 + 标准 pair 预抓 dry-run
```

阶段 B 增量目标：

```text
TCP / box 接触或距离条件判定 + 固定约束 + 释放 + 失败映射
```

---

## 3. 三人分工

### 3.1 GFQ：机械资产与物理可信度负责人

GFQ 负责把 SolidWorks 侧机器人和场景资产整理成可用于仿真的稳定输入，并判断仿真里出现的几何问题是否来自模型本身。

主要职责：

- 确认机器人本体模型尺寸、坐标系、关节轴方向、关节限位。
- 确认 `left_v5_tool0` / `right_v5_tool0` 的 0.1 m TCP 偏置是否等于最终吸盘 TCP。
- 整理场地、集装箱、纸箱和必要障碍物的模型。
- 为 Gazebo 准备简化 collision mesh，避免直接使用高面数 visual mesh 做碰撞。
- 给纸箱、集装箱提供第一版物理参数建议，包括质量、摩擦、弹性、阻尼。
- 对仿真画面中的穿模、尺度错误、姿态错误、左右臂错位、TCP 不准等问题做机械侧判断。

GFQ 产出物：

- 机器人模型检查说明：尺寸、原点、关节轴、关节限位、左右臂链路、TCP。
- 场地 / 集装箱 / 纸箱 visual mesh 与 collision mesh。
- 纸箱和集装箱物理参数表。
- TCP 偏置确认结论。
- 模型问题清单和修正记录。

第一阶段完成标准：

```text
[ ] 确认 robot_description 中左右臂、升降轴、TCP 命名和机械含义一致
[ ] 提供 1 个集装箱模型和 1 个纸箱模型
[ ] 提供纸箱第一版质量、摩擦、弹性、阻尼参数
[ ] 提供可用于 Gazebo 的简化 collision 资产
```

### 3.2 FHL：仿真运行时与物理场景负责人

FHL 负责把机器人和物体真正放进 Gazebo，并打通仿真侧运行时。FHL 有 Isaac 经验，重点利用其仿真世界、物理参数、接触和约束经验。

主要职责：

- 搭建最小 Gazebo world：地面、机器人、1 个集装箱、1 个纸箱。
- 让机器人能通过 xacro / robot_description 正确 spawn 到 Gazebo。
- 打通 ros2_control 仿真控制链，启动 `joint_state_broadcaster`、`dual_v5_arm_controller`、`torso_controller`。
- 调整纸箱基础物理行为，确保有重力、碰撞和摩擦，不追求高保真材料模型。
- 编写或协助编写 `bringup_with_light_physics_sim.launch.py`。
- 编写 Gazebo 真值 bridge，把纸箱位姿转换成 FSM 需要的 mock 感知输入。
- 阶段 B 编写 `grasp_constraint_bridge`，实现接触或距离条件判定、固定约束、释放和失败注入。
- 准备 Foxglove / RViz 观测布局，方便会议和调试时看到 world、机器人、纸箱和状态。

FHL 产出物：

- Gazebo world / model / physics 资产。
- `bringup_with_light_physics_sim.launch.py`。
- robot spawn + controller 启动说明。
- Gazebo 真值到 `/perception/box_detections` 或 `/box_perception/result` 的 bridge。
- 阶段 B 的 `grasp_constraint_bridge`。
- 仿真观测布局。

第一阶段完成标准：

```text
[ ] Gazebo 能加载地面、机器人、集装箱、纸箱
[ ] 纸箱有基础重力、碰撞、摩擦行为
[ ] ros2_control controller 能启动
[ ] /joint_states 有数据
[ ] Gazebo 真值可以输出到 FSM 感知输入
```

### 3.3 HCJ：ROS / FSM 集成与验收负责人

HCJ 负责把仿真环境和现有 FSM、MoveIt、测试脚本、错误码、文档串起来，保证仿真只作为接口实现之一，不污染生产 FSM。

主要职责：

- 安装和锁定 ROS 依赖：MoveIt 2、xacro、ros2_control、controller_manager、Gazebo / ros_gz、bio_ik。
- 让 `move_group` 能加载当前 robot_description、SRDF、kinematics、joint limits、controller 映射。
- 补齐或确认 `ompl_planning.yaml`。
- 把 `pair_grasp_execution_node` 的 real MoveGroup 骨架接到 L3-SIM-03 第一版 launch。
- 维护 `business.yaml` / `interfaces.yaml` 与 MoveIt 配置中的名称一致性。
- 编写 L3-SIM-03 smoke：检查 world、robot、controller、MoveIt、mock 感知、标准 pair dry-run、边界 pair 错误码。
- 保证 FSM 不订阅 Gazebo 内部 topic，不新增生产 fsm_msgs 字段。
- 维护任务进度、问题清单、验收记录和会议材料。

HCJ 产出物：

- 依赖安装记录和启动说明。
- `move_group` 启动成功记录。
- L3-SIM-03 smoke 脚本。
- L3-SIM-03 第一版验收记录。
- 错误码 5200 / 5201 / 5210 覆盖记录。
- 文档和任务表更新。

第一阶段完成标准：

```text
[ ] xacro 能展开 robot_description
[ ] move_group 能启动并提供 /move_action
[ ] MoveIt planning group 和 tip link 与 business.yaml 一致
[ ] 标准 pair 能完成规划、接近、预抓 dry-run
[ ] 边界 pair 能触发 5200 / 5201 / 5210 中对应错误映射
```

---

## 4. 工作阶段安排

### 阶段 A：最小链路打通

目标：先证明机器人、物理世界、控制链路、MoveIt 和 FSM 接口能连起来。

时间建议：3 到 5 个工作日。

任务：

- GFQ 确认 TCP、关节轴、关节限位，提供集装箱和纸箱简化模型。
- FHL 搭 Gazebo world，spawn 机器人和纸箱，启动 ros2_control。
- HCJ 安装依赖，启动 MoveIt，写最小 L3-SIM-03 smoke。

阶段 A 验收：

```text
[ ] robot_description 能 xacro 展开
[ ] Gazebo world 能加载机器人、地面、集装箱、1 个纸箱
[ ] ros2_control controller 能启动
[ ] /joint_states 正常发布
[ ] move_group 能启动
[ ] 标准 pair 能规划到预抓位
```

### 阶段 B：业务真实性增强

目标：补齐 mock 真值感知、边界 pair、错误码和简化抓取规则。

时间建议：3 到 5 个工作日。

任务：

- GFQ 根据阶段 A 结果修正模型、collision mesh 和物理参数。
- FHL 编写真值 bridge 和 `grasp_constraint_bridge`。
- HCJ 补 L3-SIM-03 smoke 的标准 pair、边界 pair、错误码断言。

阶段 B 验收：

```text
[ ] mock 真值感知能稳定输出目标箱
[ ] 标准 pair 能完成规划、接近、预抓 dry-run
[ ] 边界 pair 能稳定触发 5200 / 5201 / 5210
[ ] 抓取桥接器能按 TCP / box 条件创建和解除固定约束
[ ] 约束失败能映射既有抓取错误码
```

### 阶段 C：演示与复盘

目标：让仿真结果能被会议、排障和后续开发复用。

时间建议：2 到 3 个工作日。

任务：

- GFQ 确认演示模型外观和尺度。
- FHL 准备 Foxglove / RViz 观察布局。
- HCJ 整理启动命令、验收脚本、错误码覆盖表和会议汇报材料。

阶段 C 验收：

```text
[ ] 一条命令能启动 L3-SIM-03 第一版
[ ] 浏览器或 Foxglove 能看到机器人、纸箱、状态、错误码
[ ] 失败时有 rosbag / 日志 / 截图可复盘
[ ] 三人能根据同一份问题清单推进修复
```

---

## 5. 每日进度对齐方式

建议每天固定 15 分钟同步，不讨论长技术细节，只对齐状态和阻塞。

每人按同一格式汇报：

```text
昨天完成：
今天计划：
当前阻塞：
需要谁配合：
是否影响阶段验收：
```

问题统一登记到一个表中：

| ID | 问题 | 负责人 | 类型 | 严重度 | 当前状态 | 下一步 |
|---|---|---|---|---|---|---|
| SIM-001 | TCP 偏置待确认 | GFQ | 机械资产 | 高 | open | 对照 SolidWorks 和 xacro |
| SIM-002 | Gazebo 依赖未安装 | HCJ | 环境 | 高 | open | 安装 MoveIt / Gazebo / ros2_control |
| SIM-003 | 纸箱 collision mesh 未定 | GFQ / FHL | 资产 / 物理 | 中 | open | 给出简化 box collision |

问题类型建议固定为：

```text
机械资产
仿真世界
ROS 依赖
MoveIt / ros2_control
FSM 接口
错误码 / 测试
演示 / 可视化
```

严重度定义：

```text
高：阻塞阶段验收
中：不阻塞启动，但会影响稳定性或准确性
低：体验、文档或后续优化
```

---

## 6. 协作边界

必须遵守：

- FSM 生产节点不得订阅 Gazebo 内部 topic。
- 仿真 topic、world、model、physics 资产只服务 L3-SIM，不进入生产 bringup。
- 第一版不追求吸盘流体、柔性纸箱形变、真实视觉渲染和真实雷达噪声。
- 接触固定约束是阶段 B，不阻塞阶段 A。
- 每次只打开一个大变量：先 world 和 controller，再 MoveIt，再 mock 感知，再抓取约束。

避免的做法：

- 不要一开始就做复杂接触抓取。
- 不要用高面数 visual mesh 直接做碰撞。
- 不要让 FSM 业务节点读取 Gazebo 私有 topic。
- 不要把 Isaac、Gazebo、真机三套模型各自维护成互不一致的版本。
- 不要在模型、控制器、MoveIt、抓取规则都未稳定时做整墙长跑。

---

## 7. 第一周建议排期

| 天 | GFQ | FHL | HCJ | 共同检查 |
|---|---|---|---|---|
| D1 | TCP、关节轴、关节限位确认 | Gazebo 版本和 world 骨架确认 | 安装 MoveIt / xacro / ros2_control / Gazebo / bio_ik | 依赖和模型入口是否能展开 |
| D2 | 提供集装箱、纸箱简化模型 | robot + ground + box spawn | move_group 启动调试 | robot_state_publisher、/joint_states |
| D3 | 修正模型尺度和 collision | controller 启动和纸箱物理调参 | 标准 pair dry-run 规划 | 标准 pair 是否到预抓位 |
| D4 | 补物理参数说明 | 真值 bridge 初版 | L3-SIM-03 smoke 初版 | mock 感知是否进入 FSM |
| D5 | 模型问题收敛 | world / launch 稳定化 | 错误码边界 pair 验证 | 阶段 A 验收会议 |

阶段 A 通过后，再排阶段 B 的 `grasp_constraint_bridge`。

---

## 8. 会议结论模板

每次会议建议输出以下结论：

```text
本次是否仍按 L3-SIM-03 第一版目标推进：是 / 否
阶段 A 当前状态：未开始 / 进行中 / 通过 / 阻塞
最大阻塞项：
本次新增问题：
下次会议前每个人必须交付：
是否进入阶段 B：
```

---

## 9. 当前最小待办

```text
GFQ:
  [ ] 确认左右 TCP 偏置是否为最终吸盘 TCP
  [ ] 输出纸箱 / 集装箱简化 collision 模型
  [ ] 输出第一版物理参数

FHL:
  [ ] 创建最小 Gazebo world
  [ ] spawn 机器人、地面、集装箱、纸箱
  [ ] 打通 ros2_control 仿真 controller

HCJ:
  [ ] 安装 MoveIt / xacro / ros2_control / Gazebo / bio_ik
  [ ] 启动 move_group
  [ ] 编写 L3-SIM-03 第一版 smoke
```
