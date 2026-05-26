# Sevnova FSM

基于 ROS 2 Humble 的分层有限状态机系统，用于双臂拆垛任务的任务编排、策略决策、导航适配、抓取执行、感知适配、安全监控与测试联调。

## 项目目标

这个仓库的目标不是只做单个节点，而是搭建一套可逐步落地到真机的完整控制框架：

- 用 `task_manager` 管理系统级和任务级状态。
- 用 `wall_destacking_strategy` 承载拆垛主流程与子 FSM。
- 用 `navigation_manager`、`pair_grasp_execution`、`perception_adapter`、`vacuum_io` 屏蔽外部系统差异。
- 用 `safety_monitor` 独立承担急停和安全状态闭环。
- 用 `fsm_test`、mock 节点和 smoke 脚本支撑 M0/M1/M2 分阶段验证。

这套工程遵循几个核心原则：

- `adapter-first`：业务 FSM 不直接耦合 Nav2、MoveIt、感知上游或真空硬件。
- `mock-first`：先用 mock 跑通主流程，再逐步替换真实后端。
- `接口先冻结`：`fsm_msgs`、错误码、配置与节点边界优先稳定，减少多人并行开发时的反复返工。

## 当前状态

结合 `docs/09_任务拆解.md`、`docs/10_navigation_manager_设计笔记.md`、`docs/11_pair_grasp_execution_设计笔记.md`、`docs/12_M2离线预集成计划.md`，当前整体状态可以概括为：

- `M0`：接口与骨架已建立。
- `M1`：mock 闭环已经打通，包含 happy path、重试、急停、clear_error、取消等 smoke 覆盖。
- `M2`：真实后端接入与离线预集成在推进中。
- `L3-SIM`：作为真机前的风险前置验证轨道，已开始预备。

也就是说，这个仓库已经不再是纯设计稿，而是一套可构建、可跑 mock、可继续接真实后端的 ROS 2 工作区。

## 仓库结构

```text
sevnova_fsm/
├── docs/                 设计文档、接口契约、配置说明、阶段计划
├── scripts/              自检与 smoke 脚本
├── fsm_ws/               ROS 2 colcon 工作区
│   └── src/
│       ├── fsm_core/
│       ├── fsm_msgs/
│       ├── box_perception_msgs/
│       ├── fsm_config/
│       ├── task_manager/
│       ├── wall_destacking_strategy/
│       ├── navigation_manager/
│       ├── perception_adapter/
│       ├── pair_grasp_execution/
│       ├── vacuum_io/
│       ├── safety_monitor/
│       ├── fsm_test/
│       └── fsm_ui/
└── .github/workflows/    CI 相关配置
```

## 核心架构

系统按“策略层 + 适配层 + 公共层”拆分：

- 公共层：
  - `fsm_core`：FSM 引擎、状态基类、错误码、恢复策略、节点辅助、日志与广播能力。
  - `fsm_msgs`：系统统一 `msg / srv / action` 接口。
  - `box_perception_msgs`：感知上游消息定义，供适配器消费。

- 策略与任务层：
  - `task_manager`：系统总控，维护 `RobotSystemFSM` 与 `TaskFSM`。
  - `wall_destacking_strategy`：拆垛主流程，串联建图、局部感知、pair 选择、导航与抓取。

- 适配层：
  - `navigation_manager`：导航 Action/恢复服务，对接 Nav2 与底盘能力。
  - `pair_grasp_execution`：抓取执行 Action，对接 MoveIt、控制器、真空链路。
  - `perception_adapter`：把感知上游结果转换成系统统一检测消息。
  - `vacuum_io`：真空控制与压力/健康状态链路。

- 安全层：
  - `safety_monitor`：急停、健康与安全状态统一出口，要求独立于业务流程。

- 测试层：
  - `fsm_test`：mock 节点、测试 launch、pytest 用例与 smoke 支撑。

典型调用链如下：

```text
safety_monitor
      ↓
task_manager
      ↓
wall_destacking_strategy
   ↙      ↓         ↘
nav   perception   grasp
 ↓        ↓          ↓
navigation_manager  perception_adapter  pair_grasp_execution
                                           ↓
                                        vacuum_io
```

## 包职责总览

| 包名 | 类型 | 作用 |
| --- | --- | --- |
| `fsm_core` | 公共库 | FSM 引擎、错误码、恢复策略、节点基类、通用 helper |
| `fsm_msgs` | 接口包 | 系统统一 `msg/srv/action` |
| `box_perception_msgs` | 接口包 | 感知上游消息定义 |
| `fsm_config` | 配置/启动 | 参数 YAML、系统 launch 汇总 |
| `task_manager` | 业务节点 | 系统 FSM、任务 FSM、任务控制服务、clear_error 主协议 |
| `wall_destacking_strategy` | 业务节点 | 拆垛主策略、子 FSM 组装、pair 选择与恢复逻辑 |
| `navigation_manager` | 适配节点 | `NavigateToPose`、底盘恢复、FINE_ALIGN/定位检查 |
| `perception_adapter` | 适配节点 | 外部感知结果转内部标准消息，并维护 perception health |
| `pair_grasp_execution` | 适配节点 | `ExecutePairGrasp`，支持 `dry_run / fake_real / real` |
| `vacuum_io` | 适配节点 | 真空命令、压力模拟/采集、健康状态 |
| `safety_monitor` | 安全节点 | 急停与安全状态闭环 |
| `fsm_test` | 测试包 | mock 节点、launch、pytest、集成辅助 |
| `fsm_ui` | 占位包 | UI 侧预留，当前不是主执行闭环的一部分 |

## 文档导航

如果你第一次接手这个仓库，建议按下面顺序阅读：

1. `docs/01_系统架构与代码骨架.md`
2. `docs/03_接口契约.md`
3. `docs/04_错误码与恢复策略表.md`
4. `docs/05_配置与参数清单.md`
5. `docs/07_Mock规格与测试用例.md`
6. `docs/09_任务拆解.md`

补充文档：

- `docs/02_状态规格手册.md`：详细状态卡
- `docs/06_状态机Mermaid图集.md`：状态拓扑与关系图
- `docs/08_UI.md`：UI 边界与定位
- `docs/10_navigation_manager_设计笔记.md`
- `docs/11_pair_grasp_execution_设计笔记.md`
- `docs/12_M2离线预集成计划.md`

## 开发环境

推荐环境：

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- Python 3.10+  

至少需要确认：

```bash
source /opt/ros/humble/setup.bash
command -v ros2
command -v colcon
```

如果你在 WSL2 开发，建议先确认：

- 代理和 `apt` 网络可用
- `ros2` 与 `colcon` 已经就绪
- GUI/rviz 需求时，WSLg 或 DISPLAY 配置正常

## 快速开始

### 1. 构建工作区

```bash
cd /root/blue-sword/FSM_develop/sevnova_fsm/fsm_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. 最小自检

```bash
cd /root/blue-sword/FSM_develop/sevnova_fsm
python3 scripts/m0_self_check.py
```

### 3. 开发期推荐的 mock 闭环启动

```bash
cd /root/blue-sword/FSM_develop/sevnova_fsm/fsm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fsm_config bringup_with_mock.launch.py
```

### 4. 完整链路启动

```bash
cd /root/blue-sword/FSM_develop/sevnova_fsm/fsm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fsm_config bringup_full.launch.py
```

### 5. 仅启动策略主线

```bash
cd /root/blue-sword/FSM_develop/sevnova_fsm/fsm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fsm_config bringup_strategy_only.launch.py
```

## 常用 Launch

`fsm_config/launch` 下目前最常用的入口有 3 个：

- `bringup_full.launch.py`
  - 启动 `safety_monitor`、`task_manager`、`wall_destacking_strategy`
  - 启动 `perception_adapter`、`navigation_manager`、`vacuum_io`、`pair_grasp_execution`
  - 适合完整联调或接真实后端

- `bringup_with_mock.launch.py`
  - 支持 `mock_nav`、`mock_grasp`、`mock_perception`、`mock_vacuum`
  - 开发期最常用，便于半真半 mock 组合验证

- `bringup_strategy_only.launch.py`
  - 只保留最小策略相关节点
  - 适合局部调试策略逻辑

## 关键配置

主要参数位于 `fsm_ws/src/fsm_config/params/`：

- `business.yaml`
  - 业务流程、策略、后端模式选择
- `fsm.yaml`
  - FSM 运行参数
- `interfaces.yaml`
  - topic/service/action 名称与外部接口映射
- `error_codes.yaml`
  - 错误码与恢复策略覆盖
- `logging.yaml`
  - 日志相关配置

当前比较关键的后端模式：

- `navigation_manager.backend_mode`
- `pair_grasp_execution.backend_mode`

M2 期间会常见 `dry_run`、`fake_real`、`real` 等组合，用于把真实链路逐个接入，而不是一次全开。

## 测试与 Smoke

### L0/L1 基础测试

```bash
cd /root/blue-sword/FSM_develop/sevnova_fsm/fsm_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select fsm_test
```

### 常用 smoke 脚本

在仓库根目录下：

```bash
python3 scripts/m0_self_check.py
python3 scripts/m1_mock_bringup_smoke.py
python3 scripts/m1_task_manager_l2_smoke.py
python3 scripts/m1_strategy_l2_failure_smoke.py
python3 scripts/m1_strategy_l3_e2e_smoke.py
python3 scripts/m1_strategy_l3_retry_smoke.py
python3 scripts/m1_strategy_l3_safety_cancel_smoke.py
python3 scripts/m2_pre_perception_replay_smoke.py
python3 scripts/m2_pre_nav_fake_real_smoke.py
python3 scripts/m2_pre_grasp_fake_real_smoke.py
```

这些脚本的定位大致如下：

- `M0`：骨架和接口自检
- `M1`：mock 闭环、失败重试、急停/清错、任务控制
- `M2-pre`：真实感知/导航/抓取接入前的离线预集成

## 典型开发流程

建议的日常开发路径：

1. 修改某个包或参数
2. 在 `fsm_ws/` 下 `colcon build`
3. `source install/setup.bash`
4. 先跑对应 smoke，而不是一上来跑全链路
5. 需要时再跑 `colcon test --packages-select fsm_test`

推荐优先级：

- 算法或状态机逻辑：先跑单测/局部 smoke
- 节点接口改动：先跑 mock bringup
- 适配层联调：先跑对应 `m2_pre_*_smoke.py`
- 全流程变更：最后再跑 full bringup 或 L3 级别 smoke

## 已知边界与注意事项

- 这是一个正在向真实系统收敛的工程，当前不同模块成熟度不完全一致。
- `M1` 的“完成”指 mock 闭环完成，不等于所有真实硬件后端已接通。
- `M2-preintegration` 的“通过”指真实代码路径已能离线验证，不等价于真机现场验收。
- `fsm_ui` 仍是预留包，当前主执行闭环不依赖 UI。
- `fsm_test/launch` 下部分 launch 仍偏测试占位，实际开发中优先使用 `fsm_config/launch`。
- 在 WSL2 环境下，如果要联调真硬件、串口、USB、相机、雷达，实际接入成本通常高于原生 Ubuntu。

## 里程碑概览

来自 `docs/09_任务拆解.md` 的总体路线：

- `M0`：接口冻结与工作区骨架
- `M1`：mock 闭环
- `M2`：适配层接真实后端
- `M3`：实机端到端
- `M4`：上线与长稳

当前仓库更适合的理解方式是：

- 作为一个已经完成 `M1`、正在推进 `M2/L3-SIM` 的 ROS 2 FSM 工程基线
- 既能支撑纯 mock 开发，也能逐步替换为真实导航、真实抓取和真实感知

## 建议的新接手顺序

如果你是第一次进入这个仓库，建议按下面顺序上手：

1. 读 `docs/01`、`docs/03`、`docs/09`
2. 看 `fsm_core`、`fsm_msgs`
3. 看 `task_manager`、`wall_destacking_strategy`
4. 看 `fsm_config/launch` 和 `fsm_config/params`
5. 运行 `m0_self_check.py`
6. 运行 `m1_mock_bringup_smoke.py`
7. 再根据你负责的模块进入 `navigation_manager`、`perception_adapter`、`pair_grasp_execution` 等包

## 参考入口

- 架构总览：`docs/01_系统架构与代码骨架.md`
- 接口契约：`docs/03_接口契约.md`
- 错误码与恢复：`docs/04_错误码与恢复策略表.md`
- 配置说明：`docs/05_配置与参数清单.md`
- 测试分层：`docs/07_Mock规格与测试用例.md`
- 路线图：`docs/09_任务拆解.md`

