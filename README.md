<<<<<<< HEAD
# Alfa WSL ROS2 工作空间

这个工作空间是 WSL2 Ubuntu 侧的 ROS2 工程入口，用来驱动 Windows 侧 Isaac Sim / Isaac Lab 中的机器人模型：

```text
C:\Users\Administrator\Desktop\alfa_ws\alfa_demo_direct
```

目标是验证 `alfa_robot_v2_arm_v7` 的状态机导航夹取流程：

```text
WSL ROS2 状态机
  -> /alfa/command_json
  -> Windows Isaac Sim alfa_controller.py
  -> alfa_robot_v2_arm_v7 底盘、门、双臂、吸盘动作
  -> /clock 与 /alfa/state_json 返回 WSL
```

## 目录结构

```text
/home/sevnova/ros2_ws
  src/
    robot_bringup/              统一 launch 入口
    robot_state_machine/        状态机节点与状态汇总节点
    robot_navigation/           /cmd_vel 桥和导航 action 测试客户端
    robot_msgs/                 验证用轻量消息
    isaac_visualization_bridge/ 验证状态和 Marker 发布节点
  scripts/
    build_ros2_ws.sh
    run_alfa_task_demo.sh
    smoke_alfa_task_demo.sh
    verify_alfa_windows_e2e.sh
    start_windows_isaac.sh
    wait_windows_isaac_ready.sh
    run_full_windows_e2e.sh
    run_alfa_wsl.sh
    smoke_alfa_wsl.sh
```

已有的完整 FSM 包仍然保留在：

```text
/home/sevnova/projects/fsm_simulation_plan/alpha_fsm/fsm_ws
```

本工作空间会 overlay 这套旧 FSM 包，不复制旧代码。

## 构建

在 WSL 中执行：

```bash
alfa_build
```

等价命令：

```bash
bash /home/sevnova/ros2_ws/scripts/build_ros2_ws.sh
```

构建脚本会使用系统 `/usr/bin/python3`，避免 conda Python 干扰 ROS2 Humble 的消息生成。

## Windows 侧启动 Isaac Sim

在 Windows PowerShell 中运行：

```powershell
cd C:\Users\Administrator\Desktop\alfa_ws\alfa_demo_direct
.\run_ros2_direct.bat
```

这个脚本会启动 `alfa_controller.py`，加载：

```text
assets\alfa_env_demo.usda
```

并启用 ROS2 DDS：

```text
订阅: /alfa/command_json
发布: /clock
发布: /alfa/state_json
```

WSL 侧确认 Windows 已经起来：

```bash
ros2 topic info /clock -v
ros2 topic echo /alfa/state_json
```

`/alfa/state_json` 中会包含当前命令、底盘位姿、关节角、门状态和吸盘状态。

也可以从 WSL 侧请求启动 Windows Isaac：

```bash
alfa_start_win
```

等待 Windows Isaac 的 DDS 话题就绪：

```bash
alfa_wait_win
```

一键请求启动 Windows Isaac、等待 DDS、然后跑端到端验收：

```bash
alfa_full_e2e
```

## 直接状态机任务

推荐先用这个入口验证 `alfa_robot_v2_arm_v7` 的真实动作链路：

```bash
alfa_task
```

等价命令：

```bash
ros2 launch robot_bringup alfa_task_demo.launch.py auto_start:=true
```

该状态机会按顺序执行：

```text
RESET_SCENE              复位场景
OPEN_DOORS               打开仓库门和集装箱门
NAVIGATE_TO_CONTAINER    导航到集装箱前
ALIGN_TO_BOX_WALL        贴近箱墙并微调姿态
MOVE_TO_PREGRASP         移动双臂到预抓取位
APPROACH_AND_CONTACT     低速前进接触箱体
VACUUM_GRASP             闭合左右吸盘吸附箱体
EXTRACT_BOX              后退抽箱
CARRY_TO_PLACE           搬运到放置区
RELEASE_BOX              释放箱体
RETREAT_SAFE             退回安全位
COMPLETE                 任务完成并保持静止
```

常用参数：

```bash
alfa_task auto_start:=true wait_for_clock:=true stage_time_scale:=1.0
```

本地快速 smoke，不依赖 Windows Isaac：

```bash
alfa_task_smoke
```

Windows Isaac 已经启动后，跑端到端验收：

```bash
alfa_e2e
```

`alfa_e2e` 会自动启动 WSL 状态机，并检查：

```text
/clock 可见
/alfa/command_json 持续发布
/alfa/state_json 持续返回
/alfa_task/state_json 走到 COMPLETE
Windows 返回的机器人实际状态发生变化
```

状态变化判据包括任意一项：

```text
底盘 x/y/yaw 发生变化
集装箱门角度发生变化
机械臂/升降/回转关节发生变化
```

成功时会输出：

```text
E2E_OK
```

## 真值驱动抓取返回 Demo

`alfa_truth_demo` 使用 Windows Isaac 返回的 `/alfa/state_json` 真值，包括机器人起点和 cargo 箱体位姿，执行当前用于联调的箱墙抓取返回流程。

当前调试目标已经从集装箱门口改为场景里的箱墙：

```text
/World/CargoBoxWall
```

默认抓取对象是箱墙可见面最上面一行的第 1 个和第 3 个箱子：

```text
/World/CargoBoxWall/box_x0_y0_z4
/World/CargoBoxWall/box_x2_y0_z4
```

状态机阶段：

```text
RESET_SCENE              复位场景
CAPTURE_TRUTH            读取 /alfa/state_json 中的机器人起点和箱体真值
PREPARE_BOX_WALL         准备箱墙抓取任务
NAVIGATE_TO_PREGRASP     导航到目标箱子正前方约 1.05 m
ALIGN_TO_TARGET          底盘转向目标箱子
MOVE_TO_PREGRASP         机械臂移动到预抓取姿态
APPROACH_AND_GRASP       末端靠近目标后吸附
RETREAT_WITH_BOX         抓取后后退到安全距离
NAVIGATE_HOME            带箱子回到任务起始点
DROP_AT_START            在起始点释放箱子
COMPLETE                 完成任务
```

吸附条件：

```text
Windows alfa_controller.py 默认 suction_grip_distance = 0.30 m
WSL 状态机在 suction_target 中发送 max_distance = 0.30
snap=True 时也会先检查末端和目标 bbox 间隙不超过 0.30 m
```

Windows Isaac 已经运行后，WSL 侧启动：

```bash
alfa_truth_demo
```

端到端验收：

```bash
alfa_truth_e2e
```

成功时会输出：

```text
TRUTH_DEMO_E2E_OK
```

验收条件包括 `/clock` 可见、`/alfa/state_json` 带有 cargo 真值、目标箱体被吸附过、目标箱体位置发生变化、机器人回到起点附近并释放吸盘。

如果 Windows Isaac 没启动，或者没有发布 `/alfa/state_json`，这个脚本会超时并列出缺失项。

重要：修改 Windows 侧 `C:\Users\Administrator\Desktop\alfa_ws\alfa_demo_direct\alfa_controller.py` 后，必须关闭并重新运行 Windows 端 Isaac 启动脚本。已经在运行的 Isaac 进程不会自动加载新的 Python 代码。

```powershell
cd C:\Users\Administrator\Desktop\alfa_ws\alfa_demo_direct
.\run_ros2_direct.bat
```

本轮调试还在 Windows 控制器中加入了 reset 恢复逻辑：每次收到 WSL 的 `reset=True` 时，会把 `/World/CargoBoxWall` 和其它可吸附 cargo 恢复到 Isaac 启动时捕获的初始局部位姿，并清零速度、恢复碰撞。这用于避免上一次测试把箱子撞飞或留在错误位置后污染下一次 E2E。

常见缺失项含义：

```text
/clock                  Windows Isaac 没启动或 ROS2 DDS 没通
/alfa/state_json        Windows alfa_controller.py 没有发布状态
/alfa/command_json      WSL 状态机没有开始发命令
task_completed          WSL 任务没有走到 COMPLETE
windows_state_changed   Windows 返回的机器人状态没有发生实际变化
```

## 验收记录

2026-07-06 已完成一次真实 Windows Isaac 联调验收：

```bash
TIMEOUT_SEC=120 STAGE_TIME_SCALE=0.5 bash /home/sevnova/ros2_ws/scripts/verify_alfa_windows_e2e.sh
```

结果：

```text
clock=True
/alfa/state_json states=116
/alfa/command_json commands=276
final_stage=COMPLETE
max_move_delta=0.725
max_container_angle_delta=5156.620
E2E_OK
```

这证明 WSL 侧状态机已经通过 DDS 驱动 Windows Isaac 中的 `alfa_robot_v2_arm_v7`，且 Windows 侧返回的机器人状态发生了实际变化。

## 完整旧 FSM 链路

如果要跑旧 FSM 主链路，包括 `task_manager`、`wall_destacking_strategy`、`windows_navigation_bridge_node` 和 `windows_pair_grasp_bridge_node`：

```bash
alfa_bringup auto_start:=true
```

只启动节点，不自动开始任务：

```bash
alfa_bringup auto_start:=false
```

完整链路 smoke：

```bash
alfa_ws_smoke
```

## 手动运动验证

手动打开 `/cmd_vel -> /alfa/command_json` 桥：

```bash
alfa_bringup auto_start:=false enable_cmd_vel_bridge:=true
```

另开一个 WSL 终端发送短时前进命令：

```bash
alfa_cmd_test
```

手动发送导航 action：

```bash
alfa_nav_goal
```

## 关键话题

WSL -> Windows：

```text
/alfa/command_json       std_msgs/msg/String，机器人控制 JSON
/alfa/fsm_event_json     std_msgs/msg/String，状态机事件 JSON
/cmd_vel                 geometry_msgs/msg/Twist，手动速度输入
/navigate_to_pose        fsm_msgs/action/NavigateToPose，导航 action
```

Windows -> WSL：

```text
/clock                   rosgraph_msgs/msg/Clock
/alfa/state_json         std_msgs/msg/String，Windows 侧机器人实时状态 JSON
```

WSL 内部验证：

```text
/alfa_task/state_json
/alfa_truth_demo/state_json
/alfa/validation_status
/behavior_state
/robot/behavior_state
/visualization_marker_array
/fsm/alfa_task_state
/fsm/alfa_truth_demo_state
```

## 常用检查命令

确认 ROS 环境：

```bash
echo $ROS_DISTRO
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
```

确认包已安装：

```bash
ros2 pkg prefix robot_bringup
ros2 pkg executables robot_state_machine
```

观察任务状态：

```bash
ros2 topic echo /alfa_task/state_json
ros2 topic echo /alfa_truth_demo/state_json std_msgs/msg/String --field data
ros2 topic echo /alfa/validation_status
ros2 topic echo /alfa/state_json
```

观察发给 Isaac 的控制命令：

```bash
ros2 topic echo /alfa/command_json std_msgs/msg/String --field data
```

确认当前 Isaac 真值里是否有箱墙目标：

```bash
source /opt/ros/humble/setup.bash
source /home/sevnova/projects/fsm_simulation_plan/alpha_fsm/fsm_ws/install/setup.bash
source /home/sevnova/ros2_ws/install/setup.bash

/usr/bin/python3 - <<'PY'
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

wanted = {
    "/World/CargoBoxWall/box_x0_y0_z4",
    "/World/CargoBoxWall/box_x2_y0_z4",
}

class One(Node):
    def __init__(self):
        super().__init__("one_alfa_state_check")
        self.done = False
        self.create_subscription(String, "/alfa/state_json", self.cb, 10)

    def cb(self, msg):
        data = json.loads(msg.data)
        items = data.get("state", {}).get("cargo", {}).get("items", [])
        for item in items:
            if item.get("path") in wanted:
                print(json.dumps({
                    "path": item.get("path"),
                    "name": item.get("name"),
                    "bbox_center": item.get("bbox_center"),
                    "held_by": item.get("held_by"),
                }, ensure_ascii=False, sort_keys=True))
        self.done = True

rclpy.init()
node = One()
deadline = time.monotonic() + 6.0
try:
    while not node.done and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

## 常见问题

### WSL 看不到 `/clock`

Windows 侧没有启动直接 DDS 模式，或者 Isaac 进程还没完成 ROS2 初始化。重新运行：

```powershell
cd C:\Users\Administrator\Desktop\alfa_ws\alfa_demo_direct
.\run_ros2_direct.bat
```

### WSL 看得到 `/clock`，但机器人不动

检查 WSL 是否正在发布命令：

```bash
ros2 topic echo /alfa/command_json std_msgs/msg/String --field data
```

检查 Windows 是否收到并发布状态：

```bash
ros2 topic echo /alfa/state_json std_msgs/msg/String --field data
```

如果刚刚改过 Windows 侧 `alfa_controller.py`，先重启 Windows Isaac。否则 `/clock` 和 `/alfa/state_json` 可能仍然来自旧控制器。

### `alfa_truth_e2e` 卡在 `APPROACH_AND_GRASP`

当前吸附要求末端和目标 bbox 间隙不超过 `0.30 m`。如果输出类似：

```text
suction did not attach target box_x0_y0_z4
```

说明 ROS 通信和目标选择通常已经正常，但机械臂末端没有进入 0.30 m 吸附距离。处理顺序：

```bash
ros2 topic echo /alfa_truth_demo/state_json std_msgs/msg/String --field data
ros2 topic echo /alfa/command_json std_msgs/msg/String --field data
ros2 topic echo /alfa/state_json std_msgs/msg/String --field data
```

重点看：

```text
stage 是否进入 APPROACH_AND_GRASP
target.path 是否是 /World/CargoBoxWall/box_x0_y0_z4 或 /World/CargoBoxWall/box_x2_y0_z4
suction.left / suction.right 是否变成目标 path
base_pose 是否到达 goal_pose 附近
```

如果流程必须先跑通，可以临时放宽 `contact_distance_m` 或 Windows `suction_grip_distance`；如果要保持 0.30 m 物理约束，则需要继续完善机械臂 IK/末端位姿控制。

### 箱子飞走或下次 E2E 初始目标位置不对

Windows 控制器现在会在 `reset=True` 时恢复 cargo 初始位姿。仍出现污染时，优先重启 Windows Isaac，让 USD 从磁盘重新加载：

```powershell
cd C:\Users\Administrator\Desktop\alfa_ws\alfa_demo_direct
.\run_ros2_direct.bat
```

然后在 WSL 检查目标箱子的 `bbox_center` 是否回到箱墙附近，再运行：

```bash
alfa_truth_e2e
```

### 杀掉上一次 WSL truth demo 进程

```bash
pkill -INT -f '/home/sevnova/ros2_ws/install/robot_state_machine/lib/robot_state_machine/alfa_truth_demo_state_machine_node' 2>/dev/null || true
pkill -INT -f '/home/sevnova/ros2_ws/install/isaac_visualization_bridge/lib/isaac_visualization_bridge/validation_status_node' 2>/dev/null || true
sleep 1
pkill -TERM -f '/home/sevnova/ros2_ws/install/robot_state_machine/lib/robot_state_machine/alfa_truth_demo_state_machine_node' 2>/dev/null || true
pkill -TERM -f '/home/sevnova/ros2_ws/install/isaac_visualization_bridge/lib/isaac_visualization_bridge/validation_status_node' 2>/dev/null || true
```

### 运行脚本时 rclpy 报 Python 版本错误

不要用 conda Python 运行 ROS2 节点。使用本工作空间脚本，例如：

```bash
alfa_build
alfa_task_smoke
alfa_e2e
alfa_full_e2e
```

这些脚本会把系统 Python 放到 PATH 前面。
=======
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

- `统一集成，边界隔离`：整机联调用一个 ROS 2 colcon 集成工作区和统一 launch 编排，但视觉、导航、机械臂、真空等能力按 package / node / 接口契约隔离，不写成一个大节点。
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
│       ├── isaac_sim_bridge/
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
  - `isaac_sim_bridge`：Isaac Sim 全流程仿真桥，第一阶段提供 groundTruth 感知和底盘状态/恢复协议。
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

这意味着 FSM 项目不是完全独立地“各控制各的”，也不是把所有真实算法和驱动塞进状态机包里。状态机只编排任务和恢复路径；Nav2、MoveIt、视觉感知、底盘驱动、真空硬件和 Isaac Sim 仿真可以来自独立源码包或上游工作区，但进入本系统时必须经过 `navigation_manager`、`pair_grasp_execution`、`perception_adapter`、`isaac_sim_bridge`、`vacuum_io` 这些边界节点。

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
| `isaac_sim_bridge` | 仿真适配节点 | Isaac groundTruth 感知、底盘状态和后续底盘/机械臂/抓取桥接 |
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
- `docs/13_isaac_sim_full_flow_plan.md`

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
- `sim.yaml`
  - 轻量仿真和 Isaac Sim 仿真参数
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

- 作为一个已经完成 `M1`、正在推进 `M2/L3-SIM/L3-ISAAC` 的 ROS 2 FSM 工程基线
- 既能支撑纯 mock 开发，也能逐步替换为真实导航、真实抓取、真实感知和 Isaac Sim groundTruth 仿真

## 建议的新接手顺序

如果你是第一次进入这个仓库，建议按下面顺序上手：

1. 读 `docs/01`、`docs/03`、`docs/09`
2. 看 `fsm_core`、`fsm_msgs`
3. 看 `task_manager`、`wall_destacking_strategy`
4. 看 `fsm_config/launch` 和 `fsm_config/params`
5. 运行 `m0_self_check.py`
6. 运行 `m1_mock_bringup_smoke.py`
7. 再根据你负责的模块进入 `navigation_manager`、`perception_adapter`、`pair_grasp_execution`、`isaac_sim_bridge` 等包

## 参考入口

- 架构总览：`docs/01_系统架构与代码骨架.md`
- 接口契约：`docs/03_接口契约.md`
- 错误码与恢复：`docs/04_错误码与恢复策略表.md`
- 配置说明：`docs/05_配置与参数清单.md`
- 测试分层：`docs/07_Mock规格与测试用例.md`
- 路线图：`docs/09_任务拆解.md`
- Isaac 全流程仿真：`docs/13_isaac_sim_full_flow_plan.md`
>>>>>>> 178846c25aef7ff24e4606bee52ccda169b85c39
