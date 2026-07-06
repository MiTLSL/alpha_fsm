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
