# L3-SIM-03 Upstream Import

Source workspace:

```text
/root/blue-sword/FSM_develop/old_codes/alfa_robot-emoji-father-motion-5-updown-lookup-bioik/alfa_robot-emoji-father-motion-5-updown-lookup-bioik
```

Relevant upstream packages found:

```text
ros2_ws/src/alfa_robot_description
ros2_ws/src/alfa_robot_moveit_config
ros2_ws/src/alfa_robot_bringup
ros2_ws/src/bio_ik
ros2_ws/src/trac_ik
ros2_ws/src/pick_ik
```

Imported into this asset area:

```text
robot_description/urdf/alfa_robot_l3_sim.urdf.xacro
robot_description/urdf/alfa_robot_macro.ros2_control.xacro
robot_description/urdf/alfa_robot_v2_arm_v6_raw.urdf
moveit_config/config/alfa_robot.urdf.xacro
moveit_config/config/alfa_robot.ros2_control.xacro
moveit_config/config/alfa_robot.srdf
moveit_config/config/kinematics.yaml
moveit_config/config/joint_limits.yaml
moveit_config/config/moveit_controllers.yaml
moveit_config/config/initial_positions.yaml
moveit_config/config/mujoco_initial_positions.yaml
moveit_config/config/pilz_cartesian_limits.yaml
controllers/ros2_controllers.yaml
rviz/moveit.rviz
moveit_config/launch/*.launch.py
```

Local adaptations:

```text
alfa_robot_l3_sim.urdf.xacro:
  include ros2_control macro from this asset directory
  reuse package://alfa_robot_v2_arm_v6/meshes/... for meshes

moveit_config/config/alfa_robot.urdf.xacro:
  include ../../robot_description/urdf/alfa_robot_l3_sim.urdf.xacro

alfa_robot_v2_arm_v6_raw.urdf:
  reuse package://alfa_robot_v2_arm_v6/meshes/... for meshes
```

Semantic model facts:

```text
MoveIt planning group expected by FSM: dual_v5_arm_with_base
SRDF provides dual_v5_arm_with_base: yes
left tip link expected by FSM: left_v5_tool0
right tip link expected by FSM: right_v5_tool0
SRDF provides both tool links: yes
```

TCP frames:

```text
left_v5_tool0:
  parent: left_v5_link6
  fixed origin: xyz="0 0 0.1" rpy="0 0 0"

right_v5_tool0:
  parent: right_v5_link6
  fixed origin: xyz="0 0 0.1" rpy="0 0 0"
```

Kinematics:

```text
left_v5_arm: KDL
right_v5_arm: KDL
left_v5_arm_with_base: KDL
right_v5_arm_with_base: KDL
dual_v5_arm_with_base: bio_ik/BioIKKinematicsPlugin
dual_v5_arm: bio_ik/BioIKKinematicsPlugin
```

Controllers:

```text
MoveIt controllers:
  torso_controller: pitch, turn
  dual_v5_arm_controller: updown + left_v5_joint1..6 + right_v5_joint1..6

ros2_control controllers:
  joint_state_broadcaster
  torso_controller
  dual_v5_arm_controller
```

Still missing:

```text
ompl_planning.yaml
installed xacro package
installed MoveIt runtime packages
installed ros2_control / controller_manager packages
available bio_ik plugin, either installed or built from upstream source
```

Do not promote the full upstream `alfa_robot_moveit_config` package into
`fsm_ws/src` until the MoveIt dependencies are installed. Its CMake build
requires MoveIt planning libraries that are not part of the current lightweight
FSM test environment.
