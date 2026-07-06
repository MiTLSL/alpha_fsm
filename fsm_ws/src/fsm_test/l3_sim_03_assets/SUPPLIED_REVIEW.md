# Supplied Robot Description Review

Reviewed source:

```text
fsm_ws/src/fsm_test/l3_sim_03_assets/supplied/alfa_robot_v2_arm_v6/
```

Workspace placement:

```text
fsm_ws/src/alfa_robot_v2_arm_v6/
```

The supplied material is a complete ROS 2 description package. It should stay as
a package instead of being flattened into this test asset directory because the
URDF references meshes through:

```text
package://alfa_robot_v2_arm_v6/meshes/...
```

Copied content:

```text
CMakeLists.txt
package.xml
config/joint_names_alfa_robot_v2_arm_v6.yaml
launch/display.launch.py
launch/gazebo.launch.py
meshes/collision/*.STL
meshes/visual/*.STL
rviz/urdf.rviz
textures/
urdf/alfa_robot_v2_arm_v6.csv
urdf/alfa_robot_v2_arm_v6.urdf
export.log
```

Skipped content:

```text
*:Zone.Identifier*
```

These are Windows download metadata streams and should not be used as ROS
assets.

Current model facts:

```text
robot name: alfa_robot_v2_arm_v6
root link: base_link
leaf links: leftjoint6, rightjoint6
links: 16
joints: 15
mesh refs: 32
ros2_control tags: absent
transmission tags: absent
```

URDF joint tree:

```text
base_link
  pitch
    turn
      updown
        leftjoint1
          leftjoint2
            leftjoint3
              leftjoint4
                leftjoint5
                  leftjoint6
        rightjoint1
          rightjoint2
            rightjoint3
              rightjoint4
                rightjoint5
                  rightjoint6
```

Validation already run:

```text
colcon build --symlink-install --packages-select alfa_robot_v2_arm_v6
ros2 pkg prefix alfa_robot_v2_arm_v6
URDF package mesh reference check: 32 refs, 0 missing
ros2 launch alfa_robot_v2_arm_v6 display.launch.py use_rviz:=False use_joint_state_pub:=False
```

Observed warnings:

```text
robot_state_publisher uses backwards-compatible file argument in display.launch.py
base_link has root-link inertia; KDL recommends adding a dummy root link
```

Resolved by the upstream motion-control import:

```text
SRDF semantic model
MoveIt planning groups
left/right TCP tool links
kinematics.yaml
joint_limits.yaml with non-zero velocity/acceleration values
moveit_controllers.yaml
ros2_controllers.yaml for mock hardware
ros2_control mock_components integration in xacro
MoveIt/RViz config
```

Names that need design confirmation:

```text
business.yaml currently expects:
  moveit_planning_group: dual_v5_arm_with_base
  left_tip_link: left_v5_tool0
  right_tip_link: right_v5_tool0

supplied URDF currently provides:
  left leaf link: leftjoint6
  right leaf link: rightjoint6
```

The latest upstream motion-control code provides the missing V5 semantic layer:

```text
left_v5_tool0:
  parent: left_v5_link6
  fixed origin: xyz="0 0 0.1" rpy="0 0 0"

right_v5_tool0:
  parent: right_v5_link6
  fixed origin: xyz="0 0 0.1" rpy="0 0 0"
```

This is now copied into `robot_description/urdf/alfa_robot_l3_sim.urdf.xacro`.
The mechanical/control team should still confirm that this 0.1 m fixed offset is
the intended suction TCP definition.

Still missing before L3-SIM-03 MoveIt dry-run:

```text
ompl_planning.yaml
installed xacro package
installed MoveIt runtime packages
installed ros2_control / controller_manager packages
available bio_ik plugin, either installed or built from upstream source
```
