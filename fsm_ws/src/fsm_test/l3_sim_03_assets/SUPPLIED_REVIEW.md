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

Missing before L3-SIM-03 MoveIt dry-run:

```text
SRDF semantic model
MoveIt planning groups
left/right TCP or tool links
kinematics.yaml
joint_limits.yaml with non-zero velocity/acceleration values
ompl_planning.yaml
moveit_controllers.yaml
ros2_controllers.yaml for mock hardware
ros2_control or mock_components integration
MoveIt/RViz config
sample target pairs tied to confirmed tool frames
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

Do not silently map `left_v5_tool0` to `leftjoint6` or `right_v5_tool0` to
`rightjoint6`. The mechanical/control design should confirm the actual TCP
frames, tool offsets, and naming contract. If needed, add fixed tool links to a
wrapper xacro or an updated official URDF.
