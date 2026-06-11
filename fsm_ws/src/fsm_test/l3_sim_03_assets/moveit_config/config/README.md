# MoveIt Config Files

Replace the example files in this directory with the real MoveIt config.

The names can differ, but the L3-SIM-03 launch must know where to find:

- SRDF
- kinematics config
- joint limits
- planning pipeline config
- MoveIt controller mapping

Current imported real config files:

```text
alfa_robot.srdf
alfa_robot.urdf.xacro
alfa_robot.ros2_control.xacro
kinematics.yaml
joint_limits.yaml
moveit_controllers.yaml
initial_positions.yaml
mujoco_initial_positions.yaml
pilz_cartesian_limits.yaml
```

The `ompl_planning.example.yaml` file is still only a placeholder; the upstream
motion-control tree did not contain an `ompl_planning.yaml`.
