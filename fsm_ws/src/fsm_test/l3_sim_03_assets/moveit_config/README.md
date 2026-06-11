# MoveIt Config Drop Zone

Put the MoveIt config package files needed for L3-SIM-03 here.

Required before the MoveIt dry-run smoke:

- `config/*.srdf` or `config/robot.srdf`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/ompl_planning.yaml`
- MoveIt controller mapping
- Any upstream launch fragments needed to start `move_group`

Imported from the latest upstream motion-control code:

```text
config/alfa_robot.srdf
config/alfa_robot.urdf.xacro
config/alfa_robot.ros2_control.xacro
config/kinematics.yaml
config/joint_limits.yaml
config/moveit_controllers.yaml
config/initial_positions.yaml
config/mujoco_initial_positions.yaml
config/pilz_cartesian_limits.yaml
launch/*.launch.py
```

Still missing:

```text
config/ompl_planning.yaml
```

`kinematics.yaml` uses `bio_ik/BioIKKinematicsPlugin` for the dual-arm groups,
so L3-SIM-03 also needs bio_ik available in the ROS environment.
