# MoveIt Config Drop Zone

Put the MoveIt config package files needed for L3-SIM-03 here.

Required before the MoveIt dry-run smoke:

- `config/*.srdf` or `config/robot.srdf`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/ompl_planning.yaml`
- MoveIt controller mapping
- Any upstream launch fragments needed to start `move_group`
