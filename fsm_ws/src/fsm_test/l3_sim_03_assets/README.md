# L3-SIM-03 Assets

This directory is the test-only drop zone for MoveIt / IK dry-run assets.

Use it for copies or symlinks of the robot description, MoveIt config,
ros2_control mock hardware config, RViz config, and sample grasp goals needed
by L3-SIM-03.

Nothing in production launch files should depend on this directory.

Expected layout:

```text
l3_sim_03_assets/
  robot_description/
    urdf/
    meshes/
  moveit_config/
    config/
    launch/
  controllers/
  rviz/
  sample_goals/
```

Current supplied robot description:

```text
fsm_ws/src/alfa_robot_v2_arm_v6/
```

See `SUPPLIED_REVIEW.md` for the import decision, validation result, and the
remaining MoveIt design gaps.
