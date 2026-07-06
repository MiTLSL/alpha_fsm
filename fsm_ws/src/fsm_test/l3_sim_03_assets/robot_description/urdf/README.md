# URDF / Xacro

Place the real robot model entrypoint here.

Expected examples:

```text
robot.urdf.xacro
robot.ros2_control.xacro
```

The entrypoint must expand with `xacro` and include the complete base, torso,
dual arms, end effectors, collision geometry, and planning frames.
