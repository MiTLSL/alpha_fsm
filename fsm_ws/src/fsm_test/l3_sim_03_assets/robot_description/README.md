# Robot Description Drop Zone

Put the complete robot description entrypoint here, preferably as a xacro file.

The currently supplied mechanical package has been placed as a real ROS 2
package instead of being copied here:

```text
fsm_ws/src/alfa_robot_v2_arm_v6/
```

The latest upstream motion-control code provided that wrapper layer, now copied
here:

```text
urdf/alfa_robot_l3_sim.urdf.xacro
urdf/alfa_robot_macro.ros2_control.xacro
urdf/alfa_robot_v2_arm_v6_raw.urdf
```

`alfa_robot_l3_sim.urdf.xacro` maps the raw v6 model into V5 semantic names such
as `left_v5_joint1..6`, `right_v5_joint1..6`, `left_v5_tool0`, and
`right_v5_tool0`.

Required before L3-SIM-03:

- Main URDF/xacro entrypoint.
- All mesh files referenced by the model.
- ros2_control tags for mock hardware, if the official description keeps them in the model.
- Frame names for `base_link`, `body`, left/right arm chains, and tool links.

Do not create a dummy URDF here for validation. L3-SIM-03 should use the real robot geometry.
