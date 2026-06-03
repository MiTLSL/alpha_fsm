# Robot Description Drop Zone

Put the complete robot description entrypoint here, preferably as a xacro file.

The currently supplied mechanical package has been placed as a real ROS 2
package instead of being copied here:

```text
fsm_ws/src/alfa_robot_v2_arm_v6/
```

Use this drop zone only for a wrapper xacro, compatibility links, or temporary
test-only overlays that should not be committed back into the supplied package.

Required before L3-SIM-03:

- Main URDF/xacro entrypoint.
- All mesh files referenced by the model.
- ros2_control tags for mock hardware, if the official description keeps them in the model.
- Frame names for `base_link`, `body`, left/right arm chains, and tool links.

Do not create a dummy URDF here for validation. L3-SIM-03 should use the real robot geometry.
