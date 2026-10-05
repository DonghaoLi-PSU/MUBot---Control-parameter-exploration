Optional Nav2 integration (docs/architecture/ros2_port.md, Fig. 3 of the ROS 2 page).

- Planner: Smac Hybrid-A* with Reeds-Shepp motion (μBot can reverse).
- Controller output `cmd_vel` → `twist_to_primitive` (mubot_control) → primitive.
- `collision_monitor` on `/mubot/range` slows or stops the robot.
- μBot cannot hold position, so goal tolerances must be loose, and its minimum
  useful speed is above zero.

Add `nav2_params.yaml` and behavior trees here when missions need planning.
