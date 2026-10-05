# ROS 1 → ROS 2 port

| ROS 1 + Gazebo Classic | ROS 2 + gz sim | Notes |
|---|---|---|
| `Hydro_plugin.cc` ModelPlugin | MuBotHydro system (`ISystemConfigure`, `ISystemPreUpdate`) | math moved to `hydro_model.hpp`, unchanged |
| `actuator_plugin.cc` | MuBotActuator system | `Joint::Position/Velocity` in, `Joint::SetForce` out; commands on a gz topic |
| ODE | DART (default) | keep 0.25 ms step and zero gravity; re-validate gait speed |
| ray, camera, IMU plugins | Sensors system (`gpu_lidar`, `camera`, ogre2) + Imu system | headless rendering needs a GPU with EGL |
| joint states, p3d | JointStatePublisher, OdometryPublisher | bridged |
| `/gazebo/reset_simulation` | `/world/<name>/control` | reset, pause, multi-step |
| catkin, rospy, XML launch | colcon/ament, rclpy/rclcpp, Python launch | |
| latched topics | `transient_local` QoS | e-stop, map, tf_static |
| dynamic_reconfigure | parameters + set-parameter callback | |
| heartbeat supervisor | lifecycle nodes + lifecycle_manager | |
| rosbag | ros2 bag (MCAP) | |
| CSV file bus | gz topics + world control, parallel workers | |

## Bridge

Standard types go through `ros_gz_bridge` (`src/mubot_gazebo/config/bridge.yaml`).
GaitCmd and ActuatorState are custom, so `mubot_gz_cmd_bridge` converts them to
and from `gz.msgs.Double_V`.

GaitCmd packing on the gz side (`Double_V.data`):
`[n, e_1..e_n, psi_1..psi_n, f, bias, stiffness, tail_ratio]`, where n is the
number of active joints.

ActuatorState packing: `[n, v_1..v_n, clipped_1..clipped_n, power]`.

## QoS

| Data | Reliability | Durability | Extras |
|---|---|---|---|
| IMU, range, camera | best effort, depth 5 | volatile | sensor-data profile |
| primitive, gait_cmd | reliable, depth 1 | volatile | deadline 100 ms |
| estop | reliable | transient local | |
| map, tf_static | reliable | transient local | |
| pose, odometry | reliable, depth 10 | volatile | |

## Version choice

ROS 2 Jazzy + Gazebo Harmonic (LTS). ROS 2 Lyrical + Gazebo Jetty is the newer
LTS pair; check package availability (GTSAM especially) before switching.
Verify gz-sim API names against the installed version's docs.
