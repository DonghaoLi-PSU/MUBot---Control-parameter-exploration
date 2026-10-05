# Deployment (autonomy stack)

```bash
ros2 launch mubot_bringup deploy.launch.py policy:=gamma_star_AN4_Kmed_HM3_AR2.yaml
ros2 action send_goal /mubot/seek_target mubot_interfaces/action/SeekTarget "{target_id: 0}"
```

Bring-up order, each step validated before the next (see `roadmap.md`):

1. Safety plumbing and teleop
2. Heading hold (after `tools/bias_step_test.py`)
3. Speed model + EKF (after `tools/fit_speed_model.py`)
4. Target seeking
5. Obstacle avoidance
6. SLAM
7. Missions

E-stop from the command line:

```bash
ros2 topic pub --once /mubot/estop std_msgs/msg/Bool "{data: true}"
```
