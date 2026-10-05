# Roadmap

| Step | Work | Done when |
|---|---|---|
| 0 | Record ROS 1 baseline speeds | `legacy/baseline/baseline_speeds.csv` filled |
| 1 | Port hydro + actuator systems | `compare_baseline.py` within 5 % for the reference case |
| 2 | Parallel training workers | re-trained reference case lands near the paper's speed |
| 3 | Sensors + bridge | every topic visible in RViz at its rate |
| 4 | Safety plumbing, teleop | watchdog and e-stop stop the robot; primitives drivable by hand |
| 5 | Heading hold | bias-step model fitted; heading error < 5° in steady swimming |
| 6 | Speed model + EKF | odometry drift measured against ground truth |
| 7 | Target seeking | reaches a visible target from 1 m |
| 8 | Obstacle avoidance | no collisions in the tank world |
| 9 | SLAM | landmark map error < 5 cm against ground truth |
| 10 | Missions | ReturnHome, Explore, SeekTarget succeed |
| 11 | Nav2 (optional) | planned paths around mapped obstacles |
