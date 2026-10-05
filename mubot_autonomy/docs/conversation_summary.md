# How this design was reached

A record of the design conversation that produced this project, in order.
Each step lists what was decided and why. Design decisions with lasting
consequences also have their own record in `docs/decisions/`.

## 1. Understanding the original repository

The repository holds the code for the IROS 2022 μBot paper: a ROS 1 (catkin) +
Gazebo Classic package, `des_mubot`.

- `Hydro_plugin.cc` applies a custom per-segment hydrodynamic model: added
  mass (reactive) plus drag and skin friction (resistive).
- `actuator_plugin.cc` drives each active joint with
  `V = A·sin(2π(f·t + φ))`, clipped to ±15 V, through a DC-motor model and a
  torsional spring. The last joint is a passive spring.
- `EM_pgpe.py` + `start_em_pgpe_sin.py` optimize the gait with reward-weighted
  EM (EPHE): 50 rollouts per episode, top 25 kept, 40 episodes.
- Python and the plugins communicate through CSV files in `result/DD/`.

Problems noted: hard-coded `/home/donghao` paths mixed with relative paths,
a possible `NameError` when `velocity.csv` is empty, `self.reset` without
parentheses, `shutil.rmtree` of old results, NoA = 2 hard-coded in both
plugins, busy-wait loops.

## 2. Reference case: NoA = 4, medium stiffness, HM-3

From Table I of the paper:

| Setting | Paper | Code |
|---|---|---|
| NoA = 4 | 4 actuators | 6 segments, 5 revolute joints (4 active + caudal) |
| Medium stiffness | K̄ = 0.75 | `spring_stiff` = 3.75e-3, caudal 5× |
| HM-3 | Ca = 1, Cp = 0.5 | `Seg_Cm` = 1, `loco_indicator` = 0.5 |
| AR | nominal 2 | AR2 URDF and AR2 hydro constants |

Files needed: `config/em_pgpe.yaml` (joint_number 4), the training script
(fixed stiffness, 3 trials), both plugins (NoA = 4), `hydro_parameter.csv`
(AR2 values, HM-3 coefficients), and `urdf/AR2_AN4_ADD_V3.urdf`. ADD was
inferred to mean "added-mass inertia included" because its link masses equal
structural mass + 0.75 × added mass and its `izz` values are ~15× those of RES.

## 3. Swimming primitives from one forward gait

Given an optimized forward policy γ*:

- **Turn**: add a bias voltage ±b to every active actuator. Steady bend is
  small (~0.2° per volt per joint at medium stiffness), so turning may come
  mostly from stroke asymmetry. Bias eats headroom under the ±15 V clip.
- **Backward**: time-reverse the wave, which equals negating every phase.
  Expected to be slower than forward because the body and tail are not
  symmetric.

## 4. Perception

Sensor options were compared for a 43 × 13.7 × 7 mm head. Final set:
**camera + range fan + IMU**. The contact whisker was dropped; its collision
reflex role went to two independent time-to-collision triggers (range, and
camera looming). The lateral line was set aside because the hydro model has no
fluid field for objects to disturb.

Head swing was initially handled by sampling once per stroke. After the head
swing was minimized, each sensor is processed at its own rate and only a
moving average over one stroke period (1/f) remains.

Range and camera are not interchangeable: the camera can cover obstacle
avoidance (looming, optical flow) but range cannot identify targets. Keeping
both gives cheap reliable close-range checks plus target identity.

## 5. Control

There is only one steering input, the bias b. So:

- range and camera never set b; they propose a desired heading ψ_ref or ask
  for BACKWARD;
- a fixed-priority arbiter picks one: teleop > BACKWARD > avoid > target >
  hold;
- one IMU heading cascade (P on heading → yaw-rate setpoint, PI on yaw rate →
  b) is the only writer of b.

| Sensor | Estimation | Decision |
|---|---|---|
| IMU | complementary filter, gyro bias, 1/f average | heading cascade → b |
| Range | median + per-sector Kalman (d, ḋ), histogram | TTC → BACKWARD; VFH+ → ψ_ref |
| Camera | HSV blob / ArUco + Kalman (θ, d), looming | ψ_ref = ψ + θ; slow down with k(d) |

## 6. Missing system pieces

Compared with a typical ROS robot: TF and `camera_info`, a goal/mission
source, a command watchdog and e-stop, teleop; then odometry/state
estimation, actuator feedback, diagnostics, live tuning, logging. All added.

## 7. SLAM

μBot swims in a plane, so SLAM estimates x, y, heading. Chosen:
**landmark pose-graph SLAM on ArUco markers** (GTSAM), with odometry from a
fitted gait speed model + IMU heading, and an occupancy grid built from the
range fan using the optimized poses. 2D laser SLAM was rejected (5 beams are
too sparse); visual-inertial SLAM deferred (needs texture, compute, and an IMU
that sees gravity; the legacy world has gravity 0).

## 8. ROS 2 + modern Gazebo

ROS 1 Noetic and Gazebo Classic are past end of life, so the project targets
ROS 2 Jazzy + Gazebo Harmonic. Gazebo no longer speaks ROS: the plugins become
gz-sim systems on gz-transport, and `ros_gz_bridge` plus a small custom bridge
(for the custom GaitCmd message) connect to ROS 2. Training drops the CSV file
bus and runs parallel headless workers that step an exact number of
iterations. Nav2 is optional, through an adapter that turns `cmd_vel` into
amplitude scale and bias.

## Correction made while building the project

During the conversation the legacy reward was described as averaging the
*first* 2 s. That was wrong. `Hydro_plugin.cc` only starts writing
`velocity.csv` after t = 4 s, so the reward already covers the last 2 s of
each rollout, matching the paper.
