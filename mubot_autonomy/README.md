# μBot Autonomy

Perception, SLAM and closed-loop control for μBot, the magnetic, modular,
undulatory swimming robot from:

> D. Li, H. Deng, Y. E. Bayiz, B. Cheng, "Effects of Design and Hydrodynamic
> Parameters on Optimized Swimming for Simulated, Fish-inspired Robots,"
> IROS 2022. (`docs/paper/IROS22_1603_MS.pdf`)

That work optimized a forward-swimming gait in simulation. This project extends
it into an autonomous robot that senses its surroundings, knows where it is, and
swims to goals. It is built on **ROS 2 Jazzy + Gazebo Harmonic**.

This folder is self-contained. The original ROS 1 code stays in the repository
root under `source code/` and is used only as the reproduction baseline
(see `legacy/README.md`).

## Reference case

| Setting | Value |
|---|---|
| Number of actuators (NoA) | 4 (6 segments: head, 3 body, tail, tip) |
| Joint stiffness | medium, K = 3.75e-3 N·m/rad; caudal joint 5 × K |
| Hydrodynamic model | HM-3 (Ca = 1, Cp = 0.5) |
| Aspect ratio | 2 (paper nominal) |
| Sensors | camera + range fan + IMU, mounted on the head |

## Swimming primitives

All motion comes from one optimized forward gait
γ* = [e₁, Ψ₂, e₂, Ψ₃, e₃, Ψ₄, e₄, f]:

| Primitive | Voltage on actuator j |
|---|---|
| Forward | e_j · sin(2π(f·t + Ψ_j)) |
| Turn left / right | e_j · sin(2π(f·t + Ψ_j)) ± b |
| Backward | e_j · sin(2π(f·t − Ψ_j)), i.e. phases negated |

Voltages are clipped to ±15 V, so the usable bias is |b| ≤ 15 − max e_j.

## Layout

```
docs/          architecture, design decisions, setup guides, the paper
legacy/        pointer to the ROS 1 baseline in ../source code
tools/         offline scripts: baseline comparison, speed-model fit, plots
docker/        Jazzy + Harmonic development image
ci/            CI workflow (move to .github/workflows/ if this becomes its own repo)
src/
  mubot_description    xacro robot model, meshes, body parameters
  mubot_interfaces     messages, actions, services
  mubot_gazebo         hydro + actuator Gazebo systems, worlds, bridge
  mubot_learning       EM-PGPE gait optimization with parallel workers
  mubot_perception     heading, range and vision estimation
  mubot_control        primitives, gait generator, behaviour, heading control
  mubot_localization   gait odometry, EKF config, landmark SLAM
  mubot_mission        SeekTarget / Explore / ReturnHome
  mubot_bringup        launch files, supervisor, RViz, bag profiles
```

Start with `docs/conversation_summary.md` for how the design was reached and
`docs/architecture/system.md` for the full system.

## Status

Scaffold. The pure-math modules (gait primitives, motor model, hydro model,
heading controller, arbiter, VFH+, filters, EM-PGPE, occupancy grid, speed
model) are implemented and unit-tested. ROS nodes and Gazebo systems are
skeletons with the interfaces fixed and TODOs marking what is left. See
`docs/roadmap.md` for the build order.

## Running the tests that work without ROS

```bash
pip install numpy pytest
python -m pytest src/*/test
```
