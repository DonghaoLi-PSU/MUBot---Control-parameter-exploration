# Legacy ROS 1 baseline

The original ROS 1 (catkin) + Gazebo Classic code lives in the repository root
at `../source code/` and is not copied here. It is the reference implementation
the port must reproduce.

| Legacy file | Replaced by |
|---|---|
| `Hydro_plugin.cc` | `src/mubot_gazebo/include/mubot_gazebo/hydro_model.hpp` + `src/hydro_system.cpp` |
| `actuator_plugin.cc` | `src/mubot_gazebo/include/mubot_gazebo/motor_model.hpp` + `src/actuator_system.cpp` |
| `urdf/Hydro_plugin_AR*.cc`, `hydro_parameter.csv` | `src/mubot_gazebo/config/hydro/AR*.yaml`, `HM*.yaml` |
| 18 × `urdf/AR*_AN*_*.urdf` | `src/mubot_description/urdf/mubot.urdf.xacro` + `config/body/AR*.yaml` |
| `meshes/*.STL` | `src/mubot_description/meshes/` (copied) |
| `scripts/EM_pgpe.py` | `src/mubot_learning/mubot_learning/em_pgpe.py` |
| `scripts/start_em_pgpe_sin.py`, `mubot_rob_env.py`, `gazebo_connection.py` | `src/mubot_learning/mubot_learning/trainer.py`, `worker.py`, `reward.py` |
| `config/em_pgpe.yaml` | `src/mubot_learning/config/em_pgpe.yaml` + `cases/*.yaml` |
| `scripts/controllers_connection.py`, `get_gazebo_model_odometry.py` | dropped (unused) |

## Baseline to record before porting

Run the legacy stack for the reference case and a few others, and store the
forward speed of the optimized gaits in `baseline/baseline_speeds.csv`
(columns: `case,noa,K,hm,ar,gamma,speed_mps`). `tools/compare_baseline.py`
checks the Gazebo Harmonic port against it.

### Legacy facts worth knowing

- Reward = mean forward (−x) velocity of the mass-weighted centre of gravity.
  `Hydro_plugin.cc` starts writing `velocity.csv` at t = 4 s, so the first 8000
  rows at 4 kHz cover t = 4–6 s: the last 2 s of each 6.1 s rollout, as in the
  paper.
- Spring stiffness is swept across trials as `5e-3/4 * (4 - trial//2)`; medium
  (0.75) is 3.75e-3.
- The first 0.1 s of every rollout is a "calibration" phase in which the hydro
  plugin zeroes link velocities.
- Parameter vector per rollout: `[A1, φ2, A2, …, A_NoA, f]` followed by the
  stiffness. Joint 1 has phase 0.
