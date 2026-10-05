# Simulation

```bash
ros2 launch mubot_bringup sim.launch.py world:=mubot_tank.sdf noa:=4
ros2 launch mubot_bringup teleop.launch.py      # drive primitives by hand
```

- Worlds: `open_water.sdf` (training, no sensors), `mubot_tank.sdf` (deploy).
- Physics: DART, 0.25 ms step, zero gravity.
- Rendering sensors need a GPU. Headless: `gz sim -s --headless-rendering`.
- Before building on the simulation, run `tools/compare_baseline.py` to check
  the port against the ROS 1 baseline.
