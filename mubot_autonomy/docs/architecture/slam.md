# SLAM

μBot swims in a plane at fixed depth, so SLAM estimates x, y and heading only.

## Pipeline

```
EKF odometry (IMU ψ + gait speed) ─┐
ArUco detections (id, θ, d) ───────┼─► front end ─► pose graph (GTSAM iSAM2, SE(2)) ─► /tf map→odom
                                   │   keyframe every 5 cm or 10°                    ─► /mubot/landmarks
Range scan (5 beams) ──────────────┴──────────────► occupancy mapper (log-odds) ◄── optimized poses
                                                                                     ─► /mubot/map
```

- Odometry factors connect consecutive keyframes.
- Bearing-range factors connect a keyframe to a landmark. Re-seeing a marker
  closes a loop.
- Landmarks pin down the graph; the range fan only builds the map, because
  5 beams are too sparse for scan matching but enough to mark walls once poses
  are known.

## Odometry from the gait

μBot has no wheels. Run the fixed policy at several amplitude scales k, biases
b and both directions; read true speed from the ground-truth `/mubot/odom`;
fit v = g(k, b, mode) as a lookup table (`tools/fit_speed_model.py` →
`config/speed_model.yaml`). The EKF fuses that speed with IMU heading.

## Options considered

| Approach | Verdict | Why |
|---|---|---|
| Landmark pose-graph SLAM (ArUco + GTSAM) | use | works with a small camera and sparse range; data association from marker IDs |
| 2D laser SLAM (slam_toolbox, Cartographer) | no | needs hundreds of beams |
| Visual-inertial SLAM (ORB-SLAM3, VINS, OpenVINS) | later | needs texture, compute, and an IMU that measures gravity |

The legacy world sets gravity to 0, so the simulated accelerometer never sees
gravity. For visual-inertial SLAM later: turn world gravity on and disable it
per link.

See `../tank_setup.md` for marker placement.
