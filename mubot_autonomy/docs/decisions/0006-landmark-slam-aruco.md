# 0006. Landmark pose-graph SLAM on ArUco markers

**Status:** accepted

**Decision.** SE(2) pose-graph SLAM (GTSAM iSAM2) with ArUco markers as
landmarks, odometry from a fitted gait speed model + IMU heading, and an
occupancy grid built from the range fan using optimized poses.

**Why.** 5 beams are too sparse for laser SLAM; visual-inertial SLAM needs
texture, compute and gravity. Marker IDs make data association free.

**Consequences.** The tank needs markers about 20 cm apart. Position drifts
between marker sightings.
