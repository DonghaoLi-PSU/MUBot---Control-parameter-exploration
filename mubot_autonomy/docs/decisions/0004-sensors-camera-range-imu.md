# 0004. Sensors: camera + range fan + IMU

**Status:** accepted (contact whisker dropped; lateral line deferred)

**Decision.** Head-mounted camera (320 × 240, 15 Hz), 5-beam range fan
(±30°, 50 Hz) and IMU (200 Hz). Head swing is minimized mechanically.

**Why.** The camera alone could do avoidance and targets, but range gives cheap
reliable close-range distance. The IMU is needed for heading control. The
whisker's role is covered by two TTC triggers. A lateral line would need a
fluid-disturbance model the hydro plugin doesn't have.

**Consequences.** Underwater, infrared range only reaches centimetres: real
hardware needs ultrasonic sonar. Camera looming needs textured surfaces.
