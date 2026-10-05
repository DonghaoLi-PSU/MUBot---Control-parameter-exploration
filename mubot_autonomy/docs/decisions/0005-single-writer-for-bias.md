# 0005. One writer for the bias voltage

**Status:** accepted

**Decision.** Only the IMU heading cascade sets bias b. Range and camera
propose a heading reference or request BACKWARD; a fixed-priority arbiter
(teleop > BACKWARD > avoid > target > hold) chooses.

**Why.** Separate controllers on b fight whenever an obstacle is between the
robot and the target. A cascade also absorbs the slow, uncertain bias → turn
rate response.
