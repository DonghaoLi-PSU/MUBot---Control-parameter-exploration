# 0001. ROS 2 Jazzy + Gazebo Harmonic

**Status:** accepted

**Context.** The original code is ROS 1 Noetic + Gazebo Classic, both end of
life in 2025. Extending it means writing many new nodes.

**Decision.** Build the new stack on ROS 2 Jazzy + Gazebo Harmonic (LTS pair).
Keep the ROS 1 code untouched as the reproduction baseline.

**Consequences.** Plugins become gz-sim systems on gz-transport; a bridge layer
is needed; custom messages need a custom bridge. The physics engine changes
from ODE to DART, so the optimized gait speeds must be re-validated against
the ROS 1 baseline before anything is built on top.
