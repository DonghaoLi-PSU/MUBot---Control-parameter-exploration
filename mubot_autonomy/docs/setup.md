# Setup

## With Docker (recommended)

```bash
cd mubot_autonomy
docker compose -f docker/compose.yaml build
docker compose -f docker/compose.yaml run --rm dev bash
# inside the container
colcon build --symlink-install
source install/setup.bash
```

## Native (Ubuntu 24.04)

1. Install ROS 2 Jazzy and Gazebo Harmonic (`ros-jazzy-ros-gz`).
2. Install dependencies:
   ```bash
   sudo apt install ros-jazzy-robot-localization ros-jazzy-xacro \
     ros-jazzy-robot-state-publisher ros-jazzy-diagnostic-updater \
     ros-jazzy-cv-bridge python3-opencv python3-numpy
   pip install gtsam   # for slam_node
   ```
3. Build from this folder: `colcon build --symlink-install`.

## Tests without ROS

```bash
pip install numpy pytest
python -m pytest src/*/test
```
