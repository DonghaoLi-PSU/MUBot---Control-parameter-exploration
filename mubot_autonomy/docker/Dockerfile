# Development image: ROS 2 Jazzy + Gazebo Harmonic + project dependencies.
FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-robot-localization ros-jazzy-xacro \
      ros-jazzy-teleop-twist-keyboard ros-jazzy-diagnostic-aggregator \
      ros-jazzy-cv-bridge python3-opencv python3-numpy python3-yaml python3-pip \
      python3-pytest python3-matplotlib libyaml-cpp-dev xterm \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages gtsam

WORKDIR /ws
CMD ["bash"]
