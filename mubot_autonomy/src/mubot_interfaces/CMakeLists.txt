cmake_minimum_required(VERSION 3.8)
project(mubot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GaitCmd.msg"
  "msg/GaitState.msg"
  "msg/Primitive.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/Target.msg"
  "msg/Landmark.msg"
  "msg/LandmarkArray.msg"
  "msg/ActuatorState.msg"
  "action/SeekTarget.action"
  "action/Explore.action"
  "action/ReturnHome.action"
  "srv/SetMode.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
