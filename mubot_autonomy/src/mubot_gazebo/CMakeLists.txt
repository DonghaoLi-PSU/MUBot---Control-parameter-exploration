cmake_minimum_required(VERSION 3.8)
project(mubot_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mubot_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
# Gazebo Harmonic libraries via the ROS 2 Jazzy vendor packages.
# For Gazebo Jetty, update the version suffixes.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

include_directories(include)

# Gazebo systems (loaded by gz sim, not linked against ROS).
add_library(mubot_hydro_system SHARED src/hydro_system.cpp)
target_link_libraries(mubot_hydro_system gz-sim::gz-sim gz-plugin::register yaml-cpp)

add_library(mubot_actuator_system SHARED src/actuator_system.cpp)
target_link_libraries(mubot_actuator_system
  gz-sim::gz-sim gz-plugin::register gz-transport::gz-transport gz-msgs::gz-msgs)

# ROS 2 ↔ gz bridge for the custom messages.
add_executable(mubot_gz_cmd_bridge src/gz_cmd_bridge.cpp)
target_link_libraries(mubot_gz_cmd_bridge gz-transport::gz-transport gz-msgs::gz-msgs)
ament_target_dependencies(mubot_gz_cmd_bridge rclcpp mubot_interfaces)

install(TARGETS mubot_hydro_system mubot_actuator_system LIBRARY DESTINATION lib)
install(TARGETS mubot_gz_cmd_bridge RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config worlds models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_models test/test_models.cpp)
endif()

# Lets gz sim find the systems by file name (mubot_hydro_system, ...).
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_package()
