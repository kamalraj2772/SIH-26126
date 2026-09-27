cmake_minimum_required(VERSION 3.8)
project(sih_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Phase 2: mission.yaml, mission_tf_publisher, mission_goal_sender, Nav2 bring-up.
# Phase 3: custom nav2_costmap_2d::Layer C++ plugin (will require a C++ target here).
# Not built yet.

ament_package()
