cmake_minimum_required(VERSION 3.8)
project(sih_localization)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Phase 2: wheel odometry node, EKF config, UTM<->map utility + round-trip test.
# Not built yet -- package intentionally holds no nodes until that phase starts.

ament_package()
