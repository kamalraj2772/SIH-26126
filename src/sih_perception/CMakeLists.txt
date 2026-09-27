cmake_minimum_required(VERSION 3.8)
project(sih_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Phase 3: segmentation_camera dataset export, YOLO-seg training/export, detection node.
# Not built yet.

ament_package()
