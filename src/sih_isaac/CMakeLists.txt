cmake_minimum_required(VERSION 3.16)
project(sih_isaac)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES sih_rover.urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY generated DESTINATION lib)
install(PROGRAMS
  sih_isaac/utm_goal.py
  DESTINATION lib/${PROJECT_NAME})
# imported by the installed utm_goal.py (it adds its own directory to sys.path)
install(FILES
  sih_isaac/geodesy.py
  sih_isaac/mission_run.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
