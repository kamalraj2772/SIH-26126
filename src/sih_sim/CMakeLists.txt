cmake_minimum_required(VERSION 3.8)
project(sih_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  sih_sim/heightmap_generator.py
  sih_sim/world_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

# worlds/ and heightmaps/ are deliberately not installed: they hold only
# seed-generated output, which heightmap_generator/the launch file create on
# demand under share/sih_sim. Keeping them out means a fresh clone (where they
# are gitignored and therefore absent) still configures and builds.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
