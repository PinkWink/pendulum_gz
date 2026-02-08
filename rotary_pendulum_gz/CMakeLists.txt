cmake_minimum_required(VERSION 3.8)
project(rotary_pendulum_gz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds urdf config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pendulum_interface_node.py
    scripts/reset_pendulum_service.py
    scripts/lqr_controller_node.py
    scripts/state_plotter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
