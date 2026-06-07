cmake_minimum_required(VERSION 3.8)
project(wheel_foot_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/lifecycle_boot.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
