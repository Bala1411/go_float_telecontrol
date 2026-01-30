cmake_minimum_required(VERSION 3.8)
project(dsr_a0509_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf xacro meshes launch config ros2_control
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
