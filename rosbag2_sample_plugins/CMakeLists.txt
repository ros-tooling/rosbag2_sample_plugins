cmake_minimum_required(VERSION 3.8)
project(rosbag2_sample_plugins)

find_package(ament_cmake REQUIRED)

add_custom_target(${PROJECT_NAME})
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)
install(
  DIRECTORY config/
  DESTINATION config/
)
ament_package()
