cmake_minimum_required(VERSION 3.8)
project(rosbag2_compressor_zlib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-Wno-implicit-fallthrough)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(ZLIB REQUIRED)

add_library(${PROJECT_NAME} src/zlib_compressor.cpp src/zlib_utils.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> ${ZLIB_DEPS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROSBAG2_COMPRESSOR_ZLIB_BUILDING_LIBRARY")
target_link_libraries(${PROJECT_NAME} z)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rosbag2_compression
)

pluginlib_export_plugin_description_file(rosbag2_compression plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rosbag2_compression)

ament_package()
