cmake_minimum_required(VERSION 3.8)
project(wavemap_ros_conversions)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)
voliro_build_cpp()

add_definitions(-w)

# Compiler definitions and options
include(cmake/wavemap-extras.cmake)
add_wavemap_compile_definitions_and_options()

# Libraries
ament_auto_add_library(${PROJECT_NAME}
    src/map_msg_conversions.cc
    src/time_conversions.cc)

# Export
ament_auto_package()
