cmake_minimum_required(VERSION 3.8)
project(gazebo_grasp_plugin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Protobuf REQUIRED)

# Build protobuf messages
add_subdirectory(msgs)

# Plugin library
add_library(gazebo_grasp_fix SHARED
  src/GazeboGraspFix.cpp
  src/GazeboGraspGripper.cpp
)

target_include_directories(gazebo_grasp_fix PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/msgs>
  ${gazebo_dev_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)

target_link_libraries(gazebo_grasp_fix
  gazebo_grasp_msgs
  ${gazebo_dev_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

# Make Gazebo find this plugin via GAZEBO_PLUGIN_PATH
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/gazebo_grasp_plugin.dsv"
)

install(TARGETS gazebo_grasp_fix
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
