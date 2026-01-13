cmake_minimum_required(VERSION 3.8)
project(eiriarm_mujoco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Suppress OpenGL GLVND warning
cmake_policy(SET CMP0072 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(mujoco REQUIRED)

add_executable(mujoco_bridge_node src/mujoco_bridge_node.cpp)
target_include_directories(mujoco_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Use ament_target_dependencies for ROS 2 packages
ament_target_dependencies(mujoco_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  ament_index_cpp
)

# Link non-ROS libraries separately
target_link_libraries(mujoco_bridge_node
  mujoco::mujoco
)

install(TARGETS mujoco_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
