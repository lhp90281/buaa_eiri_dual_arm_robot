cmake_minimum_required(VERSION 3.8)
project(eiriarm_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include)

# ============================================================================
# 1. Build the pure algorithm library (eiriarm_dynamics)
# ============================================================================
add_library(eiriarm_dynamics SHARED
  src/eiriarm_dynamics.cpp
)

target_include_directories(eiriarm_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(eiriarm_dynamics pinocchio::pinocchio)

# Export the library for use by other packages
ament_export_targets(eiriarm_dynamics_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(pinocchio)

install(TARGETS eiriarm_dynamics
  EXPORT eiriarm_dynamics_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# 2. Build ros2_control plugins
# ============================================================================

# Gravity Compensation Controller Plugin
add_library(gravity_compensation_controller SHARED
  src/gravity_compensation_controller.cpp
)

target_include_directories(gravity_compensation_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(gravity_compensation_controller 
  eiriarm_dynamics
  pinocchio::pinocchio
)

ament_target_dependencies(gravity_compensation_controller
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  sensor_msgs
)

# Joint Impedance Controller Plugin
add_library(joint_impedance_controller_plugin SHARED
  src/joint_impedance_controller_plugin.cpp
)

target_include_directories(joint_impedance_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(joint_impedance_controller_plugin 
  eiriarm_dynamics
  pinocchio::pinocchio
)

ament_target_dependencies(joint_impedance_controller_plugin
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  sensor_msgs
)

# Topic-based Hardware Interface (Bridge for topic-based systems)
add_library(topic_based_hardware_interface SHARED
  src/topic_based_hardware_interface.cpp
)

target_include_directories(topic_based_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(topic_based_hardware_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface eiriarm_controllers.xml)

# Export hardware interface plugin
pluginlib_export_plugin_description_file(hardware_interface eiriarm_hardware_interface.xml)

install(TARGETS 
  gravity_compensation_controller
  joint_impedance_controller_plugin
  topic_based_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# 3. Keep legacy executables for backward compatibility (optional)
# ============================================================================
add_executable(gravity_comp_controller src/gravity_comp_controller.cpp)
target_include_directories(gravity_comp_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(gravity_comp_controller pinocchio::pinocchio)

ament_target_dependencies(gravity_comp_controller
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(joint_impedance_controller src/joint_impedance_controller.cpp)
target_include_directories(joint_impedance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(joint_impedance_controller pinocchio::pinocchio)

ament_target_dependencies(joint_impedance_controller
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS 
  gravity_comp_controller
  joint_impedance_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================================
# 4. Install headers and other files
# ============================================================================
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES 
  eiriarm_controllers.xml
  eiriarm_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
