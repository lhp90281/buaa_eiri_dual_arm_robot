cmake_minimum_required(VERSION 3.8)
project(eiriarm_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(include)

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

install(TARGETS gravity_comp_controller
  DESTINATION lib/${PROJECT_NAME})

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

install(TARGETS joint_impedance_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
