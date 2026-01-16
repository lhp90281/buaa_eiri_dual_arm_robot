cmake_minimum_required(VERSION 3.8)
project(eiriarm_mujoco)

# 设置mujoco文件路径
set(MUJOCO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco)
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_LIB_DIR ${MUJOCO_DIR}/lib/libmujoco.so.3.3.0)

# 设置构建时的 RPATH（编译阶段生效）
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
# 添加 MUJOCO_DIR/lib 到 RPATH 搜索路径
set(CMAKE_INSTALL_RPATH "$ORIGIN:$ORIGIN/../lib:${MUJOCO_DIR}/lib")
# 防止系统路径覆盖自定义 RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
# find_package(mujoco REQUIRED) # We use local version
find_package(glfw3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# ==========================================================
# Simulate UI Node
# ==========================================================
aux_source_directory(src/demo SIMULATE_SOURCES)
aux_source_directory(third_party/lodepng SIMULATE_SOURCES)

add_executable(simulate
  ${SIMULATE_SOURCES}
  src/mujoco_simulator_main.cpp
  src/simulate_bridge.cpp
)

target_link_libraries(simulate 
  ${MUJOCO_LIB_DIR}
  glfw 
  yaml-cpp
)

target_include_directories(simulate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lodepng
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tabulate/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/demo
)

target_compile_features(simulate PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(simulate
  "rclcpp"
  "std_srvs"
  "sensor_msgs"
  "geometry_msgs"
  "urdf"
  "kdl_parser"
  "robot_state_publisher"
  "tf2"
  "tf2_ros"
)

# ==========================================================
# 安装规则
# ==========================================================

# 这里的 TARGETS 名字也要对应修改
install(
  TARGETS simulate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


install(
  DIRECTORY include/
  DESTINATION include
)

if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(
    DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()