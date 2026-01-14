cmake_minimum_required(VERSION 3.8)
project(eiriarm_mujoco)

# ==========================================================
# 新增：配置 RPATH (运行时库搜索路径)
# ==========================================================
# 1. 跳过构建时的 RPATH (使用系统默认)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
# 2. 安装时不要清除 RPATH
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
# 3. 将链接库的路径自动添加到 RPATH 中 (这会自动把 MuJoCo 的路径加进去)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# 4. 确保安装后的 RPATH 包含安装目录本身
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(mujoco REQUIRED)

include_directories(include)

# ==========================================================
# 核心修改：明确指定库名为 eiriarm_mujoco_hardware
# ==========================================================
add_library(eiriarm_mujoco_hardware SHARED
  src/mujoco_hardware.cpp
)

target_link_libraries(eiriarm_mujoco_hardware
  mujoco::mujoco
)

target_include_directories(eiriarm_mujoco_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(eiriarm_mujoco_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface eiriarm_mujoco_hardware.xml)

# ==========================================================
# 安装规则
# ==========================================================

# 这里的 TARGETS 名字也要对应修改
install(
  TARGETS eiriarm_mujoco_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES eiriarm_mujoco_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(
    DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()