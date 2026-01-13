cmake_minimum_required(VERSION 3.8)
project(eiriarm_mujoco)

# --- 修复 OpenGL Warning (Policy CMP0072) ---
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

# 默认 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mujoco REQUIRED) 

# 2. 定义可执行文件
add_executable(mujoco_bridge_node src/mujoco_bridge_node.cpp)

# 3. 链接库
ament_target_dependencies(mujoco_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
)

# 链接 Mujoco 库
target_link_libraries(mujoco_bridge_node mujoco::mujoco)

# 4. 安装规则
# 安装节点可执行文件
install(TARGETS mujoco_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# 安装 Python 脚本
install(PROGRAMS scripts/mujoco_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()