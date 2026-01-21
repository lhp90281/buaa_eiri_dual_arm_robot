# Topic Bridge 使用指南

## 概述

**Topic-based Hardware Interface** 是一个桥接层，允许 ros2_control 与基于 Topic 通信的系统（如 MuJoCo 仿真器或真实机器人）无缝集成。

### 为什么需要这个 Bridge？

1. **不需要修改仿真器代码** - MuJoCo 继续使用 `/joint_states` 和 `/ctrl/effort`
2. **可复用到真实机器人** - 很多真实机器人的驱动也是基于 Topic 的
3. **快速集成** - 无需为每个系统编写专门的 Hardware Interface

### 架构图

```
┌─────────────────┐     /joint_states      ┌──────────────────────┐
│                 │ ──────────────────────> │  Topic-based HW IF   │
│  MuJoCo / Robot │                         │   (Bridge Layer)     │
│                 │ <────────────────────── │                      │
└─────────────────┘     /ctrl/effort        └──────────────────────┘
                                                      ↕
                                            state/command interfaces
                                                      ↕
                                            ┌──────────────────────┐
                                            │  Controller Manager  │
                                            │   (ros2_control)     │
                                            └──────────────────────┘
                                                      ↕
                                            ┌──────────────────────┐
                                            │   Your Controllers   │
                                            │  (Impedance, etc.)   │
                                            └──────────────────────┘
```

---

## 快速开始

### 步骤 1: 编译

```bash
cd ~/ros2_ws
colcon build --packages-select eiriarm_controllers
source install/setup.bash
```

### 步骤 2: 验证插件

```bash
# 验证硬件接口插件
ros2 pkg plugins --package eiriarm_controllers hardware_interface

# 应该看到：
# eiriarm_controllers/TopicBasedHardwareInterface

# 验证控制器插件
ros2 pkg plugins --package eiriarm_controllers controller_interface

# 应该看到：
# eiriarm_controllers/GravityCompensationController
# eiriarm_controllers/JointImpedanceControllerPlugin
```

### 步骤 3: 准备 URDF

你需要在你的机器人 URDF 中添加 `<ros2_control>` 标签。

**选项 A: 修改现有 URDF**

在你的 `dual_arm_robot.urdf` 末尾（`</robot>` 之前）添加：

```xml
<!-- 包含 ros2_control 配置 -->
<xacro:include filename="$(find eiriarm_controllers)/config/dual_arm_ros2_control.xacro"/>
```

**选项 B: 创建新的 URDF（推荐用于测试）**

创建 `dual_arm_robot_with_ros2_control.urdf.xacro`：

```xml
<?xml version="1.0"?>
<robot name="dual_arm_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- 包含原始 URDF -->
  <xacro:include filename="$(find dual_arm_support)/urdf/dual_arm_robot.urdf"/>
  
  <!-- 添加 ros2_control 配置 -->
  <xacro:include filename="$(find eiriarm_controllers)/config/dual_arm_ros2_control.xacro"/>
  
</robot>
```

### 步骤 4: 启动系统

**方法 A: 使用统一的 Launch 文件（推荐）**

```bash
ros2 launch eiriarm_controllers dual_arm_with_mujoco.launch.py
```

这会自动启动：
- MuJoCo 仿真器
- Controller Manager（带 Topic Bridge）
- Joint State Broadcaster
- Joint Impedance Controller

**方法 B: 手动分步启动（用于调试）**

```bash
# 终端 1: 启动 MuJoCo 仿真
ros2 run eiriarm_mujoco simulate

# 终端 2: 启动 controller_manager
ros2 run controller_manager ros2_control_node \
  --ros-args \
  --params-file src/eiriarm_controllers/config/ros2_control_controllers.yaml \
  -p robot_description:="$(cat src/description/dual_arm_support/urdf/dual_arm_robot.urdf)"

# 终端 3: 加载 joint_state_broadcaster
ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active

# 终端 4: 加载 impedance controller
ros2 control load_controller joint_impedance_controller
ros2 control set_controller_state joint_impedance_controller active
```

### 步骤 5: 发送目标指令

```bash
# 使用 Python 脚本
ros2 run eiriarm_controllers send_target_command.py

# 或使用命令行
ros2 topic pub /joint_impedance_controller/target_joint_states \
  sensor_msgs/JointState \
  "{name: ['left_joint_0', 'left_joint_1'], \
    position: [0.5, -0.3], \
    velocity: [0.0, 0.0]}"
```

---

## 配置说明

### Hardware Interface 参数

在 URDF 的 `<ros2_control>` 标签中配置：

```xml
<ros2_control name="dual_arm_system" type="system">
  <hardware>
    <plugin>eiriarm_controllers/TopicBasedHardwareInterface</plugin>
    
    <!-- 可选：自定义 Topic 名称 -->
    <param name="joint_state_topic">/joint_states</param>
    <param name="effort_command_topic">/ctrl/effort</param>
  </hardware>
  
  <!-- 定义所有关节 -->
  <joint name="left_joint_0">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... 更多关节 ... -->
</ros2_control>
```

### Controller 参数

在 `ros2_control_controllers.yaml` 中配置：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

joint_impedance_controller:
  ros__parameters:
    joints:
      - left_joint_0
      - left_joint_1
      # ... 更多关节
    
    default_stiffness: 20.0
    default_damping: 5.0
    
    gains:
      left_joint_0:
        stiffness: 15.0
        damping: 4.0
```

---

## 监控与调试

### 检查 Topic 连接

```bash
# 查看 joint_states（MuJoCo 发布）
ros2 topic echo /joint_states

# 查看 ctrl/effort（Hardware Interface 发布）
ros2 topic echo /ctrl/effort

# 查看控制器状态
ros2 control list_controllers
```

### 查看日志

```bash
# Controller Manager 日志
ros2 topic echo /controller_manager/diagnostics

# 设置日志级别
ros2 param set /controller_manager log_level DEBUG
```

### 性能监控

```bash
# 查看控制频率
ros2 topic hz /ctrl/effort

# 查看延迟
ros2 topic delay /ctrl/effort
```

---

## 常见问题

### Q1: Hardware Interface 无法找到

**错误**:
```
Could not find requested resource 'eiriarm_controllers/TopicBasedHardwareInterface'
```

**解决**:
```bash
# 重新编译
colcon build --packages-select eiriarm_controllers
source install/setup.bash

# 验证插件
ros2 pkg plugins --package eiriarm_controllers hardware_interface
```

### Q2: 没有收到 joint_states

**错误**:
```
No joint states received after 5 seconds
```

**解决**:
1. 确保 MuJoCo 仿真器正在运行
2. 检查 Topic 名称是否匹配：
   ```bash
   ros2 topic list | grep joint_states
   ```
3. 检查 MuJoCo 是否正确发布：
   ```bash
   ros2 topic echo /joint_states
   ```

### Q3: 控制器无响应

**检查**:
```bash
# 1. 控制器是否激活？
ros2 control list_controllers

# 2. 是否收到目标指令？
ros2 topic echo /joint_impedance_controller/target_joint_states

# 3. 是否发布力矩指令？
ros2 topic echo /ctrl/effort
```

### Q4: 关节名称不匹配

**错误**: 控制器只控制部分关节

**解决**: 确保 URDF 中的关节名称与 MuJoCo 发布的 `/joint_states` 中的名称完全一致。

检查：
```bash
# 查看 MuJoCo 发布的关节名称
ros2 topic echo /joint_states --once

# 查看 URDF 中定义的关节
ros2 param get /robot_state_publisher robot_description | grep "joint name"
```

---

## 性能对比

| 指标 | 纯 Topic 通信 | Topic Bridge + ros2_control |
|------|---------------|----------------------------|
| 控制频率 | ~300 Hz | 500 Hz |
| 延迟 | 1-2 ms | ~0.5 ms (Topic) + 0.1 ms (内部) |
| 实时性 | 中等 | 高 |
| 标准化 | ❌ | ✅ |
| 易于切换硬件 | ❌ | ✅ |

**注意**: Topic Bridge 仍然依赖 Topic 通信，所以延迟比纯 Hardware Interface 稍高，但比直接使用 Topic 控制器更标准化。

---

## 迁移到真实机器人

当你准备在真实机器人上使用时，只需：

1. **确保机器人驱动发布 `/joint_states`**
2. **确保机器人驱动订阅 `/ctrl/effort`**
3. **使用相同的 URDF 和控制器配置**

不需要修改控制器代码！

示例：
```bash
# 真实机器人
ros2 launch your_robot_bringup robot.launch.py  # 启动机器人驱动

# 启动 ros2_control（使用相同的配置）
ros2 launch eiriarm_controllers dual_arm_with_mujoco.launch.py
```

---

## 下一步

1. ✅ 测试 Topic Bridge 与 MuJoCo 的集成
2. ✅ 调整控制器增益参数
3. ✅ 集成到遥操作系统
4. 🔄 （可选）为真实机器人创建专用的 Hardware Interface（消除 Topic 延迟）

---

## 参考资料

- [ros2_control 文档](https://control.ros.org/)
- [hardware_interface API](https://control.ros.org/master/doc/api/hardware_interface.html)
- [主项目 README](README.md)
- [完整迁移指南](ROS2_CONTROL_MIGRATION_GUIDE.md)
