# EiriArm Bringup Package

快速启动 EiriArm 双臂机器人仿真和控制系统。

## 📦 功能

- **MuJoCo 仿真**：启动 MuJoCo 物理仿真器
- **多种控制器**：支持阻抗控制、重力补偿、轨迹跟踪
- **一键启动**：完整系统或分离启动

---

## 🚀 快速开始

### 方式1：完整系统启动（推荐）

一键启动 MuJoCo 仿真 + 控制器：

```bash
# 阻抗控制器（默认）
ros2 launch eiriarm_bringup system.launch.py

# 重力补偿控制器
ros2 launch eiriarm_bringup system.launch.py controller_type:=gravity_compensation

# 轨迹跟踪控制器
ros2 launch eiriarm_bringup system.launch.py controller_type:=joint_trajectory
```

### 方式2：分离启动

分别启动仿真和控制器（用于调试）：

```bash
# 终端1：启动 MuJoCo 仿真
ros2 launch eiriarm_bringup mujoco_sim.launch.py

# 终端2：启动控制器
ros2 launch eiriarm_bringup controllers.launch.py controller_type:=impedance
```

---

## 🎮 控制器类型

### 1. 阻抗控制器（impedance）

**特点**：
- 柔顺控制，允许外力干扰
- 自动保持初始姿态
- 可通过话题发送目标位置

**启动**：
```bash
ros2 launch eiriarm_bringup system.launch.py controller_type:=impedance
```

**发送目标命令**：
```bash
# 使用示例脚本
ros2 run eiriarm_controllers send_target_command.py

# 或手动发布
ros2 topic pub /joint_impedance_controller/target_joint_states sensor_msgs/msg/JointState \
  "{name: ['right_joint_0'], position: [0.5]}"
```

**配置文件**：
- `/src/eiriarm_controllers/config/ros2_control_controllers.yaml`

**参数调整**：
```yaml
# 刚度和阻尼
gains:
  right_joint_0:
    stiffness: 30.0  # 位置恢复力
    damping: 6.0     # 速度阻尼

# 安全限制
position_error_limit: 1.5      # 最大位置误差（rad）
velocity_saturation: 8.0       # 速度误差限制（rad/s）
velocity_filter_alpha: 0.98    # 速度滤波强度
```

---

### 2. 重力补偿控制器（gravity_compensation）

**特点**：
- 仅补偿重力
- 机器人可自由移动（零刚度）
- 用于示教或手动操作

**启动**：
```bash
ros2 launch eiriarm_bringup system.launch.py controller_type:=gravity_compensation
```

**配置文件**：
- `/src/eiriarm_controllers/config/ros2_control_controllers.yaml`

---

### 3. 轨迹跟踪控制器（joint_trajectory）

**特点**：
- PID 位置控制
- 精确轨迹跟踪
- 适合预定义运动

**启动**：
```bash
ros2 launch eiriarm_bringup system.launch.py controller_type:=joint_trajectory
```

**发送轨迹命令**：
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['right_joint_0', 'right_joint_1'], 
    points: [{positions: [0.5, 0.3], time_from_start: {sec: 2}}]}"
```

**配置文件**：
- `/src/eiriarm_bringup/config/controllers.yaml`

---

## 📊 话题和服务

### 通用话题

```bash
# 关节状态（所有控制器）
/joint_states                    # sensor_msgs/msg/JointState

# 机器人描述
/robot_description               # std_msgs/msg/String
```

### 阻抗控制器专用

```bash
# 目标位置命令
/joint_impedance_controller/target_joint_states  # sensor_msgs/msg/JointState

# 反馈（可选）
/joint_impedance_controller/feedback             # sensor_msgs/msg/JointState
```

### 轨迹控制器专用

```bash
# 轨迹命令
/joint_trajectory_controller/joint_trajectory    # trajectory_msgs/msg/JointTrajectory

# 状态反馈
/joint_trajectory_controller/state               # control_msgs/msg/JointTrajectoryControllerState
```

### 控制器管理服务

```bash
# 列出所有控制器
ros2 control list_controllers

# 切换控制器
ros2 control switch_controllers \
  --activate joint_impedance_controller \
  --deactivate gravity_compensation_controller
```

---

## 🔧 配置文件

### 控制器配置

| 文件 | 用途 |
|------|------|
| `eiriarm_controllers/config/ros2_control_controllers.yaml` | 阻抗控制器和重力补偿控制器配置 |
| `eiriarm_bringup/config/controllers.yaml` | 轨迹跟踪控制器配置 |

### URDF 配置

| 文件 | 用途 |
|------|------|
| `dual_arm_support/urdf/dual_arm_robot_plug.urdf` | ros2_control 硬件接口配置 |

---

## 🐛 故障排除

### 问题1：控制器启动失败

**症状**：
```
[spawner]: Failed to activate controller
```

**解决方案**：
1. 确认 MuJoCo 仿真已启动
2. 检查话题是否发布：
   ```bash
   ros2 topic list | grep joint
   ```
3. 查看控制器状态：
   ```bash
   ros2 control list_controllers
   ```

### 问题2：机器人抖动

**原因**：阻抗控制器参数不合适

**解决方案**：
1. 降低阻尼增益（`damping`）
2. 增大速度饱和限制（`velocity_saturation`）
3. 增强速度滤波（`velocity_filter_alpha` 接近 1.0）

参考配置：
```yaml
velocity_filter_alpha: 0.98
velocity_saturation: 8.0
gains:
  right_joint_0:
    damping: 6.0  # 降低此值
```

### 问题3：无法返回初始位置

**原因**：位置误差限制过小

**解决方案**：
增大 `position_error_limit`：
```yaml
position_error_limit: 1.5  # 从 0.5 增加到 1.5
```

### 问题4：力矩不足

**原因**：关节力矩限制

**解决方案**：
检查 URDF 中的关节限制：
```xml
<limit effort="40.0" .../>
```

---

## 📚 相关文档

- **控制器详细说明**：`/src/eiriarm_controllers/README.md`
- **迁移指南**：`/src/eiriarm_controllers/ROS2_CONTROL_MIGRATION_GUIDE.md`
- **快速参考**：`/src/eiriarm_controllers/QUICK_REFERENCE.md`
- **Topic Bridge 指南**：`/src/eiriarm_controllers/TOPIC_BRIDGE_GUIDE.md`

---

## 🎯 示例工作流

### 示例1：阻抗控制示教

```bash
# 1. 启动系统
ros2 launch eiriarm_bringup system.launch.py

# 2. 手动移动机器人到目标位置（在 MuJoCo 中拖动）

# 3. 发送当前位置作为新目标
ros2 topic pub --once /joint_impedance_controller/target_joint_states \
  sensor_msgs/msg/JointState \
  "{name: ['right_joint_0', 'right_joint_1'], position: [0.5, 0.3]}"

# 4. 机器人会保持该位置
```

### 示例2：重力补偿模式

```bash
# 1. 启动重力补偿
ros2 launch eiriarm_bringup system.launch.py controller_type:=gravity_compensation

# 2. 机器人现在可以自由移动，但重力被补偿
# 3. 在 MuJoCo 中手动移动机器人，感受零重力效果
```

### 示例3：轨迹执行

```bash
# 1. 启动轨迹控制器
ros2 launch eiriarm_bringup system.launch.py controller_type:=joint_trajectory

# 2. 发送轨迹
ros2 topic pub /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['right_joint_0', 'right_joint_1', 'right_joint_2'],
    points: [
      {positions: [0.0, 0.0, 0.0], time_from_start: {sec: 0}},
      {positions: [0.5, 0.3, 0.2], time_from_start: {sec: 2}},
      {positions: [0.0, 0.0, 0.0], time_from_start: {sec: 4}}
    ]}"
```

---

## 💡 最佳实践

1. **首次使用**：从重力补偿开始，熟悉系统
2. **参数调整**：先在仿真中测试，再应用到实际机器人
3. **安全第一**：设置合理的力矩限制和误差限制
4. **分离调试**：使用分离启动方式便于查看日志
5. **监控状态**：使用 `ros2 topic echo /joint_states` 监控关节状态

---

## 🔄 版本历史

- **v1.0.0**：初始版本，支持三种控制器类型
- **v1.1.0**：优化阻抗控制器参数，减少抖动
- **v1.2.0**：添加完整系统启动文件

---

## 📧 支持

如有问题，请查看：
1. 日志输出（`--ros-args --log-level debug`）
2. 控制器状态（`ros2 control list_controllers`）
3. 话题列表（`ros2 topic list`）
4. 相关文档（见上方链接）
