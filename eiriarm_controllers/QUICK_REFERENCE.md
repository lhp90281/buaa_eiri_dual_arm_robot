# 快速参考卡片

## 编译与安装

```bash
cd ~/ros2_ws
colcon build --packages-select eiriarm_controllers
source install/setup.bash
```

## 验证插件

```bash
# 检查插件是否注册成功
ros2 pkg plugins --package eiriarm_controllers controller_interface

# 期望输出：
# eiriarm_controllers/GravityCompensationController
# eiriarm_controllers/JointImpedanceControllerPlugin
```

## 启动控制器

### 方法 1: 使用 Controller Manager 命令行

```bash
# 1. 启动 controller_manager (需要先启动你的硬件接口)
ros2 run controller_manager ros2_control_node \
  --ros-args --params-file src/eiriarm_controllers/config/ros2_control_controllers.yaml

# 2. 列出所有控制器
ros2 control list_controllers

# 3. 加载控制器
ros2 control load_controller joint_impedance_controller

# 4. 激活控制器
ros2 control set_controller_state joint_impedance_controller active

# 5. 查看控制器状态
ros2 control list_controllers
```

### 方法 2: 使用 Spawner

```bash
# 在一个终端启动 controller_manager
ros2 run controller_manager ros2_control_node \
  --ros-args --params-file config/ros2_control_controllers.yaml

# 在另一个终端加载并激活控制器
ros2 run controller_manager spawner joint_impedance_controller
```

## 发送目标指令

### 使用命令行

```bash
# 发送静态目标位置
ros2 topic pub --once /joint_impedance_controller/target_joint_states \
  sensor_msgs/JointState \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, \
    name: ['left_joint_0', 'left_joint_1'], \
    position: [0.5, -0.3], \
    velocity: [0.0, 0.0]}"
```

### 使用 Python 脚本

```bash
# 使用提供的示例脚本
cd ~/ros2_ws/src/eiriarm_controllers/scripts
chmod +x send_target_command.py
./send_target_command.py
```

或在 Python 代码中：

```python
import rclpy
from sensor_msgs.msg import JointState

# ... 在你的节点中 ...
publisher = self.create_publisher(
    JointState, 
    '/joint_impedance_controller/target_joint_states', 
    10
)

msg = JointState()
msg.header.stamp = self.get_clock().now().to_msg()
msg.name = ['left_joint_0', 'left_joint_1']
msg.position = [0.5, -0.3]
msg.velocity = [0.0, 0.0]
publisher.publish(msg)
```

## 监控控制器

```bash
# 查看反馈力矩 (调试用)
ros2 topic echo /joint_impedance_controller/effort_feedback

# 查看关节状态
ros2 topic echo /joint_states

# 查看控制器参数
ros2 param list /joint_impedance_controller
ros2 param get /joint_impedance_controller default_stiffness
```

## 动态调整参数

```bash
# 调整刚度
ros2 param set /joint_impedance_controller gains.left_joint_0.stiffness 25.0

# 调整阻尼
ros2 param set /joint_impedance_controller gains.left_joint_0.damping 6.0

# 注意：参数更新可能需要控制器支持动态重配置
```

## 切换控制器

```bash
# 停用当前控制器
ros2 control set_controller_state joint_impedance_controller inactive

# 激活另一个控制器
ros2 control set_controller_state gravity_compensation_controller active

# 或者使用 switch 命令（原子切换）
ros2 control switch_controllers \
  --activate gravity_compensation_controller \
  --deactivate joint_impedance_controller
```

## 卸载控制器

```bash
# 先停用
ros2 control set_controller_state joint_impedance_controller inactive

# 再卸载
ros2 control unload_controller joint_impedance_controller
```

## 诊断与调试

```bash
# 检查 controller_manager 日志
ros2 topic echo /controller_manager/diagnostics

# 查看控制器节点日志级别
ros2 run rqt_console rqt_console

# 设置日志级别为 DEBUG
ros2 param set /controller_manager log_level DEBUG
```

## 配置文件位置

- **控制器配置**: `config/ros2_control_controllers.yaml`
- **插件描述**: `eiriarm_controllers.xml`
- **启动文件示例**: `launch/ros2_control_example.launch.py`

## 常见问题速查

| 问题 | 解决方案 |
|------|----------|
| 插件无法加载 | 检查 `ros2 pkg plugins` 输出，重新编译并 source |
| 无法获取 robot_description | 确保在启动 controller_manager 时传入了 robot_description 参数 |
| Interface 数量不匹配 | 检查 URDF 中 `<ros2_control>` 标签是否正确定义 |
| 控制器无响应 | 检查目标指令的 topic 名称和关节名称是否正确 |
| 力矩饱和 | 降低刚度/阻尼增益，或增加 `max_effort` 限制 |

## 性能调优

| 参数 | 建议值 | 说明 |
|------|--------|------|
| `update_rate` | 500 Hz | 控制循环频率 |
| `default_stiffness` | 10-30 N·m/rad | 根据机器人惯量调整 |
| `default_damping` | 3-8 N·m·s/rad | 通常为刚度的 20-30% |
| `velocity_filter_alpha` | 0.95-0.99 | 速度滤波系数，越大越平滑 |
| `position_error_limit` | 0.087 rad | ~5°，防止突变 |

## 下一步

1. 阅读 [`ROS2_CONTROL_MIGRATION_GUIDE.md`](ROS2_CONTROL_MIGRATION_GUIDE.md) 了解详细实现
2. 为你的硬件创建 Hardware Interface 插件
3. 根据实际机器人调整增益参数
4. 集成到你的遥操作系统

## 有用的链接

- [ros2_control 文档](https://control.ros.org/)
- [Pinocchio API](https://stack-of-tasks.github.io/pinocchio/)
- [realtime_tools](https://github.com/ros-controls/realtime_tools)
