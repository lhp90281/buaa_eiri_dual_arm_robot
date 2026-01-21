# 手动测试步骤

## ✅ 插件验证已通过

所有插件都已正确编译和安装：
- ✅ Topic-based Hardware Interface
- ✅ Gravity Compensation Controller
- ✅ Joint Impedance Controller

---

## 🚀 现在开始测试 ros2_control 集成

### 重要提示

在启动 controller_manager 之前，你需要确保 URDF 包含 `<ros2_control>` 标签。

### 选项 1: 快速测试（使用临时 URDF）

```bash
# 1. 创建临时 URDF（包含 ros2_control 配置）
cd ~/ros2_ws
cat src/description/dual_arm_support/urdf/dual_arm_robot.urdf > /tmp/robot_with_ros2_control.urdf

# 2. 添加 ros2_control 标签（在 </robot> 之前）
# 手动编辑或使用以下命令：
sed -i 's|</robot>|<!-- ros2_control configuration -->\n<xacro:include filename="$(find eiriarm_controllers)/config/dual_arm_ros2_control.xacro"/>\n</robot>|' /tmp/robot_with_ros2_control.urdf
```

### 选项 2: 直接修改 URDF（推荐）

编辑 `src/description/dual_arm_support/urdf/dual_arm_robot.urdf`，在 `</robot>` 之前添加：

```xml
<!-- ros2_control configuration -->
<ros2_control name="dual_arm_system" type="system">
  <hardware>
    <plugin>eiriarm_controllers/TopicBasedHardwareInterface</plugin>
    <param name="joint_state_topic">/joint_states</param>
    <param name="effort_command_topic">/ctrl/effort</param>
  </hardware>
  
  <!-- Left arm joints -->
  <joint name="left_joint_0">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="left_joint_1">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="left_joint_2">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="left_joint_3">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="left_joint_4">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="left_joint_5">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="left_joint_6">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  
  <!-- Right arm joints -->
  <joint name="right_joint_0">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_joint_1">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_joint_2">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_joint_3">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_joint_4">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_joint_5">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_joint_6">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

---

## 📋 完整测试流程

### 终端 1: 启动 MuJoCo 仿真

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run eiriarm_mujoco simulate
```

**期望输出**: 仿真器启动，发布 `/joint_states`

### 终端 2: 启动 Controller Manager

```bash
cd ~/ros2_ws
source install/setup.bash

# 使用修改后的 URDF
ros2 run controller_manager ros2_control_node \
  --ros-args \
  --params-file src/eiriarm_controllers/config/ros2_control_controllers.yaml \
  -p robot_description:="$(cat src/description/dual_arm_support/urdf/dual_arm_robot.urdf)"
```

**期望输出**:
```
[INFO] [controller_manager]: update rate is 500 Hz
[INFO] [controller_manager]: Spawning controller_manager RT thread
[INFO] [TopicBasedHardwareInterface]: Configured with 14 joints
[INFO] [TopicBasedHardwareInterface]: Subscribing to: /joint_states
[INFO] [TopicBasedHardwareInterface]: Publishing to: /ctrl/effort
```

**如果失败**: 检查 URDF 是否包含 `<ros2_control>` 标签

### 终端 3: 加载 Joint State Broadcaster

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 control load_controller joint_state_broadcaster
ros2 control set_controller_state joint_state_broadcaster active
```

**期望输出**:
```
Successfully loaded controller joint_state_broadcaster
Successfully configured controller joint_state_broadcaster
Successfully activated controller joint_state_broadcaster
```

### 终端 4: 加载 Impedance Controller

```bash
cd ~/ros2_ws
source install/setup.bash

ros2 control load_controller joint_impedance_controller
ros2 control set_controller_state joint_impedance_controller active
```

**期望输出**:
```
[INFO] [JointImpedanceController]: Configuring with 14 joints
[INFO] [JointImpedanceController]: Dynamics initialized. nq=18, nv=18
Successfully loaded controller joint_impedance_controller
Successfully activated controller joint_impedance_controller
```

### 终端 5: 验证运行状态

```bash
# 查看控制器状态
ros2 control list_controllers

# 期望输出：
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# joint_impedance_controller[eiriarm_controllers/JointImpedanceControllerPlugin] active

# 查看 Topic
ros2 topic list | grep -E "joint_states|ctrl/effort"

# 查看力矩输出频率
ros2 topic hz /ctrl/effort
# 期望: ~500 Hz
```

### 终端 6: 发送测试指令

```bash
# 发送目标位置
ros2 topic pub --once /joint_impedance_controller/target_joint_states \
  sensor_msgs/JointState \
  "{name: ['left_joint_0', 'left_joint_1'], \
    position: [0.5, -0.3], \
    velocity: [0.0, 0.0]}"
```

**观察**: 机械臂应该移动到目标位置

---

## 🔍 故障排查

### 问题 1: Controller Manager 启动失败

**错误**: `No hardware interface found`

**解决**: 检查 URDF 是否包含 `<ros2_control>` 标签
```bash
cat src/description/dual_arm_support/urdf/dual_arm_robot.urdf | grep ros2_control
```

如果没有输出，说明需要添加 ros2_control 配置。

### 问题 2: Hardware Interface 无法连接

**错误**: `No joint states received after 5 seconds`

**解决**: 
1. 确保 MuJoCo 仿真器正在运行
2. 检查 Topic 是否发布：
   ```bash
   ros2 topic echo /joint_states
   ```

### 问题 3: 关节名称不匹配

**错误**: 只有部分关节被控制

**解决**: 确保 URDF 中的关节名称与 MuJoCo 发布的一致
```bash
# 查看 MuJoCo 发布的关节名称
ros2 topic echo /joint_states --once | grep name

# 查看 URDF 中的关节名称
grep '<joint name=' src/description/dual_arm_support/urdf/dual_arm_robot.urdf
```

---

## ✅ 成功标志

当你看到以下输出时，说明集成成功：

```bash
$ ros2 control list_controllers
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
joint_impedance_controller[eiriarm_controllers/JointImpedanceControllerPlugin] active

$ ros2 topic hz /ctrl/effort
average rate: 500.123
	min: 0.002s max: 0.002s std dev: 0.00001s window: 500
```

---

## 📝 下一步

1. ✅ 测试基本功能
2. 调整控制器增益（编辑 `config/ros2_control_controllers.yaml`）
3. 集成到遥操作系统
4. 在真实机器人上测试（使用相同的配置）

---

## 🆘 需要帮助？

- 查看日志: `ros2 topic echo /controller_manager/diagnostics`
- 查看详细指南: `TOPIC_BRIDGE_GUIDE.md`
- 快速参考: `QUICK_REFERENCE.md`
