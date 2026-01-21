# ROS 2 Control Migration Guide

本指南说明如何从基于 Topic 的控制器迁移到 ros2_control 插件架构。

## 架构对比

### AS-IS (旧架构 - 基于 Topic)

```
仿真环境 (MuJoCo) --/joint_states--> 控制器节点 --/ctrl/effort--> 仿真环境
                                         ^
                                         |
                               /impedance/target_joint_states
```

**问题**：
- Topic 通信存在延迟（~1-2ms）
- 无法保证实时性
- 控制频率受限于网络抖动
- 难以达到 500Hz 以上的控制频率

### TO-BE (新架构 - ros2_control)

```
Hardware Interface <--> Controller Manager <--> Controllers
                            (实时循环)            (插件)
                                                   ^
                                                   |
                                         /target_joint_states
                                      (通过 RealtimeBuffer)
```

**优势**：
- 零拷贝、共享内存通信
- 保证实时性（500Hz+）
- 标准化接口，易于切换硬件
- 支持插件化控制器

---

## 新架构说明

### 1. 独立算法库：`eiriarm_dynamics`

**文件**：
- `include/eiriarm_controllers/eiriarm_dynamics.hpp`
- `src/eiriarm_dynamics.cpp`

**功能**：
- 纯 C++ 算法类，封装所有 Pinocchio 计算
- 与 ROS 解耦，可单独测试
- 提供接口：
  - `initFromURDF()` - 从 URDF 初始化模型
  - `updateState()` - 更新关节状态
  - `computeGravity()` - 计算重力补偿力矩
  - `computeImpedance()` - 计算阻抗控制力矩

**为什么要分离**？
- ✅ 提高可测试性（可以编写单元测试）
- ✅ 提高可复用性（其他包也可以使用）
- ✅ 清晰的职责划分（算法 vs 接口）

### 2. ros2_control 插件

#### A. `GravityCompensationController`

**文件**：
- `include/eiriarm_controllers/gravity_compensation_controller.hpp`
- `src/gravity_compensation_controller.cpp`

**功能**：
- 读取关节位置/速度（`state_interfaces_`）
- 计算重力补偿力矩 `g(q)`
- 写入力矩指令（`command_interfaces_`）

**配置参数**：
```yaml
gravity_compensation_controller:
  ros__parameters:
    joints: [joint1, joint2, ...]
    gravity_gain: 1.0  # 重力补偿增益
```

#### B. `JointImpedanceControllerPlugin`

**文件**：
- `include/eiriarm_controllers/joint_impedance_controller_plugin.hpp`
- `src/joint_impedance_controller_plugin.cpp`

**功能**：
- 实现阻抗控制：`tau = g(q) + Kp*(qd-q) + Kd*(vd-v)`
- 支持每个关节独立增益
- 支持实时目标指令更新（通过 `realtime_tools::RealtimeBuffer`）

**配置参数**：
```yaml
joint_impedance_controller:
  ros__parameters:
    joints: [joint1, joint2, ...]
    default_stiffness: 20.0
    default_damping: 5.0
    max_effort: 50.0
    position_error_limit: 0.087  # ~5度
    velocity_filter_alpha: 0.99
    velocity_saturation: 5.0
    
    # 每个关节的独立增益
    gains:
      joint1:
        stiffness: 15.0
        damping: 4.0
```

**Topic 接口**：
- 订阅：`~/target_joint_states` (sensor_msgs/JointState) - 目标位置/速度
- 发布：`~/effort_feedback` (sensor_msgs/JointState) - 反馈力矩（调试用）

---

## 如何使用

### 步骤 1: 编译

```bash
cd ~/ros2_ws
colcon build --packages-select eiriarm_controllers
source install/setup.bash
```

### 步骤 2: 验证插件是否注册

```bash
ros2 pkg plugins --package eiriarm_controllers controller_interface
```

**期望输出**：
```
eiriarm_controllers/GravityCompensationController (controller_interface::ControllerInterface)
eiriarm_controllers/JointImpedanceControllerPlugin (controller_interface::ControllerInterface)
```

### 步骤 3: 配置 ros2_control

你需要创建一个 `ros2_control` URDF 标签来定义硬件接口。示例：

```xml
<ros2_control name="dual_arm_system" type="system">
  <hardware>
    <plugin>your_hardware_interface/YourHardwareInterface</plugin>
    <!-- 或者用 mock 组件进行测试 -->
    <!-- <plugin>mock_components/GenericSystem</plugin> -->
  </hardware>
  
  <joint name="left_joint_0">
    <command_interface name="effort"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  
  <!-- 重复所有关节 ... -->
</ros2_control>
```

### 步骤 4: 启动控制器

#### 方法 A: 使用 Controller Manager 命令行

```bash
# 启动 controller manager
ros2 run controller_manager ros2_control_node \
  --ros-args --params-file config/ros2_control_controllers.yaml

# 加载并激活控制器
ros2 control load_controller joint_impedance_controller
ros2 control set_controller_state joint_impedance_controller active
```

#### 方法 B: 使用 Launch 文件

```bash
ros2 launch eiriarm_controllers ros2_control_example.launch.py
```

### 步骤 5: 发送目标指令

阻抗控制器订阅 `~/target_joint_states`：

```bash
ros2 topic pub /joint_impedance_controller/target_joint_states sensor_msgs/JointState \
  "{name: ['left_joint_0', 'left_joint_1'], \
    position: [0.5, -0.3], \
    velocity: [0.0, 0.0]}"
```

或者使用 Python：

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.pub = self.create_publisher(
            JointState, 
            '/joint_impedance_controller/target_joint_states', 
            10
        )
        
    def send_target(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_joint_0', 'left_joint_1', ...]
        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        self.pub.publish(msg)
```

---

## 关键实现细节

### 1. 生命周期管理

Controllers 遵循 ROS 2 Lifecycle 节点模式：

```
[Unconfigured] --on_configure--> [Inactive] --on_activate--> [Active]
                                                                  |
                                    [Inactive] <--on_deactivate---+
```

- **`on_configure()`**: 加载参数、初始化 URDF、创建 `eiriarm_dynamics` 对象
- **`on_activate()`**: 获取 `state_interfaces_` 和 `command_interfaces_`
- **`update()`**: 实时控制循环（由 Controller Manager 调用）
- **`on_deactivate()`**: 释放资源、清零输出

### 2. URDF 加载

**重要**: 不要从文件路径加载 URDF！

```cpp
// ❌ 错误做法
std::string urdf_path = "/path/to/robot.urdf";
pinocchio::urdf::buildModel(urdf_path, model_);

// ✅ 正确做法
std::string robot_description;
get_node()->get_parameter("robot_description", robot_description);
dynamics_->initFromURDF(robot_description);
```

`robot_description` 参数由 `robot_state_publisher` 或 launch file 提供。

### 3. 实时安全的 Topic 订阅

对于外部指令（如 `/target_joint_states`），必须使用 `realtime_tools::RealtimeBuffer`：

```cpp
// 声明
realtime_tools::RealtimeBuffer<sensor_msgs::msg::JointState::SharedPtr> rt_target_buffer_;

// 在回调中写入（非实时线程）
void targetCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  rt_target_buffer_.writeFromNonRT(msg);  // 无锁写入
}

// 在 update() 中读取（实时线程）
auto target_msg = rt_target_buffer_.readFromRT();  // 无锁读取
if (target_msg && (*target_msg)) {
  // 使用数据
}
```

这保证实时线程永远不会阻塞。

### 4. State/Command Interfaces

**State Interfaces**（只读）：
- `state_interfaces_[2*i]` → `joint_i/position`
- `state_interfaces_[2*i+1]` → `joint_i/velocity`

**Command Interfaces**（写入）：
- `command_interfaces_[i]` → `joint_i/effort`

访问：
```cpp
// 读取
double position = state_interfaces_[2*i].get_value();
double velocity = state_interfaces_[2*i+1].get_value();

// 写入
command_interfaces_[i].set_value(torque);
```

---

## 性能优化建议

1. **控制频率**: 推荐 500Hz（在 `controller_manager` 配置中设置 `update_rate`）

2. **速度滤波**: 使用低通滤波器平滑速度信号
   ```
   v_filtered = (1 - alpha) * v_filtered + alpha * v_raw
   alpha = 0.99  # 重滤波
   ```

3. **错误限幅**: 限制位置误差，防止力矩突变
   ```
   error = clamp(qd - q, -0.087, 0.087)  # ±5度
   ```

4. **力矩限幅**: 使用 URDF 中的 `effort_limit`
   ```xml
   <limit effort="50.0" velocity="2.0"/>
   ```

---

## 故障排查

### 问题 1: 插件无法加载

**错误信息**：
```
Could not find requested resource 'eiriarm_controllers/JointImpedanceControllerPlugin'
```

**解决方案**：
1. 检查编译是否成功：`colcon build --packages-select eiriarm_controllers`
2. 重新 source：`source install/setup.bash`
3. 验证插件注册：`ros2 pkg plugins --package eiriarm_controllers controller_interface`
4. 检查 `eiriarm_controllers.xml` 是否正确安装到 `share/eiriarm_controllers/`

### 问题 2: 无法获取 robot_description

**错误信息**：
```
Failed to get 'robot_description' parameter!
```

**解决方案**：
确保在启动 controller_manager 时加载了 robot_description：

```python
# 在 launch 文件中
robot_description_content = Command([...])
robot_description = {'robot_description': robot_description_content}

controller_manager_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[robot_description, controller_config],
)
```

### 问题 3: State/Command Interface 数量不匹配

**错误信息**：
```
Expected 24 state interfaces, got 0
```

**解决方案**：
检查 URDF 中的 `<ros2_control>` 标签是否正确定义了所有关节的接口：
- 每个关节需要 `<state_interface name="position"/>`
- 每个关节需要 `<state_interface name="velocity"/>`
- 每个关节需要 `<command_interface name="effort"/>`

---

## 与 MuJoCo 仿真环境集成

如果你使用 `eiriarm_mujoco` 作为仿真环境，你需要创建一个 Hardware Interface 插件来桥接 MuJoCo 和 ros2_control。

示例伪代码：

```cpp
class MuJoCoHardwareInterface : public hardware_interface::SystemInterface {
  // 在 read() 中从 MuJoCo 读取关节状态
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override {
    for (size_t i = 0; i < num_joints_; ++i) {
      joint_positions_[i] = mujoco_data_->qpos[i];
      joint_velocities_[i] = mujoco_data_->qvel[i];
    }
    return hardware_interface::return_type::OK;
  }
  
  // 在 write() 中将控制指令写入 MuJoCo
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override {
    for (size_t i = 0; i < num_joints_; ++i) {
      mujoco_data_->ctrl[i] = joint_efforts_[i];
    }
    return hardware_interface::return_type::OK;
  }
};
```

---

## 进一步阅读

- [ros2_control 官方文档](https://control.ros.org/)
- [controller_interface API](https://control.ros.org/master/doc/api/controller_interface.html)
- [realtime_tools](https://github.com/ros-controls/realtime_tools)
- [Pinocchio 文档](https://stack-of-tasks.github.io/pinocchio/)

---

## 总结

重构完成后，你的控制架构将：
- ✅ 支持高频控制（500Hz+）
- ✅ 实时安全，无阻塞
- ✅ 代码结构清晰（算法库 + 插件）
- ✅ 易于切换硬件（只需更换 Hardware Interface）
- ✅ 符合 ROS 2 生态标准

**下一步**：
1. 为你的 MuJoCo 仿真创建 Hardware Interface
2. 在真实机器人上测试控制器
3. 根据性能调整增益参数
