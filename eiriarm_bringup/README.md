# EiriArm Bringup 使用手册

`eiriarm_bringup` 是 EiriArm 双臂系统的主入口，负责把 USB-CAN 桥、`ros2_control`、控制器、夹爪、MuJoCo 仿真/真机面板、回零、示教录制和回放串起来。

这个工作区当前提供的是底层控制与工具链，不提供 MoveIt 风格的全局规划。笛卡尔控制器只做当前位姿附近的逆解和关节插值，不做避障、不做路径搜索、不做碰撞检测。需要规划时应在外部规划器中生成轨迹，再通过这里的关节位置接口执行。

---

## 目录

- [1. 编译与环境](#1-编译与环境)
- [2. 真机启动](#2-真机启动)
- [3. 控制器模式](#3-控制器模式)
- [4. 关节位置控制](#4-关节位置控制)
- [5. 笛卡尔逆解控制](#5-笛卡尔逆解控制)
- [6. 回零](#6-回零)
- [7. 示教录制与回放](#7-示教录制与回放)
- [8. MuJoCo 仿真](#8-mujoco-仿真)
- [9. MuJoCo 真机显示/编辑面板](#9-mujoco-真机显示编辑面板)
- [10. 双机遥操作](#10-双机遥操作)
- [11. 夹爪](#11-夹爪)
- [12. 关节零点标定](#12-关节零点标定)
- [13. 摩擦辨识与重力补偿调参](#13-摩擦辨识与重力补偿调参不建议使用需要拆除电机独立标定不能整臂标定)
- [14. 主要文件](#14-主要文件)
- [15. 故障排查](#15-故障排查)

---

## 1. 编译与环境

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

常用开发时可以只编译相关包：

```bash
colcon build --packages-select usb2can eiriarm_controllers eiriarm_mujoco eiriarm_bringup
source install/setup.bash
```

如果启动后提示找不到新 launch 或新可执行文件，通常是忘了重新 `source install/setup.bash`。

---

## 2. 真机启动

真机启动分两个终端。这样 CAN 串口日志和控制器日志不会互相淹没，排查问题更清楚。

### 终端 1：启动 USB-CAN 桥

```bash
source install/setup.bash
ros2 launch eiriarm_bringup bridge.launch.py
```

自定义串口：

```bash
ros2 launch eiriarm_bringup bridge.launch.py device:=/dev/ttyACM1 # 仅在需要时设置，通常无需设置
```

`bridge.launch.py` 启动：

- `usb2can_node`：USB 串口到 `/dcu/command`、`/dcu/feedback`、`/imu/data`
- `dm_motor_bridge`：DM 电机 MIT 协议聚合，提供 `/motor/chN/cmd`、`/motor/chN/state`、`/motor/chN/motor_enable`

默认电机配置：

```bash
src/USB2CAN/usb2can/config/dm_motors_eiriarm.yaml
```

### 终端 2：启动真机控制端

日常强烈建议启动 MuJoCo GUI 面板。它默认是 mirror 模式，可以实时看到真机姿态，也可以用快捷键切控制器、回零、进入 slider 编辑并下发目标，调试时比纯命令行安全直观。

推荐启动：

```bash
source install/setup.bash
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true
```

最小无 GUI 启动：

```bash
source install/setup.bash
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=false
```

默认行为：

- 双臂 `arms:=dual`
- 启动 `robot_state_publisher`
- 启动 `controller_manager`
- 激活 `joint_state_broadcaster`
- 激活 `gravity_compensation_controller`
- 加载但不激活 `joint_position_controller`
- 加载但不激活 `cartesian_position_controller`
- 启动并自动标定双夹爪
- 默认不启动 MuJoCo GUI；建议日常显式加 `use_gui:=true`

常用参数：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `arms` | `dual` | `left`、`right`、`dual` |
| `controller` | `gravity` | `gravity`、`joint_position`、`cartesian_position` |
| `gripper` | `true` | 是否启动夹爪控制节点 |
| `offsets_yaml` | `joint_offsets_dual.yaml` | 关节零位和方向标定；相对路径按启动 launch 的当前目录解析 |
| `friction_model_yaml` | `friction_model.yaml` | 摩擦模型；相对路径按启动 launch 的当前目录解析 |
| `teleop_role` | `slave` | 关节位置控制增益 profile；`slave` 保持当前较硬跟踪增益，`master` 使用更软的力反馈主臂增益 |
| `teleop_gains_yaml` | 空 | 自定义 master/slave `kp_gains`、`kd_gains` profile；空值使用包内默认 |
| `use_gui` | `false` | 是否同时启动 MuJoCo 真机面板 |

示例：

```bash
# 推荐：重力补偿示教模式 + MuJoCo 真机面板
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true

# 最小启动：重力补偿示教模式，无 GUI
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=false

# 启动时直接进入关节位置控制
ros2 launch eiriarm_bringup real_robot.launch.py controller:=joint_position

# 启动时直接进入笛卡尔逆解控制
ros2 launch eiriarm_bringup real_robot.launch.py controller:=cartesian_position

# 只启动左臂，不启动夹爪
ros2 launch eiriarm_bringup real_robot.launch.py arms:=left gripper:=false

# 启动重力补偿，并打开 MuJoCo 真机显示/控制面板
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true

# 力反馈主臂：加载更软的 joint_position_controller 增益
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true teleop_role:=master
```

关机建议顺序：

1. 先在控制端终端按 `Ctrl+C`
2. 等待控制器和夹爪发送 disable
3. 再在 bridge 终端按 `Ctrl+C`

---

## 3. 控制器模式

当前主流程使用三种模式：

| 模式 | 激活控制器 | 用途 |
|---|---|---|
| `gravity` | `gravity_compensation_controller` | 默认示教、手拖、录制 |
| `joint_position` | `gravity_compensation_controller` + `joint_position_controller` | 关节轨迹执行、回零、重放、MuJoCo slider 下发 |
| `cartesian_position` | `gravity_compensation_controller` + `cartesian_position_controller` | 双臂笛卡尔目标逆解控制 |

重力补偿控制器总是作为底层补偿运行。关节位置控制器和笛卡尔控制器不直接做力矩动力学补偿，而是通过 DM MIT 五参数接口输出位置、速度、`kp`、`kd` 和前馈力矩，和真机接口保持一致。

手动查看控制器：

```bash
ros2 control list_controllers
```

手动切到关节位置控制：

```bash
ros2 control switch_controllers \
  --activate gravity_compensation_controller joint_position_controller \
  --deactivate cartesian_position_controller
```

手动切回重力补偿：

```bash
ros2 control switch_controllers \
  --activate gravity_compensation_controller \
  --deactivate joint_position_controller cartesian_position_controller
```

一般不需要手动切控制器，`go_home.launch.py`、`replay.launch.py` 和 MuJoCo 面板会自动切。

---

## 4. 关节位置控制

关节位置控制器订阅：

```bash
/joint_position_command
```

消息类型：

```bash
trajectory_msgs/msg/JointTrajectory
```

发送一个简单目标：

```bash
ros2 topic pub --once /joint_position_command trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    left_joint_0, left_joint_1, left_joint_2, left_joint_3,
    left_joint_4, left_joint_5, left_joint_6,
    right_joint_0, right_joint_1, right_joint_2, right_joint_3,
    right_joint_4, right_joint_5, right_joint_6
  ],
  points: [{
    positions: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    velocities: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    time_from_start: {sec: 5, nanosec: 0}
  }]
}"
```

控制参数在：

```bash
src/eiriarm_controllers/config/dual_arm_controllers.yaml
```

重点参数：

- `joint_position_controller.kp_gains`
- `joint_position_controller.kd_gains`
- `gravity_compensation_controller.gravity_gains`
- `gravity_compensation_controller.friction_gains`

仿真使用单独配置：

```bash
src/eiriarm_controllers/config/dual_arm_sim_controllers.yaml
```

仿真里的重力和摩擦系数不要直接照搬真机经验值。仿真配置默认 `gravity_gains=1.0`，并关闭真机摩擦前馈。

---

## 5. 笛卡尔逆解控制

笛卡尔控制器只支持双臂：

```bash
ros2 launch eiriarm_bringup real_robot.launch.py controller:=cartesian_position
```

控制器订阅：

```bash
/cartesian_position_controller/left_target_pose
/cartesian_position_controller/right_target_pose
/cartesian_position_controller/joint_homing
```

消息类型：

- 左右目标：`geometry_msgs/msg/PoseStamped`
- homing：`std_msgs/msg/String`，值为 `left`、`right` 或 `both`

它发布：

```bash
/cartesian_position_controller/left_cartesian_state
/cartesian_position_controller/right_cartesian_state
/cartesian_position_controller/feedback
/cartesian_position_controller/homing_status
```

键盘控制：

```bash
ros2 run eiriarm_controllers cartesian_keyboard_control
```

注意：

- 这里没有路径规划。
- 每次收到目标位姿后，控制器用当前关节附近的 IK 求解目标关节位姿。
- 求出目标后做关节空间插值，速度由 `position_interpolation_speed` 限制。
- 不做碰撞检测，不绕障，不保证远距离目标可达。

IK 参数在：

```bash
src/eiriarm_controllers/config/dual_arm_controllers.yaml
src/eiriarm_controllers/config/cartesian_position_controller.yaml
```

常用参数：

- `left_ee_frame`
- `right_ee_frame`
- `ik_max_iter`
- `ik_eps`
- `ik_damping`
- `ik_dt`
- `position_interpolation_speed`

---

## 6. 回零

回零通过关节位置控制器执行，会自动切到 `joint_position_controller`，发布全零关节目标，然后默认切回重力补偿。

```bash
source install/setup.bash
ros2 launch eiriarm_bringup go_home.launch.py
```

慢速回零：

```bash
ros2 launch eiriarm_bringup go_home.launch.py duration:=8.0
```

回零后保持关节位置控制，方便马上接回放：

```bash
ros2 launch eiriarm_bringup go_home.launch.py duration:=8.0 restore_gravity:=false
```

只回左臂：

```bash
ros2 launch eiriarm_bringup go_home.launch.py \
  joints:='left_joint_0 left_joint_1 left_joint_2 left_joint_3 left_joint_4 left_joint_5 left_joint_6'
```

参数：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `duration` | `5.0` | 插值到零位的时间 |
| `restore_gravity` | `true` | 完成后是否切回重力补偿 |
| `joints` | 空 | 空表示读取控制器中的关节列表 |
| `command_topic` | `/joint_position_command` | 轨迹命令话题 |

---

## 7. 示教录制与回放

录制和回放使用 `teach_replay`，bringup 里已经封装成 launch。

### 录制

录制前建议保持默认重力补偿模式，不要让关节位置控制器 active，否则 PD 会和手拖对抗。

```bash
ros2 launch eiriarm_bringup record.launch.py output:=recordings/demo.yaml
```

录制时拖动机械臂，按 `q` 或 `Ctrl+C` 停止并写文件。

只录左臂：

```bash
ros2 launch eiriarm_bringup record.launch.py \
  output:=recordings/left_demo.yaml \
  joints:='left_joint_0 left_joint_1 left_joint_2 left_joint_3 left_joint_4 left_joint_5 left_joint_6'
```

参数：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `output` | 空 | 必填，输出 YAML |
| `rate` | `50.0` | 采样频率 |
| `joints` | 空 | 空表示录 `/joint_states` 中全部关节 |
| `max_duration` | `120.0` | 最长录制时间 |

### 回放

```bash
ros2 launch eiriarm_bringup replay.launch.py input:=recordings/demo.yaml
```

半速回放：

```bash
ros2 launch eiriarm_bringup replay.launch.py \
  input:=recordings/demo.yaml \
  time_scale:=0.5
```

录制起点离当前姿态较远时，增加进入段：

```bash
ros2 launch eiriarm_bringup replay.launch.py \
  input:=recordings/demo.yaml \
  ramp_in:=4.0
```

参数：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `input` | 空 | 必填，录制 YAML |
| `time_scale` | `1.0` | 回放速度，`0.5` 为半速 |
| `ramp_in` | `2.0` | 从当前姿态插值到录制起点的时间 |
| `publish_rate` | `50.0` | 下发 setpoint 频率 |
| `command_topic` | `/joint_position_command` | 轨迹命令话题 |
| `auto_switch` | `true` | 自动切控制器 |

典型流程：

```bash
# 终端 1
ros2 launch eiriarm_bringup bridge.launch.py

# 终端 2
ros2 launch eiriarm_bringup real_robot.launch.py

# 终端 3
ros2 launch eiriarm_bringup record.launch.py output:=recordings/demo.yaml
ros2 launch eiriarm_bringup go_home.launch.py
ros2 launch eiriarm_bringup replay.launch.py input:=recordings/demo.yaml
```

---

## 8. MuJoCo 仿真

完整仿真启动：

```bash
source install/setup.bash
ros2 launch eiriarm_bringup system.launch.py
```

默认启动：

- MuJoCo 双臂模型
- 仿真用 topic-based hardware
- `joint_state_broadcaster`
- `gravity_compensation_controller`
- `joint_position_controller`

选择控制器：

```bash
# 只重力补偿
ros2 launch eiriarm_bringup system.launch.py controller_type:=gravity_compensation

# 关节位置控制
ros2 launch eiriarm_bringup system.launch.py controller_type:=joint_position

# 笛卡尔逆解控制
ros2 launch eiriarm_bringup system.launch.py controller_type:=cartesian_position
```

只启动 MuJoCo：

```bash
ros2 launch eiriarm_bringup mujoco_sim.launch.py
```

只启动仿真控制器，连接已有 MuJoCo：

```bash
ros2 launch eiriarm_bringup controllers.launch.py hardware:=sim controller_type:=joint_position
```

仿真配置：

```bash
src/eiriarm_mujoco/config/simulate.yaml
src/eiriarm_controllers/config/dual_arm_sim_controllers.yaml
src/eiriarm_controllers/config/dual_arm_sim_ros2_control.urdf.xacro
```

仿真和真机接口已经对齐为 MIT 五参数形式：

- `/ctrl/command`：`name`、`position(q_des)`、`velocity(qd_des)`、`effort(torque_ff)`
- `/ctrl/gains`：`name`、`position(kp)`、`velocity(kd)`

这个仿真主要用于验证控制接口、轨迹、录制回放和 GUI 操作流程，不用于动力学精确训练。

---

## 9. MuJoCo 真机显示/编辑面板

真机控制端可以同时启动 MuJoCo 面板：

```bash
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true
```

也可以单独启动面板：

```bash
ros2 launch eiriarm_bringup mujoco_panel.launch.py
```

默认模式是 `mirror_real`：

- 订阅 `/joint_states`
- MuJoCo 只显示真机姿态
- 不运行物理仿真
- 不发布 `/joint_states`

快捷键：

| 键 | 作用 |
|---|---|
| `J` | 切到 `joint_position_controller` |
| `G` | 切回重力补偿 |
| `K` | 切到 `cartesian_position_controller` |
| `H` | 切到关节位置控制并发布全零回零轨迹 |
| `E` | 切到关节位置控制，冻结当前姿态，进入 slider 编辑 |
| `S` | 下发当前 MuJoCo slider 姿态到 `/joint_position_command` |
| `C` | 取消编辑，回到 mirror |

目标编辑流程：

1. 启动真机控制端并打开 GUI
2. 按 `E`
3. 用 MuJoCo slider 调整关节角
4. 按 `S`，真机按 `trajectory_duration` 插值过去
5. 按 `C` 可取消编辑

服务接口：

```bash
ros2 service call /mujoco_panel/start_edit std_srvs/srv/Trigger {}
ros2 service call /mujoco_panel/send_target std_srvs/srv/Trigger {}
ros2 service call /mujoco_panel/cancel_edit std_srvs/srv/Trigger {}
```

调整下发插值时间：

```bash
ros2 launch eiriarm_bringup mujoco_panel.launch.py trajectory_duration:=8.0
```

---

## 10. 双机遥操作

遥操作使用两套机械臂和两台主机。两台主机的 ROS 2 图不需要互通，建议使用不同 `ROS_DOMAIN_ID`，避免 `/joint_states`、`/controller_manager` 等本地话题撞名。遥操作本身只通过 UDP 交换关节角度。

第一版只做双臂全关节关节角跟随，不做规划、不做避障、不做力传感器闭环。

### 启动拓扑

每台机器仍按真机流程先启动本机硬件：

```bash
# 每台机器终端 1
source install/setup.bash
ros2 launch eiriarm_bringup bridge.launch.py

# 每台机器终端 2，推荐打开 GUI
source install/setup.bash
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true
```

然后每台机器再启动一个遥操作节点。假设：

- 主臂主机 IP：`192.168.10.10`
- 从臂主机 IP：`192.168.10.20`
- 主臂接收端口：`15000`
- 从臂接收端口：`15001`

主臂主机：

```bash
source install/setup.bash
ros2 launch eiriarm_bringup teleop.launch.py \
  role:=master \
  mode:=no_feedback \
  peer_host:=192.168.10.20 \
  local_port:=15000 \
  peer_port:=15001
```

从臂主机：

```bash
source install/setup.bash
ros2 launch eiriarm_bringup teleop.launch.py \
  role:=slave \
  mode:=no_feedback \
  peer_host:=192.168.10.10 \
  local_port:=15001 \
  peer_port:=15000
```

如果两边使用同一个端口也可以，例如都用 `local_port:=15000 peer_port:=15000`，只要防火墙允许 UDP 端口互通。

### 无力反馈模式

`mode:=no_feedback`：

- 主臂对齐阶段临时切到 `joint_position_controller`
- 对齐完成后主臂切回 `gravity_compensation_controller`
- 主臂只发送当前关节角度
- 从臂切到 `joint_position_controller` 并跟踪主臂角度

启动两个 teleop 节点后，在主臂主机执行：

```bash
ros2 service call /teleop/align std_srvs/srv/Trigger {}
```

等待返回成功后，两台机器都执行 enable：

```bash
ros2 service call /teleop/enable std_srvs/srv/Trigger {}
```

停止遥操作，任意一台机器执行即可，另一台会通过 UDP 状态自动停止：

```bash
ros2 service call /teleop/disable std_srvs/srv/Trigger {}
```

### 力反馈模式

`mode:=force_feedback`：

- 主臂和从臂都使用 `joint_position_controller`
- 两边互相把对端关节角作为本机目标
- 这是位置耦合式力反馈，不是基于力矩传感器的真实力反馈

启动命令只需要把两边的 `mode` 改成 `force_feedback`。流程仍然是：

力反馈建议主臂控制端加载软增益，从臂保持默认硬增益。这个参数在 `real_robot.launch.py` 启动控制器时生效，修改后需要重启控制端：

```bash
# 主臂控制端
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true teleop_role:=master

# 从臂控制端，teleop_role 默认就是 slave，也可以显式写出
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true teleop_role:=slave
```

然后两边 teleop 节点都使用 `mode:=force_feedback`，再执行：

```bash
# 主臂主机
ros2 service call /teleop/align std_srvs/srv/Trigger {}

# 两台机器
ros2 service call /teleop/enable std_srvs/srv/Trigger {}
```

默认增益文件在：

```bash
src/eiriarm_controllers/config/teleop_joint_gains.yaml
```

力反馈模式首次测试建议从空载、低速、小范围开始。默认只给主臂加死区，从臂不加死区并完整跟随，避免从臂小幅跟踪时一卡一卡。若主臂仍然顶手，优先调大 `force_deadband_master` 或调低 `master_coupling_scale`；若反馈太弱，再反向调整。若发抖，优先小幅增加 `master.kd_gains`、降低 `max_step`，或略增大死区。

### 安全参数

| 参数 | 默认 | 说明 |
|---|---:|---|
| `rate_hz` | `50.0` | UDP 状态交换和目标下发频率 |
| `timeout` | `0.3` | 超过该时间没收到对端 UDP 包会自动 disable |
| `align_duration` | `5.0` | 主臂对齐从臂的插值时间 |
| `max_start_error` | `0.5` | enable 前两边最大关节误差限制，单位 rad |
| `max_runtime_error` | `1.0` | 运行中最大关节误差限制，超过自动 disable |
| `max_step` | `0.03` | 每个周期目标最大变化量，单位 rad |
| `force_deadband_master` | `0.015` | 力反馈主臂软死区，单位 rad；小误差不反馈 |
| `force_deadband_slave` | `0.0` | 力反馈从臂软死区，单位 rad；默认无死区，保证连续跟随 |
| `force_deadband_hysteresis` | `0.004` | 死区迟滞，防止阈值附近反复开关 |
| `force_deadband_master_joints` | 空 | 可选 14 关节主臂死区列表，逗号或空格分隔；非空时覆盖 `force_deadband_master` |
| `force_deadband_slave_joints` | 空 | 可选 14 关节从臂死区列表，逗号或空格分隔；默认空 |
| `master_coupling_scale` | `0.45` | 主臂反馈比例，越小越丝滑、反馈越轻 |
| `slave_coupling_scale` | `1.0` | 从臂跟随比例，默认完整跟随 |

示例：

```bash
ros2 launch eiriarm_bringup teleop.launch.py \
  role:=slave \
  mode:=no_feedback \
  peer_host:=192.168.10.10 \
  local_port:=15001 \
  peer_port:=15000 \
  rate_hz:=30 \
  max_step:=0.015
```

力反馈调手感示例：

```bash
ros2 launch eiriarm_bringup teleop.launch.py \
  role:=master \
  mode:=force_feedback \
  peer_host:=192.168.10.20 \
  local_port:=15000 \
  peer_port:=15001 \
  force_deadband_master:=0.02 \
  master_coupling_scale:=0.35 \
  max_step:=0.01
```

按关节单独设置主臂死区示例，顺序为 `left_joint_0..6` 后接 `right_joint_0..6`：

```bash
ros2 launch eiriarm_bringup teleop.launch.py \
  role:=master \
  mode:=force_feedback \
  peer_host:=192.168.10.20 \
  local_port:=15000 \
  peer_port:=15001 \
  force_deadband_master_joints:="0.018,0.018,0.016,0.014,0.025,0.025,0.025,0.018,0.018,0.016,0.014,0.025,0.025,0.025"
```

### 直接运行节点

不用 launch 也可以：

```bash
ros2 run eiriarm_controllers teleop_joint_bridge \
  --role master \
  --mode no_feedback \
  --peer-host 192.168.10.20 \
  --local-port 15000 \
  --peer-port 15001
```

常见排查：

- `/teleop/enable` 提示没有 peer joint state：检查 IP、UDP 端口、防火墙、网线
- 从臂不动：确认从臂本地 `real_robot.launch.py` 已启动，且 `/controller_manager` 可用
- enable 失败提示误差过大：先重新执行主臂侧 `/teleop/align`
- UDP 网络正常但 ROS 服务无响应：检查本机是否忘了 `source install/setup.bash`

---

## 11. 夹爪

夹爪不是 `ros2_control` 控制器，而是独立节点，直接走 `/motor/ch1` 和 `/motor/ch2` 的 slot 7。

默认随真机启动：

```bash
ros2 launch eiriarm_bringup real_robot.launch.py
```

禁用夹爪：

```bash
ros2 launch eiriarm_bringup real_robot.launch.py gripper:=false
```

命令话题：

```bash
/gripper_controller/left_gripper/command
/gripper_controller/right_gripper/command
```

命令类型：

```bash
std_msgs/msg/String
```

示例：

```bash
ros2 topic pub --once /gripper_controller/left_gripper/command std_msgs/msg/String "{data: 'open'}"
ros2 topic pub --once /gripper_controller/left_gripper/command std_msgs/msg/String "{data: 'close'}"
ros2 topic pub --once /gripper_controller/right_gripper/command std_msgs/msg/String "{data: 'close 1.5 2.0'}"
ros2 topic pub --once /gripper_controller/left_gripper/command std_msgs/msg/String "{data: 'stop'}"
```

键盘控制：

```bash
ros2 run eiriarm_controllers gripper_keyboard_control
```

参数在：

```bash
src/eiriarm_controllers/config/gripper_controller.yaml
```

每次启动会自动做开合标定，因为夹爪电机断电后绝对开口状态不可持久化。

---

## 12. 关节零点标定

零点标定输出 `joint_offsets_*.yaml`。真机默认读取启动目录下的：

```bash
joint_offsets_dual.yaml
```

### hard-stop 标定

适用于没有零位夹具时，逐关节推到机械硬限位。

先启动 bridge：

```bash
ros2 launch eiriarm_bringup bridge.launch.py
```

左臂：

```bash
ros2 run eiriarm_controllers joint_zero_calibration \
  --mode hard-stop \
  --calibration-yaml joint_calibration_dual_left.yaml \
  --output joint_offsets_left.yaml
```

右臂：

```bash
ros2 run eiriarm_controllers joint_zero_calibration \
  --mode hard-stop \
  --calibration-yaml joint_calibration_dual_right.yaml \
  --output joint_offsets_right.yaml
```

双臂配置可合并为 `joint_offsets_dual.yaml`。标定文件中的关键字段：

- `channel`：左臂通常是 1，右臂通常是 2
- `slot`：电机槽位，臂关节为 0 到 6
- `name`：URDF 关节名
- `motor_type`：DM 电机型号
- `limit_side`：推到正限位或负限位
- `urdf_pos_at_limit`：该硬限位对应的 URDF 角度

### 当前姿态设为零位

适用于已经把机械臂摆到 URDF 零位的情况。默认 dry-run：

```bash
ros2 run eiriarm_controllers zero_at_current_pose \
  --offsets joint_offsets_dual.yaml
```

确认输出合理后写入：

```bash
ros2 run eiriarm_controllers zero_at_current_pose \
  --offsets joint_offsets_dual.yaml \
  --apply
```

常用参数：

| 参数 | 默认 | 说明 |
|---|---:|---|
| `--samples` | `50` | 每个电机平均样本数 |
| `--timeout` | `15.0` | 采样超时 |
| `--max-spread` | `0.02` | 采样期间最大允许抖动 |

### 标定后验证

启动真机后检查：

```bash
ros2 topic echo /joint_states
```

逐个手推关节，确认：

- `/joint_states.position` 在合理范围内
- 正方向和 URDF 方向一致
- 静止时没有明显跳变

如果方向反了，修改 offset YAML 中对应关节的 `axis_sign`。

---

## 13. 摩擦辨识与重力补偿调参（不建议使用，需要拆除电机独立标定，不能整臂标定）

摩擦模型文件默认读取启动目录下的：

```bash
friction_model.yaml
```

摩擦辨识示例：

```bash
ros2 run eiriarm_controllers friction_identification \
  --motor-type DM4310 \
  --channel 1 \
  --slot 4 \
  --output-yaml friction_model.yaml \
  --output-dir friction_results
```

单电机摩擦补偿测试：

```bash
ros2 run eiriarm_controllers friction_compensation_demo \
  --channel 1 \
  --motor-type DM4310 \
  --slot 4 \
  --gain 0.5 \
  --model-yaml friction_model.yaml
```

真机重力补偿参数在：

```bash
src/eiriarm_controllers/config/dual_arm_controllers.yaml
```

调参建议：

1. 先保证 `joint_offsets_dual.yaml` 正确。
2. 在 `controller:=gravity` 下测试每个关节是否明显下坠或上抬。
3. 下坠则略增对应 `gravity_gains`，上抬则略减。
4. 摩擦补偿从小 gain 开始，出现震荡或自激就降低。
5. 仿真配置不要用真机摩擦经验值。

---

## 14. 主要文件

| 路径 | 说明 |
|---|---|
| `src/eiriarm_bringup/launch/bridge.launch.py` | USB-CAN 桥入口 |
| `src/eiriarm_bringup/launch/real_robot.launch.py` | 真机控制主入口 |
| `src/eiriarm_bringup/launch/system.launch.py` | MuJoCo 仿真 + 控制器 |
| `src/eiriarm_bringup/launch/mujoco_panel.launch.py` | 真机 MuJoCo 面板 |
| `src/eiriarm_bringup/launch/go_home.launch.py` | 回零 |
| `src/eiriarm_bringup/launch/record.launch.py` | 示教录制 |
| `src/eiriarm_bringup/launch/replay.launch.py` | 示教回放 |
| `src/eiriarm_controllers/config/dual_arm_controllers.yaml` | 真机控制器参数 |
| `src/eiriarm_controllers/config/dual_arm_sim_controllers.yaml` | 仿真控制器参数 |
| `src/eiriarm_controllers/config/dual_arm_ros2_control.urdf.xacro` | 真机 ros2_control URDF |
| `src/eiriarm_controllers/config/dual_arm_sim_ros2_control.urdf.xacro` | 仿真 ros2_control URDF |
| `src/eiriarm_mujoco/config/simulate.yaml` | MuJoCo 仿真配置 |
| `src/eiriarm_mujoco/config/mujoco_panel.yaml` | MuJoCo 真机面板配置 |
| `joint_offsets_dual.yaml` | 当前双臂零点/方向标定 |
| `friction_model.yaml` | 摩擦模型 |
| `recordings/` | 录制轨迹样例 |

---

## 15. 故障排查

### 找不到串口

```bash
ls -l /dev/ttyACM*
groups | grep dialout
```

如果用户不在 `dialout` 组，需要添加后重新登录。具体可参考USB2CAN包内容

### 电机没有状态

```bash
ros2 topic hz /motor/ch1/state
ros2 topic echo /motor/ch1/state
ros2 topic hz /motor/ch2/state
```

检查：

- bridge 是否已经启动
- `device:=/dev/ttyACM*` 是否正确
- 电机 ID 和 `dm_motors_eiriarm.yaml` 是否匹配
- 电机是否上电
- 下位机呼吸灯是否在闪烁，如不在闪烁，请单击下位机上的复位按钮

### 单电机不能使能/失能，启动失败

检查：

- 各个电机线材是否松动（先断电再检查，xt30 2+2接口极易松动）

### 重力补偿下机械臂乱动，补偿不正确

检查：

- 打开GUI，查看机械臂姿态是否与实际一致，如不一致，可能是太久没有上电导致零位丢失，需要重新标定，也有可能是有的ros进程没有杀干净，建议重启上下位机。

### 控制器切换失败

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

确认 `joint_position_controller` 和 `cartesian_position_controller` 已经 loaded。当前 `real_robot.launch.py` 在双臂模式会加载两者，默认 inactive。

### 录制时机械臂动不了

大概率是 `joint_position_controller` 仍然 active。切回重力补偿：

```bash
ros2 control switch_controllers \
  --activate gravity_compensation_controller \
  --deactivate joint_position_controller cartesian_position_controller
```

### MuJoCo 面板按 `E` 后 slider 被拉回

当前版本进入编辑模式后会暂停 mirror。如果仍出现该问题，确认已经重新编译并 source：

```bash
colcon build --packages-select eiriarm_mujoco eiriarm_bringup
source install/setup.bash
```

### 笛卡尔目标不动

检查：

- 是否 `arms:=dual`
- 是否切到了 `cartesian_position_controller`
- 目标 frame 是否合理
- 目标是否离当前位姿太远导致 IK 不收敛

再次提醒：笛卡尔控制器只做逆解，不做路径规划。
