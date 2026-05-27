# EiriArm Bringup

EiriArm 双臂机器人的统一启动入口。同时承载 **真机** 和 **MuJoCo 仿真** 两套 launch；本文档以真机为主。

---

## 真机：两个终端

真机启动**刻意**拆成两条 launch、两个终端。原因是 `usb2can_node` / `dm_motor_bridge`
会以 300 Hz 持续打印串口与 CAN 帧统计，把控制器自己的日志（ENABLE 进度、夹爪
校准、shutdown DISABLE 重试……）淹没掉，调试时几乎无法读。分开跑后两边日志互不
干扰。

```bash
# 终端 1 —— USB-CAN 桥（先启动，让电机进入 ready 状态）
ros2 launch eiriarm_bringup bridge.launch.py

# 终端 2 —— 控制端（ros2_control + 控制器 + 夹爪）
ros2 launch eiriarm_bringup real_robot.launch.py
```

顺序：先 `bridge.launch.py`（确认 `/motor/chN/state` 有数据），再
`real_robot.launch.py`。控制端会自动等到夹爪所在通道的所有 7 个臂关节都
ENABLED 之后才发夹爪命令，避免 CAN 总线在使能阶段相互踩踏。

### 终端 1：`bridge.launch.py`

会启动：

1. `usb2can_node` —— USB-CAN 串口驱动（`/dcu/command`、`/dcu/feedback`、`/imu/data`）
2. `dm_motor_bridge` —— DM MIT 协议聚合桥；把多个 `/motor/chN/cmd` 发布者聚合到
   单一 timer-driven 的 `/dcu/command`（默认 300 Hz），不会随发布者数量叠加。

参数：

| 参数 | 取值 | 默认 | 说明 |
|---|---|---|---|
| `device` | path | `/dev/ttyACM0` | USB-CAN 板的串口 |
| `motors_config` | path | `<usb2can>/config/dm_motors_eiriarm.yaml` | DM 电机限幅 YAML |

### 终端 2：`real_robot.launch.py`

会按顺序启动：

1. `robot_state_publisher` —— 由 xacro 生成 URDF
2. `controller_manager` (`ros2_control_node`) —— 加载 `DMHardwareInterface`
3. `joint_state_broadcaster`（始终 active）
4. `gravity_compensation_controller`（始终 active）
5. 顶层控制器（看 `controller:=` 参数；可选）
6. `gripper_controller_node`（看 `gripper:=` 参数；可选；自动校准）

**默认就处于"重力补偿 + 双夹爪自动校准"的示教模式**，不需要再 `ros2 control switch_controllers`。

参数：

| 参数 | 取值 | 默认 | 说明 |
|---|---|---|---|
| `arms` | `left`/`right`/`dual` | `dual` | 暴露给 ros2_control 的手臂 |
| `controller` | `gravity`/`joint_position`/`cartesian_position` | `gravity` | 顶层控制器（与 gravity 共存）|
| `gripper` | `true`/`false` | `true` | 是否启动 gripper_controller_node |
| `offsets_yaml` | path | `/home/arm/ros2_ws/joint_offsets_dual.yaml` | 14 关节零位/方向标定 YAML |

### `controller:=` 的详细含义

| 值 | 激活的控制器组合 | 用途 |
|---|---|---|
| `gravity` | gravity_compensation_controller | 纯重力补偿，自由示教 |
| `joint_position` | gravity + joint_position_controller | 关节空间 PD 跟踪 |
| `cartesian_position` | gravity + cartesian_position_controller | 笛卡尔 PD 协调（**仅 dual**）|

`gravity_compensation_controller` 在三种模式下都 active —— 它独占 effort 接口，PD 类控制器独占 pos/vel/stiff/damp 接口，二者并行求和送到电机。

---

## 常用启动示例

```bash
# === 终端 1（先起）===
ros2 launch eiriarm_bringup bridge.launch.py
# 自定义串口号：
ros2 launch eiriarm_bringup bridge.launch.py device:=/dev/ttyACM1

# === 终端 2（控制器）===

# 默认: 双臂示教 + 双夹爪
ros2 launch eiriarm_bringup real_robot.launch.py

# 双臂关节 PD 跟踪
ros2 launch eiriarm_bringup real_robot.launch.py controller:=joint_position

# 双臂笛卡尔 PD
ros2 launch eiriarm_bringup real_robot.launch.py controller:=cartesian_position

# 只起左臂(ch1)，不要夹爪
ros2 launch eiriarm_bringup real_robot.launch.py arms:=left gripper:=false
```

---

## 与子 launch 的关系

两个顶层 launch 都是薄封装：

```
eiriarm_bringup/launch/
├── bridge.launch.py
│   └── usb2can/launch/usb2can_with_dm.launch.py        (device, motors_config)
│
└── real_robot.launch.py
    └── eiriarm_controllers/launch/dual_arm.launch.py   (arms, controller, gripper, offsets_yaml)
```

如果想跳过这一层封装，可以直接调子 launch：

```bash
# 只起 bridge
ros2 launch usb2can usb2can_with_dm.launch.py device:=/dev/ttyACM0

# 只起控制端 (bridge 必须已经在跑)
ros2 launch eiriarm_controllers dual_arm.launch.py controller:=joint_position
```

但日常推荐直接用 `bridge.launch.py` + `real_robot.launch.py`。

---

## 关键配置文件

| 路径 | 用途 |
|---|---|
| `eiriarm_controllers/config/dual_arm_controllers.yaml` | ros2_control + 各控制器（joints, gravity_gains, friction_gains 等）|
| `eiriarm_controllers/config/gripper_controller.yaml` | 夹爪节点（校准、力阈值、HOLD PD 等）|
| `eiriarm_controllers/config/dual_arm_ros2_control.urdf.xacro` | URDF（含 `<ros2_control>` 块）|
| `usb2can/config/dm_motors_eiriarm.yaml` | DM 电机 per-motor 限幅 |
| `usb2can/config/usb2can.yaml` | USB-CAN 串口参数 |
| `joint_offsets_dual.yaml` (workspace 根) | 14 关节零位标定 |

---

## 关闭

两个终端各自 `Ctrl+C` 即可。**先关控制端（终端 2），再关 bridge（终端 1）**，否则
关 bridge 时电机还在收 cmd，会丢 DISABLE 帧。

控制端（终端 2）的关闭流程：

1. SIGINT 同时到 `controller_manager` 和 `gripper_controller_node`
2. `gripper_controller_node` 先暂停 cmd 发布，等 arm DISABLE 完成（最多 3 s）
3. `dm_hardware_interface` 在 `on_deactivate` 里 20 Hz × 5 s 发 FD，直到所有关节 `err=0`
4. `gripper_controller_node` 再 5 Hz × 3 s 发 FD 把夹爪也关掉

Bridge（终端 1）`Ctrl+C` 即关，没有特殊收尾。

---

## 仿真

仿真路径独立，不走 `real_robot.launch.py`。详见 `system.launch.py` / `simulation.launch.py` / `mujoco_sim.launch.py` 与 `controllers.launch.py`。

```bash
# MuJoCo + 控制器 (默认 impedance)
ros2 launch eiriarm_bringup system.launch.py

# 仅仿真
ros2 launch eiriarm_bringup mujoco_sim.launch.py

# 仅控制器（连到外部仿真）
ros2 launch eiriarm_bringup controllers.launch.py controller_type:=gravity_compensation
```

注意：仿真用的是 `dual_arm_robot_plug.urdf` + `ros2_control_controllers.yaml`，与真机的 `dual_arm_ros2_control.urdf.xacro` + `dual_arm_controllers.yaml` 是两套独立配置。

---

## 故障排查

### Bridge 起不来
- `ls -l /dev/ttyACM*` 确认设备号
- `groups | grep dialout` 确认当前用户在 dialout 组
- 拔插 USB 后再试

### `/dcu/command` 速率不对
- 期望 ~300 Hz（`dm_motor_bridge` 内部 timer 决定）
- 如果是 600~700 Hz，说明 bridge 是旧版本（per-cmd-publish 行为）。重启 `usb2can` launch。

### 关节 ENABLE 超时
- 检查 `/motor/chN/state`：电机是否在线（`err` 字段）
- 检查电机 ID 是否在 `dm_motors_eiriarm.yaml` 里正确分配槽位
- DM 电机偶发需要冷启动（断电再上电）

### 关节 DISABLE 超时（残留 1~2 个 `err=1`）
- 现已 20 Hz × 5 s 重发 FD（共 100 次机会），仍然超时多半是 CAN 噪声或电机短暂卡顿
- 实际 motor 已关闭（torque=0），下次启动重新 FC 即可
