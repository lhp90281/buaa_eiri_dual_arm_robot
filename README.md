# EiriArm Workspace Index

这个 `src/` 目录是 EiriArm ROS2 工作区的源码索引。日常操作优先看主使用手册：

```bash
eiriarm_bringup/README.md
```

## 包索引

| 路径 | 作用 |
|---|---|
| `eiriarm_bringup/` | 主启动入口和完整使用文档。包含真机、仿真、MuJoCo 面板、回零、录制、回放等 launch。 |
| `eiriarm_controllers/` | ros2_control 硬件接口、重力补偿、关节位置控制、笛卡尔 IK 控制、夹爪节点和标定/录制脚本。 |
| `eiriarm_mujoco/` | MuJoCo 仿真节点和真机 mirror/target editor 面板。 |
| `USB2CAN/usb2can/` | USB-CAN 驱动、DM 电机桥、CAN 消息定义和电机配置。 |
| `description/dual_arm_support/` | 双臂 URDF、MJCF、mesh 和显示 launch。 |
| `description/left_arm_description/` | 左臂描述包。 |
| `description/right_arm_description/` | 右臂描述包。 |
| `description/gripper_description/` | 夹爪描述包。 |

## 常用入口

```bash
# 编译
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

# 真机：终端 1
ros2 launch eiriarm_bringup bridge.launch.py

# 真机：终端 2，推荐带 MuJoCo GUI
ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true

# 仿真
ros2 launch eiriarm_bringup system.launch.py
```

## 功能边界

本工作区提供硬件通信、控制器、MuJoCo 仿真/显示、标定、示教录制和回放。它不提供运动规划；笛卡尔控制器只提供目标位姿的逆解和关节空间插值，不做路径规划、避障或碰撞检测。
