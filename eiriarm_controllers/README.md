# EiriArm Controllers

`eiriarm_controllers` 提供 EiriArm 使用的 ros2_control 硬件接口、控制器插件和辅助脚本。

日常使用请优先看主手册：

```bash
src/eiriarm_bringup/README.md
```

本包包含：

- `DMHardwareInterface`：真机 DM 电机硬件接口
- `TopicBasedHardwareInterface`：MuJoCo 仿真 topic 硬件接口
- `GravityCompensationController`：重力补偿和摩擦前馈
- `JointPositionController`：MIT 五参数关节位置控制，订阅 `/joint_position_command`
- `CartesianPositionControllerPlugin`：双臂笛卡尔目标逆解控制
- `gripper_controller_node`：独立夹爪控制节点
- `go_home`、`teach_replay`、`joint_zero_calibration`、`zero_at_current_pose` 等工具脚本

注意：本包不提供运动规划。笛卡尔控制器只对目标位姿做 IK，并把结果作为关节目标插值执行；它不做碰撞检测、避障或路径搜索。

常用入口：

```bash
# 真机，推荐通过 bringup 启动
ros2 launch eiriarm_bringup bridge.launch.py
ros2 launch eiriarm_bringup real_robot.launch.py

# 直接启动控制器侧，要求 bridge 已经在跑
ros2 launch eiriarm_controllers dual_arm.launch.py

# 回零、录制、回放也推荐通过 bringup 封装
ros2 launch eiriarm_bringup go_home.launch.py
ros2 launch eiriarm_bringup record.launch.py output:=recordings/demo.yaml
ros2 launch eiriarm_bringup replay.launch.py input:=recordings/demo.yaml
```
