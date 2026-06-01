#!/usr/bin/env python3
"""
Control-side launch for the EiriArm dual 7-DoF robot on REAL hardware.

This launch starts ONLY the ros2_control / controllers / gripper side of
the stack. The USB-CAN bridge (usb2can_node + dm_motor_bridge) is
intentionally NOT included here; it lives in `bridge.launch.py` and is
meant to run in its own terminal so its serial-IO chatter does not drown
out the controller logs.

Two-terminal workflow:

  # terminal 1 -- CAN bridge
  ros2 launch eiriarm_bringup bridge.launch.py

  # terminal 2 -- controllers (this file)
  ros2 launch eiriarm_bringup real_robot.launch.py

All knobs are exposed as launch arguments; you should never need to edit
this file or run `ros2 control switch_controllers` by hand.

Common invocations
------------------

  # Both arms, gravity-comp / teach mode, gripper on (defaults):
  ros2 launch eiriarm_bringup real_robot.launch.py

  # Both arms with joint-space PD tracking active immediately:
  ros2 launch eiriarm_bringup real_robot.launch.py controller:=joint_position

  # Joint-space PD tracking with the MuJoCo real-robot panel:
  ros2 launch eiriarm_bringup real_robot.launch.py \
       controller:=joint_position use_gui:=true

  # Gravity-comp teach mode with the MuJoCo mirror/control panel:
  ros2 launch eiriarm_bringup real_robot.launch.py use_gui:=true

  # Both arms with the cartesian PD coordinator:
  ros2 launch eiriarm_bringup real_robot.launch.py controller:=cartesian_position

  # Single-arm bring-up (e.g. left only -- ch1):
  ros2 launch eiriarm_bringup real_robot.launch.py arms:=left

  # No gripper (e.g. swapped end-effector):
  ros2 launch eiriarm_bringup real_robot.launch.py gripper:=false

  # Custom calibration file:
  ros2 launch eiriarm_bringup real_robot.launch.py \
       offsets_yaml:=/path/to/joint_offsets_dual.yaml

Argument summary
----------------

  arms           left | right | dual    (which arm(s) to expose).
  controller     gravity | joint_position | cartesian_position
                 (cartesian_position requires arms=dual).
  gripper        true | false   (start gripper_controller_node).
  offsets_yaml   absolute path to the 14-joint zero/sign calibration YAML.
  use_gui        true | false   (start MuJoCo mirror/target-editor panel).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'arms',
            default_value='dual',
            description="Which arm(s) to expose to ros2_control: 'left', 'right' or 'dual'",
            choices=['left', 'right', 'dual'],
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='gravity',
            description=(
                'Top-tier controller to spawn ACTIVE alongside the always-on '
                'gravity_compensation_controller. No manual '
                '`ros2 control switch_controllers` is required.'
            ),
            choices=['gravity', 'joint_position', 'cartesian_position'],
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='true',
            description='Start the gripper_controller standalone node (auto-calibrates on startup)',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'offsets_yaml',
            default_value='/home/arm/ros2_ws/joint_offsets_dual.yaml',
            description='Absolute path to the 14-joint zero/sign calibration YAML',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            description='Start MuJoCo mirror/target-editor panel alongside real hardware control',
            choices=['true', 'false'],
        ),
    ]

    # ---- ros2_control + controllers + gripper ----
    # NOTE: the USB-CAN bridge is intentionally NOT included here. Run it
    # in a separate terminal via `ros2 launch eiriarm_bringup bridge.launch.py`.
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('eiriarm_controllers'),
                'launch',
                'dual_arm.launch.py',
            ]),
        ),
        launch_arguments={
            'arms':         LaunchConfiguration('arms'),
            'controller':   LaunchConfiguration('controller'),
            'gripper':      LaunchConfiguration('gripper'),
            'offsets_yaml': LaunchConfiguration('offsets_yaml'),
        }.items(),
    )

    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('eiriarm_bringup'),
                'launch',
                'mujoco_panel.launch.py',
            ]),
        ),
        launch_arguments={
            'mode': 'mirror_real',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_gui')),
    )

    return LaunchDescription([*args, control_launch, gui_launch])
