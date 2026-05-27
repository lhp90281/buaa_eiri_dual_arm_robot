#!/usr/bin/env python3
"""
USB-CAN bridge bring-up for EiriArm.

Starts the two CAN-side nodes ONLY:

  - usb2can_node      (USB serial <-> /dcu/{command,feedback}, /imu/data)
  - dm_motor_bridge   (/dcu/* <-> /motor/ch{1,2,3}/{cmd,state})

Run this in its OWN terminal so its serial-IO chatter doesn't drown out the
controller logs. Then start the control side in a second terminal:

  # terminal 1
  ros2 launch eiriarm_bringup bridge.launch.py

  # terminal 2
  ros2 launch eiriarm_bringup real_robot.launch.py

Arguments
---------

  device         /dev/ttyACM* path of the USB-CAN board (default /dev/ttyACM0).
  motors_config  Per-motor limits YAML loaded by dm_motor_bridge
                 (default: <usb2can>/config/dm_motors_eiriarm.yaml).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_motors_config = os.path.join(
        get_package_share_directory('usb2can'),
        'config',
        'dm_motors_eiriarm.yaml',
    )

    args = [
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='USB-CAN serial device path (e.g. /dev/ttyACM0)',
        ),
        DeclareLaunchArgument(
            'motors_config',
            default_value=default_motors_config,
            description='Per-motor limits YAML loaded by dm_motor_bridge',
        ),
    ]

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('usb2can'),
                'launch',
                'usb2can_with_dm.launch.py',
            ]),
        ),
        launch_arguments={
            'device':        LaunchConfiguration('device'),
            'motors_config': LaunchConfiguration('motors_config'),
        }.items(),
    )

    return LaunchDescription([*args, bridge_launch])
