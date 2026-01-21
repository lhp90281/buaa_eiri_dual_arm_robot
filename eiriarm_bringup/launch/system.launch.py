#!/usr/bin/env python3
"""
Complete System Launch File
Starts MuJoCo simulation and controllers together.

Usage:
  # Impedance controller (default)
  ros2 launch eiriarm_bringup system.launch.py

  # Gravity compensation controller
  ros2 launch eiriarm_bringup system.launch.py controller_type:=gravity_compensation

  # Joint trajectory controller
  ros2 launch eiriarm_bringup system.launch.py controller_type:=joint_trajectory
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='impedance',
        description='Controller type: impedance, gravity_compensation, or joint_trajectory'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable MuJoCo GUI'
    )
    
    # Include MuJoCo simulation
    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('eiriarm_bringup'),
                'launch',
                'mujoco_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
        }.items()
    )
    
    # Include controllers (delayed to ensure MuJoCo is ready)
    controllers_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('eiriarm_bringup'),
                        'launch',
                        'controllers.launch.py'
                    ])
                ]),
                launch_arguments={
                    'controller_type': LaunchConfiguration('controller_type'),
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        controller_type_arg,
        gui_arg,
        mujoco_launch,
        controllers_launch,
    ])
