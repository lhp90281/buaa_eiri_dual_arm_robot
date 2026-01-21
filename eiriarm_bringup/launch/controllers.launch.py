#!/usr/bin/env python3
"""
Controllers Launch File
Starts ros2_control with configurable controller type.

Usage:
  # Impedance controller (default)
  ros2 launch eiriarm_bringup controllers.launch.py

  # Gravity compensation controller
  ros2 launch eiriarm_bringup controllers.launch.py controller_type:=gravity_compensation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Get launch arguments
    controller_type = LaunchConfiguration('controller_type').perform(context)
    
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('dual_arm_support'),
        'urdf',
        'dual_arm_robot_plug.urdf'
    )
    
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Select controller configuration based on type
    if controller_type == 'impedance':
        config_file = PathJoinSubstitution([
            FindPackageShare('eiriarm_controllers'),
            'config',
            'ros2_control_controllers.yaml'
        ])
        controllers_to_start = ['joint_state_broadcaster', 'joint_impedance_controller']
    elif controller_type == 'gravity_compensation':
        config_file = PathJoinSubstitution([
            FindPackageShare('eiriarm_controllers'),
            'config',
            'ros2_control_controllers.yaml'
        ])
        controllers_to_start = ['joint_state_broadcaster', 'gravity_compensation_controller']
    else:
        raise ValueError(f"Unknown controller_type: {controller_type}. "
                        f"Valid options: impedance, gravity_compensation")
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }]
    )
    
    # ros2_control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            config_file,
        ],
        output='screen',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )
    
    # Controller spawners
    controller_spawners = []
    for controller_name in controllers_to_start:
        controller_spawners.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[controller_name, '-c', '/controller_manager'],
                output='screen',
            )
        )
    
    nodes_to_start = [
        robot_state_publisher,
        ros2_control_node,
    ] + controller_spawners
    
    return nodes_to_start


def generate_launch_description():
    # Declare launch arguments
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='impedance',
        description='Controller type: impedance or gravity_compensation'
    )
    
    return LaunchDescription([
        controller_type_arg,
        OpaqueFunction(function=launch_setup)
    ])
