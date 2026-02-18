#!/usr/bin/env python3
"""Controllers Launch File
Starts ros2_control with configurable controller type.

Usage:
  # Impedance controller (default)
  ros2 launch eiriarm_bringup controllers.launch.py

  # Gravity compensation controller
  ros2 launch eiriarm_bringup controllers.launch.py controller_type:=gravity_compensation

  # Cartesian position controller
  ros2 launch eiriarm_bringup controllers.launch.py controller_type:=cartesian_position

  # With gripper controller
  ros2 launch eiriarm_bringup controllers.launch.py enable_gripper:=true
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
    enable_gripper = LaunchConfiguration('enable_gripper').perform(context).lower() == 'true'
    
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
        controllers_to_start = ['joint_impedance_controller']
    elif controller_type == 'gravity_compensation':
        config_file = PathJoinSubstitution([
            FindPackageShare('eiriarm_controllers'),
            'config',
            'ros2_control_controllers.yaml'
        ])
        controllers_to_start = ['gravity_compensation_controller']
    elif controller_type == 'cartesian_position':
        config_file = PathJoinSubstitution([
            FindPackageShare('eiriarm_controllers'),
            'config',
            'ros2_control_controllers.yaml'
        ])
        controllers_to_start = ['cartesian_position_controller']
    else:
        raise ValueError(f"Unknown controller_type: {controller_type}. "
                        f"Valid options: impedance, gravity_compensation, cartesian_position")
    
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
    
    # Gripper controller (standalone node)
    if enable_gripper:
        gripper_config_file = os.path.join(
            get_package_share_directory('eiriarm_controllers'),
            'config',
            'gripper_controller.yaml'
        )
        gripper_controller = Node(
            package='eiriarm_controllers',
            executable='gripper_controller_node',
            name='gripper_controller',
            output='screen',
            parameters=[gripper_config_file],
        )
        nodes_to_start.append(gripper_controller)
    
    return nodes_to_start


def generate_launch_description():
    # Declare launch arguments
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='impedance',
        description='Controller type: impedance, gravity_compensation, or cartesian_position'
    )
    
    enable_gripper_arg = DeclareLaunchArgument(
        'enable_gripper',
        default_value='true',
        description='Enable gripper controller'
    )
    
    return LaunchDescription([
        controller_type_arg,
        enable_gripper_arg,
        OpaqueFunction(function=launch_setup)
    ])
