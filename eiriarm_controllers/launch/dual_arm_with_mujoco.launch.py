"""
Complete launch file for dual-arm robot with MuJoCo simulation and ros2_control.

This launch file:
1. Loads the robot description (URDF with ros2_control tags)
2. Starts the MuJoCo simulator (via topic communication)
3. Starts the controller_manager with Topic-based Hardware Interface
4. Loads and activates the joint impedance controller

Usage:
    ros2 launch eiriarm_controllers dual_arm_with_mujoco.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'urdf_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('dual_arm_support'),
                'urdf',
                'dual_arm_robot.urdf'
            ]),
            description='Path to robot URDF file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'ros2_control_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('eiriarm_controllers'),
                'config',
                'dual_arm_ros2_control.xacro'
            ]),
            description='Path to ros2_control configuration xacro'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('eiriarm_controllers'),
                'config',
                'ros2_control_controllers.yaml'
            ]),
            description='Path to controller configuration file'
        )
    )

    # Initialize Arguments
    urdf_file = LaunchConfiguration('urdf_file')
    ros2_control_config = LaunchConfiguration('ros2_control_config')
    controller_config = LaunchConfiguration('controller_config')

    # Get robot description by combining URDF with ros2_control xacro
    # Note: You need to modify your URDF to include the ros2_control xacro
    # For now, we'll load a simple robot_description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='cat')]),
        ' ',
        urdf_file,
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Node 1: MuJoCo Simulator (existing, topic-based)
    mujoco_simulator = Node(
        package='eiriarm_mujoco',
        executable='simulate',
        name='mujoco_simulator',
        output='screen',
        parameters=[
            # Add any MuJoCo-specific parameters here
        ]
    )

    # Node 2: Controller Manager with Topic-based Hardware Interface
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_config,
        ],
        output='both',
        remappings=[
            # The hardware interface will use these topics by default:
            # Subscribe: /joint_states
            # Publish: /ctrl/effort
        ]
    )

    # Node 3: Robot State Publisher (publishes TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # Spawner: Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Spawner: Joint Impedance Controller
    impedance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_impedance_controller', '--controller-manager', '/controller_manager'],
    )

    # Event handler: Load joint_state_broadcaster first, then impedance controller
    load_impedance_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[impedance_controller_spawner],
        )
    )

    nodes = [
        mujoco_simulator,
        controller_manager,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        load_impedance_after_jsb,
    ]

    return LaunchDescription(declared_arguments + nodes)
