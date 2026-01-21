"""
Test launch file for Topic Bridge with MuJoCo simulation.

This properly loads the robot_description from file and starts the controller_manager.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
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
                'dual_arm_robot_plug.urdf'
            ]),
            description='Path to robot URDF file with ros2_control configuration'
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
    controller_config = LaunchConfiguration('controller_config')

    # Load robot description from file
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='cat')]),
        ' ',
        urdf_file,
    ])
    
    # Wrap in ParameterValue to explicitly mark as string
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Controller Manager Node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_config,
        ],
        output='both',
    )

    # Robot State Publisher (publishes TF)
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
        controller_manager,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        load_impedance_after_jsb,
    ]

    return LaunchDescription(declared_arguments + nodes)
