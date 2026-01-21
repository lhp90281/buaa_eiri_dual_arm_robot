"""
Example launch file for using ros2_control with eiriarm controllers.

This demonstrates how to:
1. Load the robot description
2. Start the controller manager
3. Load and activate controllers
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
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
            'controller_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('eiriarm_controllers'),
                'config',
                'ros2_control_controllers.yaml'
            ]),
            description='Path to controller configuration file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation (Gazebo/MuJoCo) or real hardware'
        )
    )

    # Initialize Arguments
    controller_config = LaunchConfiguration('controller_config')
    use_sim = LaunchConfiguration('use_sim')

    # Get URDF via xacro (adjust path to your robot description package)
    # Example: robot_description_content = Command([
    #     PathJoinSubstitution([FindExecutable(name='xacro')]),
    #     ' ',
    #     PathJoinSubstitution([
    #         FindPackageShare('your_robot_description'),
    #         'urdf',
    #         'your_robot.urdf.xacro'
    #     ])
    # ])
    
    # For this example, assume you load URDF from a parameter
    robot_description = {'robot_description': ''}  # Replace with actual URDF loading

    # Controller Manager Node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_config,
        ],
        output='both',
        remappings=[
            # Remap if needed, e.g., for joint_states
        ]
    )

    # Spawner for Joint State Broadcaster (publishes /joint_states)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Spawner for Gravity Compensation Controller
    gravity_comp_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gravity_compensation_controller', '--controller-manager', '/controller_manager'],
    )

    # Spawner for Joint Impedance Controller
    impedance_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_impedance_controller', '--controller-manager', '/controller_manager'],
    )

    # Event handlers to load controllers sequentially
    # First load joint_state_broadcaster, then load other controllers
    load_gravity_comp_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gravity_comp_spawner],
        )
    )

    # Or you can choose to load impedance controller instead
    # load_impedance_after_jsb = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[impedance_controller_spawner],
    #     )
    # )

    nodes = [
        controller_manager_node,
        joint_state_broadcaster_spawner,
        load_gravity_comp_after_jsb,
        # Uncomment to use impedance controller instead:
        # load_impedance_after_jsb,
    ]

    return LaunchDescription(declared_arguments + nodes)
