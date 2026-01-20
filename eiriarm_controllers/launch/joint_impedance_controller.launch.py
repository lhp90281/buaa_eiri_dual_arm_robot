import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Use the static URDF file which has corrected inertia parameters
    # Ideally this should come from a description package or parameter, but matching existing pattern
    urdf_file = '/home/ubuntu/ros2_ws/src/description/dual_arm_support/urdf/dual_arm_robot.urdf'
    
    config_file = os.path.join(
        get_package_share_directory('eiriarm_controllers'),
        'config',
        'joint_impedance.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_frequency',
            default_value='500.0',
            description='Control loop frequency (Hz)'
        ),
        DeclareLaunchArgument(
            'stiffness',
            default_value='30.0',
            description='Default Stiffness (Kp) for impedance control'
        ),
        DeclareLaunchArgument(
            'damping',
            default_value='8.0',
            description='Default Damping (Kd) for impedance control'
        ),
        DeclareLaunchArgument(
            'max_effort',
            default_value='50.0',
            description='Maximum torque effort'
        ),
        DeclareLaunchArgument(
            'position_error_limit',
            default_value='0.15',
            description='Position error clamp limit (rad)'
        ),
        Node(
            package='eiriarm_controllers',
            executable='joint_impedance_controller',
            name='joint_impedance_controller',
            output='screen',
            parameters=[
                config_file, # Load from YAML first
                {
                    'urdf_path': urdf_file,
                    'control_frequency': LaunchConfiguration('control_frequency'),
                    'stiffness': LaunchConfiguration('stiffness'), # Allow override from CLI
                    'damping': LaunchConfiguration('damping'),      # Allow override from CLI
                    'max_effort': LaunchConfiguration('max_effort'),
                    'position_error_limit': LaunchConfiguration('position_error_limit')
                }
            ]
        )
    ])
