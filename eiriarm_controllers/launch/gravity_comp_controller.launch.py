import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Use the static URDF file which has corrected inertia parameters
    urdf_file = '/home/ubuntu/ros2_ws/src/description/dual_arm_support/urdf/dual_arm_robot.urdf'

    return LaunchDescription([
        DeclareLaunchArgument(
            'gravity_gain',
            default_value='1.0',
            description='Gain multiplier for gravity compensation torques'
        ),
        Node(
            package='eiriarm_controllers',
            executable='gravity_comp_controller',
            name='gravity_comp_controller',
            output='screen',
            parameters=[
                {'urdf_path': urdf_file},
                {'gravity_gain': LaunchConfiguration('gravity_gain')}
            ]
        )
    ])
