import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Paths to models
    # Note: Ideally these should be found via get_package_share_directory if they were installed.
    # Since we are in dev mode, we point to the src directly or assume they are installed to share.
    # Let's use absolute paths to source for now to avoid install issues during dev iterations.
    
    # You can change these to use FindPackageShare if you install the description package properly.
    mjcf_path = '/home/ubuntu/ros2_ws/src/description/dual_arm_support/mjcf/dual_arm_robot.xml'
    urdf_path = '/home/ubuntu/ros2_ws/src/description/dual_arm_support/urdf/dual_arm_robot.urdf'

    # Read URDF
    try:
        with open(urdf_path, 'r') as inf:
            robot_desc = inf.read()
    except FileNotFoundError:
        print(f"URDF not found at {urdf_path}")
        robot_desc = ""

    # MuJoCo Bridge Node
    mujoco_node = Node(
        package='eiriarm_mujoco',
        executable='mujoco_bridge_node',
        name='mujoco_bridge',
        output='screen',
        parameters=[{
            'model_file': mjcf_path
        }]
    )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        mujoco_node,
        rsp_node,
        rviz_node
    ])
