import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="False",
            description="Whether to start RViz"
        )
    )
    
    use_rviz = LaunchConfiguration("use_rviz")

    # 1. 获取功能包路径 (自动寻找安装位置，不再硬编码路径)
    try:
        support_package_path = get_package_share_directory('dual_arm_support')
    except Exception as e:
        print("Error: Could not find package 'dual_arm_support'. Please make sure it is built and sourced.")
        raise e
    
    # 2. 构建文件的绝对路径
    mjcf_path = os.path.join(support_package_path, 'mjcf', 'dual_arm_robot.xml')
    urdf_path = os.path.join(support_package_path, 'urdf', 'dual_arm_robot.urdf')

    # 3. 读取 URDF 内容
    robot_desc = ""
    if os.path.exists(urdf_path):
        with open(urdf_path, 'r') as inf:
            robot_desc = inf.read()
    else:
        print(f"[ERROR] URDF not found at {urdf_path}")

    # MuJoCo Bridge Node
    mujoco_node = Node(
        package='eiriarm_mujoco',
        executable='mujoco_bridge_node.py',  # Switch to Python script
        name='mujoco_bridge',
        output='screen',
        parameters=[{
            'model_file': mjcf_path,
            'use_sim_time': False
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
            'use_sim_time': False # Bridge 使用系统时间 node->now()，所以这里设为 False
        }]
    )

    # RViz (Conditional)
    rviz_config_file = os.path.join(
        get_package_share_directory('eiriarm_bringup'),
        'rviz',
        'simulation.rviz'  # Assuming you might have one, or use default
    )
    
    # If no config file exists, just run rviz2 without -d
    if os.path.exists(rviz_config_file):
        rviz_args = ['-d', rviz_config_file]
    else:
        rviz_args = []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
        condition=IfCondition(use_rviz)
    )

    nodes = [
        mujoco_node,
        rsp_node,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)