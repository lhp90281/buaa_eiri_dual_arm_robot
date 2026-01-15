import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. 配置参数
    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="True",
            description="Whether to start RViz"
        )
    ]
    use_rviz = LaunchConfiguration("use_rviz")

    # 2. 动态生成 Robot Description (URDF)
    # 关键修改：使用 Command + xacro，而不是直接读取文件
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dual_arm_support"), "urdf", "dual_arm_robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 3. 控制器配置文件路径
    # 注意：你需要创建这个 yaml 文件（见下文）
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("eiriarm_bringup"),
            "config",
            "controllers.yaml",
        ]
    )

    # 4. 核心节点：ros2_control_node (Controller Manager)
    # 它会加载你的 C++ MujocoHardware 插件
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # 5. Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 6. 加载控制器 (Spawners)
    # 6.1 加载 joint_state_broadcaster (用于发布 /joint_states)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 6.2 加载你的双臂控制器 (假设名字叫 joint_trajectory_controller)
    # 我们使用延时启动，确保 broadcaster 先启动
    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    # )
    
    # 6.2 加载 forward_command_controller (用于简单的 effort 测试)
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "--controller-manager", "/controller_manager"],
    )

    # 7. RViz (可选)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dual_arm_support"), "config", "robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=None, # 你可以使用 IfCondition(use_rviz)
    )

    # 8. 定义启动顺序
    # 确保 control_node 启动后才加载 broadcaster
    # 确保 broadcaster 启动后才加载 robot_controller
    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes)