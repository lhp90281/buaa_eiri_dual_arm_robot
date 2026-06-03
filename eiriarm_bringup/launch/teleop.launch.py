#!/usr/bin/env python3
"""Launch the UDP teleoperation bridge.

Run this after the local robot has already been brought up with
bridge.launch.py and real_robot.launch.py. The two robot hosts may use
different ROS_DOMAIN_ID values; only the UDP peer_host/ports must be reachable.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'role',
            description="This host's teleop role: master or slave",
            choices=['master', 'slave'],
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='no_feedback',
            description='Teleop mode: no_feedback or force_feedback',
            choices=['no_feedback', 'force_feedback'],
        ),
        DeclareLaunchArgument(
            'peer_host',
            description='Peer host IP address or hostname',
        ),
        DeclareLaunchArgument(
            'bind_host',
            default_value='0.0.0.0',
            description='Local UDP bind address',
        ),
        DeclareLaunchArgument(
            'local_port',
            default_value='15000',
            description='Local UDP receive port',
        ),
        DeclareLaunchArgument(
            'peer_port',
            default_value='15001',
            description='Peer UDP receive port',
        ),
        DeclareLaunchArgument(
            'rate_hz',
            default_value='50.0',
            description='UDP state and command update rate',
        ),
        DeclareLaunchArgument(
            'align_duration',
            default_value='5.0',
            description='Master-to-slave alignment ramp duration, seconds',
        ),
        DeclareLaunchArgument(
            'timeout',
            default_value='0.3',
            description='Peer UDP timeout before auto-disable, seconds',
        ),
        DeclareLaunchArgument(
            'max_start_error',
            default_value='0.5',
            description='Maximum joint error allowed when enabling, rad',
        ),
        DeclareLaunchArgument(
            'max_runtime_error',
            default_value='1.0',
            description='Maximum joint error allowed while enabled, rad',
        ),
        DeclareLaunchArgument(
            'max_step',
            default_value='0.03',
            description='Maximum commanded target change per cycle, rad',
        ),
        DeclareLaunchArgument(
            'force_deadband_master',
            default_value='0.015',
            description='Force-feedback master soft deadband, rad',
        ),
        DeclareLaunchArgument(
            'force_deadband_slave',
            default_value='0.0',
            description='Force-feedback slave soft deadband, rad',
        ),
        DeclareLaunchArgument(
            'force_deadband_hysteresis',
            default_value='0.004',
            description='Force-feedback deadband hysteresis, rad',
        ),
        DeclareLaunchArgument(
            'force_deadband_master_joints',
            default_value='',
            description='Optional 14-value master deadband list, comma/space separated',
        ),
        DeclareLaunchArgument(
            'force_deadband_slave_joints',
            default_value='',
            description='Optional 14-value slave deadband list, comma/space separated',
        ),
        DeclareLaunchArgument(
            'master_coupling_scale',
            default_value='0.45',
            description='Force-feedback master coupling scale',
        ),
        DeclareLaunchArgument(
            'slave_coupling_scale',
            default_value='1.0',
            description='Force-feedback slave coupling scale',
        ),
    ]

    teleop = Node(
        package='eiriarm_controllers',
        executable='teleop_joint_bridge',
        name='teleop_joint_bridge',
        output='screen',
        arguments=[
            '--role', LaunchConfiguration('role'),
            '--mode', LaunchConfiguration('mode'),
            '--peer-host', LaunchConfiguration('peer_host'),
            '--bind-host', LaunchConfiguration('bind_host'),
            '--local-port', LaunchConfiguration('local_port'),
            '--peer-port', LaunchConfiguration('peer_port'),
            '--rate-hz', LaunchConfiguration('rate_hz'),
            '--align-duration', LaunchConfiguration('align_duration'),
            '--timeout', LaunchConfiguration('timeout'),
            '--max-start-error', LaunchConfiguration('max_start_error'),
            '--max-runtime-error', LaunchConfiguration('max_runtime_error'),
            '--max-step', LaunchConfiguration('max_step'),
            '--force-deadband-master', LaunchConfiguration('force_deadband_master'),
            '--force-deadband-slave', LaunchConfiguration('force_deadband_slave'),
            '--force-deadband-hysteresis', LaunchConfiguration('force_deadband_hysteresis'),
            '--force-deadband-master-joints', LaunchConfiguration('force_deadband_master_joints'),
            '--force-deadband-slave-joints', LaunchConfiguration('force_deadband_slave_joints'),
            '--master-coupling-scale', LaunchConfiguration('master_coupling_scale'),
            '--slave-coupling-scale', LaunchConfiguration('slave_coupling_scale'),
        ],
    )

    return LaunchDescription([*args, teleop])
