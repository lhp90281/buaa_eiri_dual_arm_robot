#!/usr/bin/env python3
"""
ros2_control launch for the dual 7-DoF arm (eiriarm) on real hardware.

This is the control-side launch (controller_manager + spawners + optional
gripper). The CAN bridge is a SEPARATE responsibility -- usually you want
the top-level wrapper that brings up everything together:

  ros2 launch eiriarm_bringup real_robot.launch.py

...which forwards all of the args below and additionally starts
usb2can_node + dm_motor_bridge. Run this launch directly only if the bridge
is already up in another terminal.

(Single board, two channels: ch1 = left arm, ch2 = right arm. Grippers on
ch1.id7 / ch2.id7 are NOT exposed to ros2_control here -- they are owned by
the standalone gripper_controller_node.)

Brings up:
  1. robot_state_publisher (URDF processed via xacro with the ros2_control block)
  2. controller_manager / ros2_control_node loading DMHardwareInterface
  3. joint_state_broadcaster (always active)
  4. gravity_compensation_controller (always active)
  5. joint_position_controller (always LOADED; ACTIVE iff
     controller:=joint_position, otherwise inactive). Kept loaded in
     every mode so the helper launches `go_home.launch.py` and
     `replay.launch.py` can switch into it without the operator
     touching `ros2 control switch_controllers`.
  6. cartesian_position_controller (active, ONLY when
     controller:=cartesian_position; requires arms:=dual).
  7. gripper_controller standalone node (talks to ch1.id7 / ch2.id7
     directly, auto-calibrates open->close on startup; toggle with
     gripper:=true|false).

Usage:
  # both arms in pure gravity-comp / teach mode (default):
  ros2 launch eiriarm_controllers dual_arm.launch.py

  # both arms with joint-space PD tracking active out of the gate:
  ros2 launch eiriarm_controllers dual_arm.launch.py controller:=joint_position

  # both arms with the cartesian PD coordinator:
  ros2 launch eiriarm_controllers dual_arm.launch.py controller:=cartesian_position

  # only the left arm (ch1, 7 joints):
  ros2 launch eiriarm_controllers dual_arm.launch.py arms:=left

  # custom calibration file:
  ros2 launch eiriarm_controllers dual_arm.launch.py \
       offsets_yaml:=/home/arm/ros2_ws/joint_offsets_dual.yaml

  # don't start the gripper controller (e.g. when you've swapped grippers):
  ros2 launch eiriarm_controllers dual_arm.launch.py gripper:=false

Notes for single-arm mode:
  * The full dual-arm URDF (both arms + grippers) is ALWAYS loaded into
    Pinocchio so gravity compensation stays correct.  Only the <ros2_control>
    block (and the per-joint arrays in controllers.yaml) is sliced to one arm.
  * controller:=cartesian_position is rejected in single-arm mode because the
    controller is a dual-arm coordinator.
  * Single-arm calibration still uses joint_offsets_dual.yaml; the YAML may
    contain entries for the OFF arm -- the hardware interface ignores any
    joint whose name is not declared in the active ros2_control block.
"""

import os
import tempfile
from typing import Any

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Which slice of the per-joint arrays to keep for each 'arms' value.
# joints in dual_arm_controllers.yaml are ordered: left_joint_0..6 then right_joint_0..6
_ARM_SLICE = {
    'left':  slice(0, 7),
    'right': slice(7, 14),
    'dual':  slice(0, 14),
}

# Keys inside gravity_compensation_controller/ros__parameters whose value is
# a 14-element array aligned with 'joints' and therefore must be sliced.
_GC_JOINT_ARRAY_KEYS = ('joints', 'motor_types', 'gravity_gains', 'friction_gains')

# Same idea for joint_position_controller.
_JP_JOINT_ARRAY_KEYS = ('joints', 'kp_gains', 'kd_gains')

def _slice_controllers_yaml(src_path: str, arms: str) -> str:
    """Read the dual-arm controllers yaml and slice the per-joint arrays for
    the selected arms. For 'dual', the file is returned unchanged. For
    'left' / 'right', a sliced copy is written to /tmp and that path is
    returned.

    Also drops cartesian_position_controller in single-arm mode since the
    controller is a dual-arm coordinator.
    """
    if arms == 'dual':
        return src_path

    if arms not in _ARM_SLICE:
        raise ValueError(
            f"arms must be 'left', 'right' or 'dual'; got {arms!r}")

    with open(src_path, 'r') as f:
        cfg: dict[str, Any] = yaml.safe_load(f)

    sl = _ARM_SLICE[arms]

    # Slice gravity_compensation_controller per-joint arrays.
    gc = cfg.get('gravity_compensation_controller', {}).get('ros__parameters', {})
    for key in _GC_JOINT_ARRAY_KEYS:
        val = gc.get(key)
        if isinstance(val, list) and len(val) == 14:
            gc[key] = val[sl]

    # Slice joint_position_controller per-joint arrays.
    jp = cfg.get('joint_position_controller', {}).get('ros__parameters', {})
    for key in _JP_JOINT_ARRAY_KEYS:
        val = jp.get(key)
        if isinstance(val, list) and len(val) == 14:
            jp[key] = val[sl]

    # cartesian_position_controller is a dual-arm coordinator; drop it in
    # single-arm mode.
    cfg.pop('cartesian_position_controller', None)

    out_path = os.path.join(
        tempfile.gettempdir(), f'dual_arm_controllers_{arms}.yaml')
    with open(out_path, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    return out_path


_VALID_CONTROLLERS = ('gravity', 'joint_position', 'cartesian_position')


def launch_setup(context, *args, **kwargs):
    arms = LaunchConfiguration('arms').perform(context).strip().lower()
    if arms not in _ARM_SLICE:
        raise RuntimeError(
            f"arms must be 'left', 'right' or 'dual'; got {arms!r}")

    controller = LaunchConfiguration('controller').perform(context).strip().lower()
    if controller not in _VALID_CONTROLLERS:
        raise RuntimeError(
            f"controller must be one of {_VALID_CONTROLLERS}; got {controller!r}")
    if controller == 'cartesian_position' and arms != 'dual':
        raise RuntimeError(
            f"controller:=cartesian_position requires arms:=dual; got arms:={arms!r}. "
            f"The cartesian_position_controller is a dual-arm coordinator.")

    enable_gripper   = LaunchConfiguration('gripper').perform(context).lower() == 'true'

    pkg_eiriarm = FindPackageShare('eiriarm_controllers')

    xacro_path = PathJoinSubstitution(
        [pkg_eiriarm, 'config', 'dual_arm_ros2_control.urdf.xacro'])

    # Resolve the source yaml absolute path (FindPackageShare gives a
    # Substitution, we need the string here to read and slice the file).
    src_controllers_yaml = os.path.join(
        FindPackageShare('eiriarm_controllers').perform(context),
        'config', 'dual_arm_controllers.yaml')
    controllers_yaml = _slice_controllers_yaml(src_controllers_yaml, arms)

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', xacro_path,
                ' offsets_yaml:=', LaunchConfiguration('offsets_yaml'),
                ' arms:=', arms,
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_yaml],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Gravity compensation runs underneath every supported controller (it
    # owns the effort interface; the others claim pos/vel/stiff/damp). It is
    # always active when this launch is used.
    gravity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gravity_compensation_controller', '-c', '/controller_manager'],
        output='screen',
    )

    nodes = [
        robot_state_publisher,
        ros2_control_node,
        jsb_spawner,
        gravity_spawner,
    ]

    # Top-tier controller layout:
    #   * gravity_compensation_controller: always active (above).
    #   * joint_position_controller: ALWAYS LOADED. Active when
    #       controller==joint_position, otherwise inactive. We load it
    #       inactive even in gravity / cartesian mode so the helper
    #       scripts (`go_home`, teach `replay`) can switch into it via
    #       /controller_manager/switch_controller without the operator
    #       having to load it by hand.
    #   * cartesian_position_controller: only loaded when explicitly
    #       selected (it is a dual-arm coordinator, requires arms==dual,
    #       and is not used by any of the bringup helper scripts).
    jp_args = ['joint_position_controller', '-c', '/controller_manager']
    if controller != 'joint_position':
        jp_args.append('--inactive')
    nodes.append(Node(
        package='controller_manager',
        executable='spawner',
        arguments=jp_args,
        output='screen',
    ))

    if controller == 'cartesian_position':
        # Pre-condition checked above (arms == 'dual').
        nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            arguments=['cartesian_position_controller', '-c', '/controller_manager'],
            output='screen',
        ))
    # else controller in {gravity, joint_position}: nothing else to spawn.

    # Gripper controller: standalone node OUTSIDE ros2_control. Talks directly
    # to /motor/ch1/cmd (left, slot 7) and /motor/ch2/cmd (right, slot 7),
    # auto-runs an open->close calibration on startup. Disable in single-arm
    # mode for the OFF channel by tweaking left_enabled / right_enabled in
    # gripper_controller.yaml; here we just toggle the whole node.
    if enable_gripper:
        gripper_yaml = os.path.join(
            FindPackageShare('eiriarm_controllers').perform(context),
            'config', 'gripper_controller.yaml')
        gripper_params = [gripper_yaml]
        # Disable the off channel automatically in single-arm mode so we don't
        # spam /motor/ch{2,1}/cmd for a gripper that isn't physically wired.
        if arms == 'left':
            gripper_params.append({'right_enabled': False})
        elif arms == 'right':
            gripper_params.append({'left_enabled': False})
        gripper_node = Node(
            package='eiriarm_controllers',
            executable='gripper_controller_node',
            name='gripper_controller',
            output='screen',
            parameters=gripper_params,
        )
        nodes.append(gripper_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arms',
            default_value='dual',
            description="Which arm(s) to expose to ros2_control: 'left', 'right' or 'dual'",
            choices=['left', 'right', 'dual'],
        ),
        DeclareLaunchArgument(
            'offsets_yaml',
            default_value='/home/arm/ros2_ws/joint_offsets_dual.yaml',
            description='Absolute path to the 14-joint zero/sign calibration YAML',
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='gravity',
            description=(
                'Top-tier controller to load active alongside '
                'gravity_compensation_controller (which is always active). '
                'joint_position_controller is always LOADED (active iff '
                'this is joint_position, otherwise inactive) so the '
                'go_home / replay helpers can switch into it without '
                'manual `ros2 control` calls.'
            ),
            choices=list(_VALID_CONTROLLERS),
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='true',
            description='Start the gripper_controller standalone node (auto-calibrates on startup)',
            choices=['true', 'false'],
        ),
        OpaqueFunction(function=launch_setup),
    ])
