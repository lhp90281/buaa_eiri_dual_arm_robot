#!/usr/bin/env python3
"""go_home.py -- ramp the arm(s) to q=0 over N seconds, then restore gravity comp.

Sequence:
    1. Read the position controller's `joints` parameter so we know which
       joints to drive (and at what order).
    2. switch_controller: activate joint_position_controller, deactivate
       gravity_compensation_controller. The activation snapshots the
       current measured pose as hold_pos_, so the next step ramps from
       wherever the arm is right now.
    3. Publish a 1-point JointTrajectory with positions all zero and
       time_from_start = --duration. The controller's pre-roll segment
       linearly interpolates from hold_pos_ to zero over that window.
    4. Sleep --duration (+ small margin) so the ramp completes.
    5. switch_controller: activate gravity_compensation_controller,
       deactivate joint_position_controller. Hand-off is bumpless because
       on_deactivate zeros kp/kd before releasing the joints.

Cleanup guarantees:
    - On Ctrl-C / exception, step 5 still runs (try/finally), so we never
      leave the arm sitting under a stiff PD with no operator in the loop.
    - --no-restore-gravity skips step 5 if you want to immediately replay
      another trajectory after the ramp without re-switching.

Examples:
    # default: 5s ramp, restore gravity comp
    ros2 run eiriarm_controllers go_home

    # slow 8s ramp; leave joint_position_controller active for the next
    # ros2 topic pub / teach_replay you intend to run
    ros2 run eiriarm_controllers go_home --duration 8.0 --no-restore-gravity

    # only zero the left arm
    ros2 run eiriarm_controllers go_home --joints \
        left_joint_0 left_joint_1 left_joint_2 left_joint_3 \
        left_joint_4 left_joint_5 left_joint_6
"""
import argparse
import signal
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions

from controller_manager_msgs.srv import SwitchController
from rcl_interfaces.srv import GetParameters
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ParameterType.PARAMETER_STRING_ARRAY -- hard-coded so we don't need a
# runtime import of rcl_interfaces.msg.ParameterType just for one constant.
PARAM_STRING_ARRAY = 9


def _call_service(node, client, request, timeout=5.0):
    """Wait for the service, call it, spin until the future completes."""
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    fut = client.call_async(request)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def query_controller_joints(node, controller_name):
    """Return the 'joints' string-array parameter of `controller_name`."""
    srv = f'/{controller_name}/get_parameters'
    client = node.create_client(GetParameters, srv)
    req = GetParameters.Request()
    req.names = ['joints']
    resp = _call_service(node, client, req, timeout=3.0)
    node.destroy_client(client)
    if resp is None or not resp.values:
        return None
    v = resp.values[0]
    if v.type != PARAM_STRING_ARRAY:
        node.get_logger().warn(
            f"/{controller_name}/joints has unexpected type {v.type}")
        return None
    return list(v.string_array_value)


def switch_controllers(node, activate=None, deactivate=None):
    """Atomic activate/deactivate via /controller_manager/switch_controller.

    Uses BEST_EFFORT strictness, so a no-op (e.g. deactivating an already-
    inactive controller) does not fail the call.
    """
    activate = list(activate or [])
    deactivate = list(deactivate or [])
    if not activate and not deactivate:
        return True
    client = node.create_client(
        SwitchController, '/controller_manager/switch_controller')
    req = SwitchController.Request()
    req.activate_controllers = activate
    req.deactivate_controllers = deactivate
    req.strictness = SwitchController.Request.BEST_EFFORT
    req.activate_asap = False
    # `timeout` is a builtin_interfaces/Duration sub-message in Humble.
    # Leaving it at its default-constructed value (0s) tells the
    # controller manager to use its own internal default.
    resp = _call_service(node, client, req, timeout=10.0)
    node.destroy_client(client)
    if resp is None:
        node.get_logger().error(
            "/controller_manager/switch_controller did not respond")
        return False
    if not resp.ok:
        node.get_logger().error(
            f"switch_controller returned ok=False "
            f"(activate={activate}, deactivate={deactivate})")
        return False
    node.get_logger().info(
        f"controllers switched (+{activate} -{deactivate})")
    return True


def build_zero_trajectory(joints, duration_sec):
    """1-point JointTrajectory: q_target = 0 reached over `duration_sec`."""
    traj = JointTrajectory()
    traj.joint_names = list(joints)
    traj.header.stamp.sec = 0
    traj.header.stamp.nanosec = 0
    pt = JointTrajectoryPoint()
    pt.positions = [0.0] * len(joints)
    sec = int(duration_sec)
    nsec = int(round((duration_sec - sec) * 1e9))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    pt.time_from_start = Duration(sec=sec, nanosec=nsec)
    traj.points.append(pt)
    return traj


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--duration', type=float, default=5.0,
                   help='ramp seconds from current pose to q=0 (default 5.0)')
    p.add_argument('--position-controller', default='joint_position_controller',
                   help='controller to drive the ramp '
                        '(default joint_position_controller)')
    p.add_argument('--gravity-controller', default='gravity_compensation_controller',
                   help='controller to restore afterwards '
                        '(default gravity_compensation_controller)')
    p.add_argument('--command-topic', default='/joint_position_command',
                   help='trajectory topic (default /joint_position_command)')
    p.add_argument('--joints', nargs='*', default=None,
                   help='override joint list (default: query controller)')
    p.add_argument('--no-restore-gravity', action='store_true',
                   help='leave joint_position_controller active afterwards')
    args = p.parse_args()

    if args.duration <= 0.0:
        print("ERROR: --duration must be > 0", file=sys.stderr)
        return 2

    # Take over Ctrl-C so we can do clean cleanup (rclpy's default SIGINT
    # handler would shutdown the context, breaking our restore step).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('go_home')
    stop_flag = {'val': False}

    def on_signal(signum, frame):
        stop_flag['val'] = True

    prev_sigint = signal.signal(signal.SIGINT, on_signal)
    prev_sigterm = signal.signal(signal.SIGTERM, on_signal)

    rc = 0
    switched_to_position = False
    try:
        # ---- resolve joint list ----------------------------------------
        joints = args.joints
        if not joints:
            joints = query_controller_joints(node, args.position_controller)
            if not joints:
                node.get_logger().error(
                    f"could not read 'joints' from /{args.position_controller}; "
                    "is it loaded? Otherwise pass --joints explicitly.")
                return 1
        node.get_logger().info(
            f"target: {len(joints)} joint(s) -> 0 over {args.duration:.2f}s")
        for j in joints:
            node.get_logger().info(f"  - {j}")

        # ---- switch to position controller -----------------------------
        if not switch_controllers(
                node,
                activate=[args.position_controller],
                deactivate=[args.gravity_controller]):
            return 1
        switched_to_position = True

        # ---- publish q=0 trajectory ------------------------------------
        pub = node.create_publisher(
            JointTrajectory, args.command_topic,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE),
        )
        node.get_logger().info(
            f"waiting for subscriber on {args.command_topic} ...")
        deadline = time.time() + 5.0
        while (pub.get_subscription_count() == 0
               and time.time() < deadline
               and not stop_flag['val']):
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            node.get_logger().error(
                f"no subscriber on {args.command_topic} after activating "
                f"{args.position_controller}")
            return 1

        traj = build_zero_trajectory(joints, args.duration)
        pub.publish(traj)
        node.get_logger().info(
            f"published q=0 trajectory; waiting {args.duration:.2f}s ...")

        # ---- wait for ramp completion ----------------------------------
        end_t = time.time() + args.duration + 0.3
        interrupted = False
        while time.time() < end_t:
            if stop_flag['val']:
                interrupted = True
                break
            rclpy.spin_once(node, timeout_sec=0.05)

        if interrupted:
            node.get_logger().info(
                "interrupted; restoring gravity comp (arm holds wherever it is)")
            rc = 130  # conventional Ctrl-C exit code
        else:
            node.get_logger().info("ramp complete; q ~= 0")
    finally:
        # ---- restore gravity comp (always, unless suppressed) ----------
        if switched_to_position and not args.no_restore_gravity:
            switch_controllers(
                node,
                activate=[args.gravity_controller],
                deactivate=[args.position_controller])
        signal.signal(signal.SIGINT, prev_sigint)
        signal.signal(signal.SIGTERM, prev_sigterm)
        rclpy.try_shutdown()

    return rc


if __name__ == '__main__':
    sys.exit(main())
