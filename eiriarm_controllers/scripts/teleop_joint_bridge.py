#!/usr/bin/env python3
"""UDP joint teleoperation bridge for two isolated ROS 2 hosts.

Each host keeps its local ROS graph private and exchanges only compact UDP
joint-state packets with the peer. Local control still uses the existing
ros2_control stack:

  * /joint_states for local measured positions
  * /joint_position_command for one-point JointTrajectory commands
  * /controller_manager/* for controller switching

Modes:
  no_feedback:
    master runs gravity compensation after alignment and only streams its
    measured joint positions. Slave tracks the master via joint_position.

  force_feedback:
    master and slave both keep joint_position active and track each other's
    measured joint positions. This is position-coupled kinesthetic feedback,
    not true force/torque sensor feedback.
"""

import argparse
import json
import math
import signal
import socket
import sys
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


PARAM_STRING_ARRAY = 9

DEFAULT_JOINTS = [
    'left_joint_0',
    'left_joint_1',
    'left_joint_2',
    'left_joint_3',
    'left_joint_4',
    'left_joint_5',
    'left_joint_6',
    'right_joint_0',
    'right_joint_1',
    'right_joint_2',
    'right_joint_3',
    'right_joint_4',
    'right_joint_5',
    'right_joint_6',
]


def _duration_msg(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


def _service_path(manager: str, service_name: str) -> str:
    return manager.rstrip('/') + '/' + service_name


def _max_abs_delta(a: List[float], b: List[float]) -> float:
    if len(a) != len(b):
        return math.inf
    return max((abs(x - y) for x, y in zip(a, b)), default=0.0)


class TeleopJointBridge(Node):
    def __init__(self, args):
        super().__init__('teleop_joint_bridge')
        self.args = args
        self.role = args.role
        self.mode = args.mode
        self.peer_role = 'slave' if self.role == 'master' else 'master'
        self.period = 1.0 / args.rate_hz
        self.cb_group = ReentrantCallbackGroup()
        self.executor_ready = False

        self.joint_names = list(args.joints) if args.joints else []
        if not self.joint_names:
            self.joint_names = self._query_controller_joints()
        if not self.joint_names:
            self.joint_names = list(DEFAULT_JOINTS)
            self.get_logger().warn(
                'Could not query joint_position_controller joints; using '
                'built-in dual-arm joint list')

        self._lock = threading.Lock()
        self.local_positions: Optional[List[float]] = None
        self.remote_positions: Optional[List[float]] = None
        self.last_rx_time = 0.0
        self.remote_aligned = False
        self.remote_enabled = False
        self.peer_enabled_seen = False
        self.remote_seq = -1
        self.seq = 0
        self.aligned = False
        self.enabled = False
        self.last_commanded: Optional[List[float]] = None
        self.force_deadband_master = self._resolve_joint_values(
            args.force_deadband_master_joints,
            args.force_deadband_master,
            'force_deadband_master_joints')
        self.force_deadband_slave = self._resolve_joint_values(
            args.force_deadband_slave_joints,
            args.force_deadband_slave,
            'force_deadband_slave_joints')
        self.force_feedback_active = [False] * len(self.joint_names)
        self._last_timeout_log = 0.0

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.joint_sub = self.create_subscription(
            JointState,
            args.joint_state_topic,
            self._on_joint_state,
            qos,
            callback_group=self.cb_group)
        self.command_pub = self.create_publisher(
            JointTrajectory, args.command_topic, 10)

        self.align_srv = self.create_service(
            Trigger, '/teleop/align', self._on_align,
            callback_group=self.cb_group)
        self.enable_srv = self.create_service(
            Trigger, '/teleop/enable', self._on_enable,
            callback_group=self.cb_group)
        self.disable_srv = self.create_service(
            Trigger, '/teleop/disable', self._on_disable,
            callback_group=self.cb_group)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((args.bind_host, args.local_port))
        self.sock.settimeout(0.1)
        self._stop_event = threading.Event()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

        self.timer = self.create_timer(
            self.period, self._on_timer, callback_group=self.cb_group)

        self.get_logger().info(
            f"teleop ready role={self.role} mode={self.mode} "
            f"local={args.bind_host}:{args.local_port} "
            f"peer={args.peer_host}:{args.peer_port} "
            f"joints={len(self.joint_names)}")

    # ------------------------------------------------------------------
    # ROS state and controller helpers
    # ------------------------------------------------------------------
    def _resolve_joint_values(self, raw: str, fallback: float, name: str) -> List[float]:
        raw = (raw or '').strip()
        if not raw:
            return [float(fallback)] * len(self.joint_names)
        values = [float(x) for x in raw.replace(',', ' ').split()]
        if len(values) == 1:
            values = values * len(self.joint_names)
        if len(values) != len(self.joint_names):
            raise ValueError(
                f'{name} must contain 1 or {len(self.joint_names)} values; '
                f'got {len(values)}')
        if any(v < 0.0 for v in values):
            raise ValueError(f'{name} values must be >= 0')
        return values

    def _call_service(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        if self.executor_ready:
            done = threading.Event()
            fut.add_done_callback(lambda _fut: done.set())
            if not done.wait(timeout):
                return None
        else:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def _query_controller_joints(self) -> Optional[List[str]]:
        srv = f'/{self.args.position_controller}/get_parameters'
        client = self.create_client(
            GetParameters, srv, callback_group=self.cb_group)
        req = GetParameters.Request()
        req.names = ['joints']
        resp = self._call_service(client, req, timeout=3.0)
        self.destroy_client(client)
        if resp is None or not resp.values:
            return None
        value = resp.values[0]
        if value.type != PARAM_STRING_ARRAY:
            return None
        return list(value.string_array_value)

    def _controller_is_active(self, name: str) -> bool:
        client = self.create_client(
            ListControllers,
            _service_path(self.args.controller_manager, 'list_controllers'),
            callback_group=self.cb_group)
        try:
            resp = self._call_service(
                client, ListControllers.Request(), timeout=3.0)
            if resp is None:
                return False
            for controller in resp.controller:
                if controller.name == name:
                    return controller.state == 'active'
            return False
        finally:
            self.destroy_client(client)

    def _switch_controllers(self, activate=None, deactivate=None) -> bool:
        activate = list(activate or [])
        deactivate = list(deactivate or [])
        if not activate and not deactivate:
            return True

        client = self.create_client(
            SwitchController,
            _service_path(self.args.controller_manager, 'switch_controller'),
            callback_group=self.cb_group)
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = False
        resp = self._call_service(client, req, timeout=10.0)
        self.destroy_client(client)
        if resp is None:
            self.get_logger().error('switch_controller did not respond')
            return False
        if not resp.ok:
            self.get_logger().error(
                f"switch_controller returned ok=False "
                f"(activate={activate}, deactivate={deactivate})")
            return False
        self.get_logger().info(f"controllers switched (+{activate} -{deactivate})")
        return True

    def _ensure_position_controller(self) -> bool:
        activate = []
        if not self._controller_is_active(self.args.position_controller):
            activate.append(self.args.position_controller)
        if not self._controller_is_active(self.args.gravity_controller):
            activate.append(self.args.gravity_controller)

        deactivate = []
        if self._controller_is_active(self.args.cartesian_controller):
            deactivate.append(self.args.cartesian_controller)
        return self._switch_controllers(activate, deactivate)

    def _ensure_gravity_controller(self) -> bool:
        activate = []
        if not self._controller_is_active(self.args.gravity_controller):
            activate.append(self.args.gravity_controller)

        deactivate = []
        if self._controller_is_active(self.args.position_controller):
            deactivate.append(self.args.position_controller)
        if self._controller_is_active(self.args.cartesian_controller):
            deactivate.append(self.args.cartesian_controller)
        return self._switch_controllers(activate, deactivate)

    def _on_joint_state(self, msg: JointState):
        pos_by_name: Dict[str, float] = {
            name: pos for name, pos in zip(msg.name, msg.position)
        }
        try:
            positions = [float(pos_by_name[name]) for name in self.joint_names]
        except KeyError:
            return
        with self._lock:
            self.local_positions = positions

    # ------------------------------------------------------------------
    # UDP transport
    # ------------------------------------------------------------------
    def _rx_loop(self):
        while not self._stop_event.is_set():
            try:
                data, _addr = self.sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                packet = json.loads(data.decode('utf-8'))
                if packet.get('version') != 1:
                    continue
                if packet.get('role') != self.peer_role:
                    continue
                names = packet.get('joint_names', [])
                positions = packet.get('positions', [])
                if not isinstance(names, list) or not isinstance(positions, list):
                    continue
                if len(names) != len(positions):
                    continue
                pos_by_name = {
                    str(name): float(pos) for name, pos in zip(names, positions)
                }
                mapped = [pos_by_name[name] for name in self.joint_names]
            except (ValueError, KeyError, TypeError, UnicodeDecodeError):
                continue

            with self._lock:
                self.remote_positions = mapped
                self.last_rx_time = time.monotonic()
                self.remote_aligned = bool(packet.get('aligned', False))
                remote_enabled = bool(packet.get('enabled', False))
                self.remote_enabled = remote_enabled
                if remote_enabled:
                    self.peer_enabled_seen = True
                self.remote_seq = int(packet.get('seq', -1))

    def _send_state(self):
        with self._lock:
            positions = None if self.local_positions is None else list(self.local_positions)
            aligned = self.aligned
            enabled = self.enabled
            self.seq += 1
            seq = self.seq
        if positions is None:
            return

        packet = {
            'version': 1,
            'role': self.role,
            'mode': self.mode,
            'seq': seq,
            'stamp': time.time(),
            'aligned': aligned,
            'enabled': enabled,
            'joint_names': self.joint_names,
            'positions': positions,
        }
        data = json.dumps(packet, separators=(',', ':')).encode('utf-8')
        try:
            self.sock.sendto(data, (self.args.peer_host, self.args.peer_port))
        except OSError as exc:
            self.get_logger().warn(f'UDP send failed: {exc}')

    # ------------------------------------------------------------------
    # Teleop services and command loop
    # ------------------------------------------------------------------
    def _have_recent_remote_locked(self, now: float) -> bool:
        return (
            self.remote_positions is not None and
            now - self.last_rx_time <= self.args.timeout
        )

    def _snapshot(self):
        with self._lock:
            local = None if self.local_positions is None else list(self.local_positions)
            remote = None if self.remote_positions is None else list(self.remote_positions)
            recent = self._have_recent_remote_locked(time.monotonic())
            remote_aligned = self.remote_aligned
            remote_enabled = self.remote_enabled
            peer_enabled_seen = self.peer_enabled_seen
        return local, remote, recent, remote_aligned, remote_enabled, peer_enabled_seen

    def _publish_trajectory(self, positions: List[float], duration: float):
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = _duration_msg(max(0.0, duration))
        traj.points.append(point)
        self.command_pub.publish(traj)
        with self._lock:
            self.last_commanded = list(positions)

    def _on_align(self, _req, resp):
        if self.role != 'master':
            resp.success = False
            resp.message = 'alignment is only allowed on the master host'
            return resp
        local, remote, recent, _remote_aligned, _remote_enabled, _peer_enabled_seen = self._snapshot()
        if local is None:
            resp.success = False
            resp.message = 'no local /joint_states yet'
            return resp
        if remote is None or not recent:
            resp.success = False
            resp.message = 'no recent peer joint state'
            return resp

        self.get_logger().info(
            f'aligning master to slave over {self.args.align_duration:.2f}s')
        if not self._ensure_position_controller():
            resp.success = False
            resp.message = 'failed to switch master to joint_position_controller'
            return resp

        self._publish_trajectory(remote, self.args.align_duration)
        time.sleep(self.args.align_duration + self.args.align_settle)

        if self.mode == 'no_feedback':
            if not self._ensure_gravity_controller():
                resp.success = False
                resp.message = 'aligned, but failed to restore master gravity mode'
                return resp

        with self._lock:
            self.aligned = True
            self.enabled = False
        resp.success = True
        resp.message = 'alignment complete; call /teleop/enable on both hosts'
        return resp

    def _on_enable(self, _req, resp):
        local, remote, recent, remote_aligned, remote_enabled, _peer_enabled_seen = self._snapshot()
        if local is None:
            resp.success = False
            resp.message = 'no local /joint_states yet'
            return resp
        if remote is None or not recent:
            resp.success = False
            resp.message = 'no recent peer joint state'
            return resp
        if self.role == 'master' and not self.aligned:
            resp.success = False
            resp.message = 'run /teleop/align on master before enabling'
            return resp
        if self.role == 'slave' and not remote_aligned:
            resp.success = False
            resp.message = 'master has not reported alignment complete'
            return resp
        error = _max_abs_delta(local, remote)
        if error > self.args.max_start_error:
            resp.success = False
            resp.message = (
                f'local/peer joint error {error:.3f} rad exceeds '
                f'{self.args.max_start_error:.3f} rad')
            return resp

        if self.mode == 'force_feedback':
            if not self._ensure_position_controller():
                resp.success = False
                resp.message = 'failed to switch to joint_position_controller'
                return resp
        elif self.role == 'slave':
            if not self._ensure_position_controller():
                resp.success = False
                resp.message = 'failed to switch slave to joint_position_controller'
                return resp
        else:
            if not self._ensure_gravity_controller():
                resp.success = False
                resp.message = 'failed to switch master to gravity mode'
                return resp

        with self._lock:
            self.enabled = True
            self.last_commanded = list(local)
            self.peer_enabled_seen = remote_enabled
            self.force_feedback_active = [False] * len(self.joint_names)
        resp.success = True
        resp.message = f'teleop enabled role={self.role} mode={self.mode}'
        return resp

    def _on_disable(self, _req, resp):
        local, _remote, _recent, _remote_aligned, _remote_enabled, _peer_enabled_seen = self._snapshot()
        with self._lock:
            self.enabled = False
            self.peer_enabled_seen = False
            self.force_feedback_active = [False] * len(self.joint_names)
        if local is not None and (self.mode == 'force_feedback' or self.role == 'slave'):
            self._publish_trajectory(local, self.args.hold_duration)
        if self.mode == 'no_feedback' and self.role == 'master':
            self._ensure_gravity_controller()
        resp.success = True
        resp.message = 'teleop disabled locally; peer will auto-disable after receiving this state'
        return resp

    def _limited_target(self, target: List[float]) -> List[float]:
        with self._lock:
            current = (
                list(self.last_commanded)
                if self.last_commanded is not None
                else (list(self.local_positions) if self.local_positions is not None else list(target))
            )
        limited = []
        for prev, nxt in zip(current, target):
            delta = nxt - prev
            if delta > self.args.max_step:
                nxt = prev + self.args.max_step
            elif delta < -self.args.max_step:
                nxt = prev - self.args.max_step
            limited.append(nxt)
        return limited

    def _force_feedback_target(self, local: List[float], remote: List[float]) -> List[float]:
        """Role-specific soft deadband and coupling for bilateral teleop."""
        if self.role == 'master':
            deadbands = self.force_deadband_master
            scale = self.args.master_coupling_scale
        else:
            deadbands = self.force_deadband_slave
            scale = self.args.slave_coupling_scale

        target = []
        with self._lock:
            if len(self.force_feedback_active) != len(local):
                self.force_feedback_active = [False] * len(local)
            active = list(self.force_feedback_active)

            for i, (q_local, q_remote) in enumerate(zip(local, remote)):
                deadband = deadbands[i]
                exit_deadband = max(0.0, deadband - self.args.force_deadband_hysteresis)
                error = q_remote - q_local
                abs_error = abs(error)

                if active[i]:
                    if abs_error < exit_deadband:
                        active[i] = False
                elif abs_error > deadband:
                    active[i] = True

                if active[i]:
                    effective_error = max(0.0, abs_error - deadband)
                    direction = 1.0 if error >= 0.0 else -1.0
                    target.append(q_local + direction * scale * effective_error)
                else:
                    target.append(q_local)

            self.force_feedback_active = active
        return target

    def _on_timer(self):
        self._send_state()
        now = time.monotonic()
        with self._lock:
            enabled = self.enabled
            local = None if self.local_positions is None else list(self.local_positions)
            remote = None if self.remote_positions is None else list(self.remote_positions)
            recent = self._have_recent_remote_locked(now)
            remote_enabled = self.remote_enabled
            peer_enabled_seen = self.peer_enabled_seen

        if not enabled:
            return
        if local is None or remote is None:
            return
        if not recent:
            with self._lock:
                self.enabled = False
                self.force_feedback_active = [False] * len(self.joint_names)
            if now - self._last_timeout_log > 1.0:
                self.get_logger().error(
                    f'peer timeout > {self.args.timeout:.2f}s; teleop disabled')
                self._last_timeout_log = now
            if self.mode == 'force_feedback' or self.role == 'slave':
                self._publish_trajectory(local, self.args.hold_duration)
            return

        if peer_enabled_seen and not remote_enabled:
            with self._lock:
                self.enabled = False
                self.peer_enabled_seen = False
                self.force_feedback_active = [False] * len(self.joint_names)
            if self.mode == 'force_feedback' or self.role == 'slave':
                self._publish_trajectory(local, self.args.hold_duration)
            if self.mode == 'no_feedback' and self.role == 'master':
                self._ensure_gravity_controller()
            self.get_logger().warn('peer disabled teleop; local teleop disabled')
            return

        if self.mode == 'no_feedback' and self.role == 'master':
            return

        error = _max_abs_delta(local, remote)
        if error > self.args.max_runtime_error:
            with self._lock:
                self.enabled = False
                self.force_feedback_active = [False] * len(self.joint_names)
            self._publish_trajectory(local, self.args.hold_duration)
            self.get_logger().error(
                f'joint error {error:.3f} rad exceeds runtime limit '
                f'{self.args.max_runtime_error:.3f}; teleop disabled')
            return

        if self.mode == 'force_feedback':
            target = self._force_feedback_target(local, remote)
        else:
            target = remote
        target = self._limited_target(target)
        self._publish_trajectory(target, self.args.command_duration)

    def shutdown(self):
        self._stop_event.set()
        try:
            self.sock.close()
        except OSError:
            pass
        if self.rx_thread.is_alive():
            self.rx_thread.join(timeout=0.5)


def parse_args(argv):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--role', required=True, choices=['master', 'slave'])
    p.add_argument('--mode', default='no_feedback',
                   choices=['no_feedback', 'force_feedback'])
    p.add_argument('--peer-host', required=True,
                   help='peer host IPv4/hostname reachable over Ethernet')
    p.add_argument('--bind-host', default='0.0.0.0',
                   help='local UDP bind host (default 0.0.0.0)')
    p.add_argument('--local-port', type=int, default=15000)
    p.add_argument('--peer-port', type=int, default=15001)
    p.add_argument('--rate-hz', type=float, default=50.0)
    p.add_argument('--timeout', type=float, default=0.3,
                   help='peer timeout seconds before auto-disable')
    p.add_argument('--align-duration', type=float, default=5.0)
    p.add_argument('--align-settle', type=float, default=0.2)
    p.add_argument('--command-duration', type=float, default=0.08,
                   help='time_from_start for streaming setpoints')
    p.add_argument('--hold-duration', type=float, default=0.2)
    p.add_argument('--max-start-error', type=float, default=0.5)
    p.add_argument('--max-runtime-error', type=float, default=1.0)
    p.add_argument('--max-step', type=float, default=0.03,
                   help='max target change per publish cycle, rad')
    p.add_argument('--force-deadband-master', type=float, default=0.015,
                   help='force-feedback master soft deadband, rad')
    p.add_argument('--force-deadband-slave', type=float, default=0.0,
                   help='force-feedback slave soft deadband, rad')
    p.add_argument('--force-deadband-hysteresis', type=float, default=0.004,
                   help='force-feedback deadband hysteresis, rad')
    p.add_argument('--force-deadband-master-joints', default='',
                   help='optional per-joint master deadbands, comma/space separated')
    p.add_argument('--force-deadband-slave-joints', default='',
                   help='optional per-joint slave deadbands, comma/space separated')
    p.add_argument('--master-coupling-scale', type=float, default=0.45,
                   help='force-feedback master coupling scale')
    p.add_argument('--slave-coupling-scale', type=float, default=1.0,
                   help='force-feedback slave coupling scale')
    p.add_argument('--joint-state-topic', default='/joint_states')
    p.add_argument('--command-topic', default='/joint_position_command')
    p.add_argument('--controller-manager', default='/controller_manager')
    p.add_argument('--position-controller', default='joint_position_controller')
    p.add_argument('--gravity-controller',
                   default='gravity_compensation_controller')
    p.add_argument('--cartesian-controller',
                   default='cartesian_position_controller')
    p.add_argument('--joints', nargs='*', default=None,
                   help='optional explicit joint list; default queries controller')
    args = p.parse_args(argv)
    if args.rate_hz <= 0.0:
        p.error('--rate-hz must be > 0')
    if args.timeout <= 0.0:
        p.error('--timeout must be > 0')
    if args.local_port <= 0 or args.peer_port <= 0:
        p.error('--local-port and --peer-port must be positive')
    if args.align_duration <= 0.0:
        p.error('--align-duration must be > 0')
    if args.max_step <= 0.0:
        p.error('--max-step must be > 0')
    if args.force_deadband_master < 0.0 or args.force_deadband_slave < 0.0:
        p.error('--force-deadband-master/slave must be >= 0')
    if args.force_deadband_hysteresis < 0.0:
        p.error('--force-deadband-hysteresis must be >= 0')
    if not (0.0 <= args.master_coupling_scale <= 1.0):
        p.error('--master-coupling-scale must be in [0, 1]')
    if not (0.0 <= args.slave_coupling_scale <= 1.0):
        p.error('--slave-coupling-scale must be in [0, 1]')
    return args


def main():
    argv = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parse_args(argv)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopJointBridge(args)

    stop = {'value': False}

    def on_signal(_signum, _frame):
        stop['value'] = True

    prev_sigint = signal.signal(signal.SIGINT, on_signal)
    prev_sigterm = signal.signal(signal.SIGTERM, on_signal)
    executor = None

    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        node.executor_ready = True
        while rclpy.ok() and not stop['value']:
            executor.spin_once(timeout_sec=0.1)
    finally:
        node.executor_ready = False
        node.shutdown()
        if executor is not None:
            try:
                executor.remove_node(node)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGINT, prev_sigint)
        signal.signal(signal.SIGTERM, prev_sigterm)


if __name__ == '__main__':
    main()
