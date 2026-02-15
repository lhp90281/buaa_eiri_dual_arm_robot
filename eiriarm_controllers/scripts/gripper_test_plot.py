#!/usr/bin/env python3
"""
Gripper open-close test script with hold-dwell observation.

Workflow:
  1. Send "close", wait for hold
  2. Dwell 5s (keep recording position/force to observe hold stability)
  3. Send "open", wait for hold
  4. Dwell 5s
  5. Plot full timeline and auto-exit

Usage:
  python3 gripper_test_plot.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import signal
import time
import re
import sys


class GripperTestNode(Node):
    DWELL_SECONDS = 5.0

    def __init__(self):
        super().__init__('gripper_test_node')

        # Data recording
        self.times = []
        self.positions = []
        self.forces = []
        self.states = []
        self.t0 = None

        # State machine
        self.phase = 'init'
        self.dwell_start = None
        self.done = False

        # Subscriber
        self.status_sub = self.create_subscription(
            String,
            '/gripper_controller/left_gripper/status',
            self.status_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            String,
            '/gripper_controller/left_gripper/command',
            10
        )

        # Wait for connections, then start
        self.start_timer = self.create_timer(1.0, self.start_test)
        self.get_logger().info('Gripper test node started, waiting for connection...')

    def send_cmd(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)

    def start_test(self):
        if self.phase != 'init':
            return
        self.start_timer.cancel()
        self.t0 = time.time()
        self.get_logger().info('=== Sending CLOSE command ===')
        self.phase = 'wait_closing'
        self.send_cmd('close')

    def parse_status(self, data: str):
        state_match = re.search(r'state:(\w+)', data)
        pos_match = re.search(r'pos:([-\d.]+)', data)
        force_match = re.search(r'force:([-\d.]+)', data)
        state = state_match.group(1) if state_match else ''
        pos = float(pos_match.group(1)) if pos_match else 0.0
        force = float(force_match.group(1)) if force_match else 0.0
        return state, pos, force

    def status_callback(self, msg: String):
        if self.phase == 'init' or self.done:
            return

        state, pos, force = self.parse_status(msg.data)
        now = time.time()

        # Always record
        if self.t0 is not None:
            self.times.append(now - self.t0)
            self.positions.append(pos)
            self.forces.append(force)
            self.states.append(state)

        # --- State machine ---
        if self.phase == 'wait_closing':
            if state == 'closing':
                self.get_logger().info('Gripper is closing...')
                self.phase = 'wait_close_hold'

        elif self.phase == 'wait_close_hold':
            if state in ('hold', 'force_hold'):
                self.get_logger().info(
                    f'Close complete (pos={pos:.4f}m, force={force:.2f}N). Dwelling {self.DWELL_SECONDS}s...')
                self.phase = 'dwell_close'
                self.dwell_start = now

        elif self.phase == 'dwell_close':
            if now - self.dwell_start >= self.DWELL_SECONDS:
                self.get_logger().info('=== Sending OPEN command ===')
                self.phase = 'wait_opening'
                self.send_cmd('open')

        elif self.phase == 'wait_opening':
            if state == 'opening':
                self.get_logger().info('Gripper is opening...')
                self.phase = 'wait_open_hold'

        elif self.phase == 'wait_open_hold':
            if state == 'hold':
                self.get_logger().info(
                    f'Open complete (pos={pos:.4f}m). Dwelling {self.DWELL_SECONDS}s...')
                self.phase = 'dwell_open'
                self.dwell_start = now

        elif self.phase == 'dwell_open':
            if now - self.dwell_start >= self.DWELL_SECONDS:
                self.get_logger().info('=== Test complete, generating plot ===')
                self.done = True
                self.plot_results()

    def plot_results(self):
        if not self.times:
            self.get_logger().warn('No data recorded!')
            return

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

        # Position plot
        ax1.plot(self.times, [p * 1000 for p in self.positions], 'b-', linewidth=1.0, label='Position')
        ax1.set_ylabel('Position (mm)')
        ax1.set_title('Gripper Open-Close Test (Left Gripper) — with hold dwell')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Force plot
        ax2.plot(self.times, self.forces, 'r-', linewidth=1.0, label='Force')
        ax2.axhline(y=10.0, color='orange', linestyle='--', alpha=0.7, label='Force threshold')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Force (N)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        # Mark state transitions
        prev_state = ''
        for i, s in enumerate(self.states):
            if s != prev_state and prev_state != '':
                for ax in (ax1, ax2):
                    ax.axvline(x=self.times[i], color='green', linestyle=':', alpha=0.6)
                ax1.annotate(s, (self.times[i], self.positions[i] * 1000),
                             textcoords="offset points", xytext=(5, 10),
                             fontsize=8, color='green')
            prev_state = s

        plt.tight_layout()
        save_path = '/home/arm/ros2_ws/gripper_test_result.png'
        plt.savefig(save_path, dpi=150)
        self.get_logger().info(f'Plot saved to {save_path}')
        plt.show(block=False)
        plt.pause(0.5)


def main():
    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)

    # Handle Ctrl+C ourselves
    shutdown_requested = False
    def _sig_handler(sig, frame):
        nonlocal shutdown_requested
        shutdown_requested = True
    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    node = GripperTestNode()
    try:
        while not shutdown_requested and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
    finally:
        node.get_logger().info('Shutting down.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
