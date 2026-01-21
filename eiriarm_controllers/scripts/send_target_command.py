#!/usr/bin/env python3
"""
Example script to send target commands to JointImpedanceController.

This demonstrates how to publish target joint positions to the impedance controller.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class TargetCommandPublisher(Node):
    def __init__(self):
        super().__init__('target_command_publisher')
        
        # Publisher to send target commands
        self.publisher = self.create_publisher(
            JointState,
            '/joint_impedance_controller/target_joint_states',
            10
        )
        
        # Timer to send periodic commands (optional)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.counter = 0
        
        self.get_logger().info('Target Command Publisher started')
        self.get_logger().info('Publishing to: /joint_impedance_controller/target_joint_states')
    
    def send_target(self, joint_names, positions, velocities=None):
        """
        Send target command to the impedance controller.
        
        Args:
            joint_names: List of joint names
            positions: List of target positions (rad)
            velocities: List of target velocities (rad/s), optional
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = positions
        
        if velocities is None:
            msg.velocity = [0.0] * len(joint_names)
        else:
            msg.velocity = velocities
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published target: {positions}')
    
    def timer_callback(self):
        """
        Example: Send sinusoidal trajectory for demonstration.
        Remove or modify this for your actual use case.
        """
        # Example: Sinusoidal motion for left_joint_0
        t = self.counter * 0.1  # time in seconds
        amplitude = 0.3  # rad (~17 degrees)
        frequency = 0.5  # Hz
        
        position = amplitude * math.sin(2 * math.pi * frequency * t)
        velocity = amplitude * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)
        
        # Define which joints to control
        joint_names = ['left_joint_0']
        positions = [position]
        velocities = [velocity]
        
        self.send_target(joint_names, positions, velocities)
        
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    
    node = TargetCommandPublisher()
    
    try:
        # Option 1: Send a single command
        # node.send_target(
        #     joint_names=['left_joint_0', 'left_joint_1'],
        #     positions=[0.5, -0.3],
        #     velocities=[0.0, 0.0]
        # )
        
        # Option 2: Spin and send periodic commands (defined in timer_callback)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
