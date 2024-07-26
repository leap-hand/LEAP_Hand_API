#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')

        # Declare parameters with default values
        self.declare_parameter('hand_name', 'dom')

        # Get parameters from the parameter server
        self.hand_name = self.get_parameter('hand_name').get_parameter_value().string_value

        # Define positions to command
        self.positions_to_command = [
            [-0.176, 4.928, -22.88, 17.688, -1.3199999999999998, -0.264, -13.815999999999999, -0.7919999999999999, -8.888, -4.664, -16.808, 20.151999999999997, 10.208, -10.208, -23.936, 1.408],
            [0.088, 4.752, -23.144, 17.599999999999998, -1.5839999999999999, -0.176, -13.815999999999999, -0.968, 113.52, -9.152, -22.0, -2.552, 82, -85.448, 40.832, -37.488],
            [0.088, 4.752, -23.144, 17.599999999999998, -1.5839999999999999, -0.176, -13.815999999999999, -0.968, 130, -9.152, -22.0, -2.552, 82, -85.448, 50, -37.488],
            [0.176, 4.576, -23.232, 17.688, 115.72, 3.784, -24.288, -0.528, -12.232, -1.3199999999999998, -22.88, 7.4799999999999995, 107.71199999999999, -87.824, 32.208, -33.528],
            [0.176, 4.576, -23.232, 17.688, 130, 3.784, -24.288, -0.528, -12.232, -1.3199999999999998, -22.88, 7.4799999999999995, 107.71199999999999, -87.824, 40, -33.528],
            [120.55999999999999, -15, -25.784, -5.191999999999999, -4.664, -10, -27.456, -0.704, -13.904, -1.408, -28.511999999999997, 7.215999999999999, 130, -84.128, 23.496, -27.895999999999997],
            [130, -30, -25.784, -5.191999999999999, -4.664, -5.72, -27.456, -0.704, -13.904, -1.408, -28.511999999999997, 7.215999999999999, 130, -84.128, 35, -27.895999999999997]
        ]

        # Initialize index for current position
        self.current_pose_index = 0

        # Interpolation parameters
        self.rate = 100  # Hz
        self.num_interpolations = int(3 * self.rate)  # 5 seconds at 60 Hz
        self.interpolation_index = 0
        self.interpolate_positions()

        # Create publisher for joint states
        self.pub_joint_states = self.create_publisher(JointState, '/' + self.hand_name + '/command_joint_states', 10)

        # Create timer to publish joint states at 60 Hz
        self.timer = self.create_timer(1.0 / self.rate, self.publish_joint_states)

    def interpolate_positions(self):
        start_pos = np.array(self.positions_to_command[self.current_pose_index], dtype=float)
        end_pos = np.array(self.positions_to_command[(self.current_pose_index + 1) % len(self.positions_to_command)], dtype=float)
        self.interpolated_positions = [
            start_pos + t * (end_pos - start_pos) / self.num_interpolations
            for t in range(self.num_interpolations)
        ]

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"joint_{i}" for i in range(16)]
        joint_state_msg.position = [float(pos) for pos in self.interpolated_positions[self.interpolation_index]]
        self.pub_joint_states.publish(joint_state_msg)

        self.interpolation_index += 1
        if self.interpolation_index >= self.num_interpolations:
            self.interpolation_index = 0
            self.current_pose_index = (self.current_pose_index + 1) % len(self.positions_to_command)
            self.interpolate_positions()

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
