#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import numpy as np

class JointOscillatorNode(Node):
    def __init__(self):
        super().__init__('joint_oscillator_node')

        self.publisher = self.create_publisher(JointState, 'command_joint_states', 10)
        self.subscription = self.create_subscription(
            Float32,
            '/scale_factor',
            self.scale_factor_callback,
            10
        )

        # Define min and max angles for each joint
        self.min_angles = np.array([0.0] * 16)  # Minimum angles (default to 0 degrees for static joints)
        self.max_angles = np.array([60.0] * 16)  # Maximum angles (default to 45 degrees for oscillating joints)
        self.max_angles[[1, 5, 9]] = 0.0

        self.max_angles[[2, 6, 10]] = 10
        self.max_angles[[3, 7, 11]] = 10

        self.min_angles[12] = 90.0
        self.max_angles[12] = 90.0

        self.min_angles[13] = -90.0
        self.max_angles[13] = -90.0

        self.min_angles[14] = -70
        self.max_angles[14] = -0

        self.max_angles[[0, 4, 8]] = 90.0

        # Initialize scale factor
        self.scale_factor = 0.0

        self.joint_angles = self.min_angles.copy()  # Start with minimum angles

    def scale_factor_callback(self, msg):
        # Callback function for receiving scale factor from /scale_factor topic
        self.scale_factor = msg.data

    def publish_joint_angles(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Calculate joint angles based on scale_factor
        for idx in range(16):
            angle = self.min_angles[idx] + (self.max_angles[idx] - self.min_angles[idx]) * self.scale_factor
            self.joint_angles[idx] = angle

        joint_state_msg.position = self.joint_angles
        self.publisher.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointOscillatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
