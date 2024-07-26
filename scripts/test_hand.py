#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
import numpy as np

class JointOscillatorNode(Node):
    def __init__(self):
        super().__init__('joint_oscillator_node')

        self.publisher = self.create_publisher(JointState, 'command_joint_states', 10)
        
        self.joint_angles = [0.0] * 16  # Initial joint angles

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

        # Frequency of oscillation (in Hz)
        self.frequency = 0.1  # Oscillate once per second

        # Time period of the sine wave (in seconds)
        self.period = 1.0 / self.frequency

        # Create a timer to publish joint angles at 60 Hz
        self.timer = self.create_timer(1.0 / 60.0, self.publish_joint_angles)

        # Start time for the sine wave
        self.start_time = time.time()

    def publish_joint_angles(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Calculate the elapsed time
        elapsed_time = time.time() - self.start_time

        # Calculate the sine wave value based on the elapsed time
        sine_value = 0.5 * (1 + np.sin(2 * np.pi * elapsed_time / self.period))

        # Calculate the angle based on the sine wave value (ranging from min_angle to max_angle)
        for idx in range(16):
            angle = self.min_angles[idx] + (self.max_angles[idx] - self.min_angles[idx]) * sine_value
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
