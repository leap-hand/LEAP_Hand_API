#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')

        # Define positions to command
        self.positions_to_command = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [-4.928, 4.136, -26.664, 17.776, -1.936, 0.0, -22.352, -1.408, 0.616, 1.144, -12.584, -4.664, 81.312, -27.368, 65.736, 55.0],
            [99.528, 3.168, 48.4, 35.904, -2.376, 0.0, -22.352, -1.408, 0.792, 1.144, -12.584, -4.664, 82.984, -27.368, 68.992, 55.176],
            [100.056, 3.344, 48.136, 35.904, 106.744, -4.576, 17.6, 38.72, 3.432, 1.232, -12.584, -4.664, 83.688, -27.192, 69.344, 53.064],
            [99.968, 3.256, 48.224, 35.904, 106.656, -4.664, 17.6, 38.72, 104.632, -10.56, 39.072, 35.024, 83.688, -27.192, 69.256, 53.064],
            [95.744, 3.256, 48.4, 35.904, 21.56, 20.152, -17.072, -10.384, 21.472, -19.36, -24.376, 22.088, 83.688, -27.72, 69.256, 53.328],
            [8.184, 3.696, -18.568, 3.168, 95.04, -6.512, -3.344, -20.592, 15.136, -18.128, -24.464, 22.088, 98.384, -78.496, 38.632, -14.96],
            [70.752, -20.328, 24.728, 3.432, 12.144, -6.424, -4.048, -20.592, 14.608, -18.128, -24.464, 22.0, 111.672, -83.776, 40.216, -22.792],
            [4.752, 6.336, -26.664, 3.168, 13.816, -5.72, -26.84, -20.592, 83.952, -13.552, 5.368, 10.12, 82.456, -102.08, 44.616, -30.096]
        ]

        # Initialize index for current position
        self.current_pose_index = 0

        # Create publisher for joint states
        self.pub_joint_states = self.create_publisher(JointState, 'command_joint_states', 10)

        # Create timer to publish joint states every 5 seconds
        self.timer = self.create_timer(1.0, self.publish_joint_states)

        self.get_logger().info('Dynamixel control node ready')

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f"joint_{i}" for i in range(16)]
        joint_state_msg.position = self.positions_to_command[self.current_pose_index]
        self.pub_joint_states.publish(joint_state_msg)

        self.current_pose_index = (self.current_pose_index + 1) % len(self.positions_to_command)

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
