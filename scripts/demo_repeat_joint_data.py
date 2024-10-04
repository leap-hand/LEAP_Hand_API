#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class DynamixelControlNode(Node):
    def __init__(self):
        super().__init__('dynamixel_control_node')

        # Declare parameters with default values
        self.declare_parameter('hand_name', 'dom')

        # Declare the hand_side parameter with a default value
        self.declare_parameter('hand_side', 'right')  # Default to 'right' if not set

        # Get parameters from the parameter server
        self.hand_name = self.get_parameter('hand_name').get_parameter_value().string_value

        # Get hand_side parameter from the parameter server
        self.hand_side = self.get_parameter('hand_side').get_parameter_value().string_value

        # Define positions to command
        # New leaphand  
        self.positions_to_command = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.144, 0.616, -12.584, -4.664, 0.0, 1.936, -22.352, -1.408, 4.136, -4.928, -26.664, 17.776, 81.312, -27.368, 65.736, 55.0],
            [1.144, 0.792, -12.584, -4.664, 0.0, -2.376, -22.352, -1.408, 3.168, 99.528, 48.4, 35.904, 82.984, -27.368, 68.992, 55.176],
            [1.232, 3.432, -12.584, -4.664, -4.576, 106.744, 17.6, 38.72, 3.344, 100.056, 48.136, 35.904, 83.688, -27.192, 69.344, 53.064],
            [-10.56, 104.632, 39.072, 35.024, -4.664, 106.656, 17.6, 38.72, 3.256, 99.968, 48.224, 35.904, 83.688, -27.192, 69.256, 53.064],
            [-19.36, 21.472, -24.376, 22.088, 20.152, 21.56, -17.072, -10.384, 3.256, 95.744, 48.4, 35.904, 83.688, -27.72, 69.256, 53.328],
            [-18.128, 15.136, -24.464, 22.088, -6.512, 95.04, -3.344, -20.592, 3.696, 8.184, -18.568, 3.168, 98.384, -78.496, 38.632, -14.96],
            [-18.128, 14.608, -24.464, 22.0, -6.424, 12.144, -4.048, -20.592, -20.328, 70.752, 24.728, 3.432, 111.672, -83.776, 40.216, -22.792],
            [-13.552, 83.952, 5.368, 10.12, -5.72, 13.816, -26.84, -20.592, 6.336, 4.752, -26.664, 3.168, 82.456, -102.08, 44.616, -30.096]
        ]
        
        # Adjust positions based on hand side
        if self.hand_side == 'left':
            for i in range(len(self.positions_to_command)):
                self.positions_to_command[i][0] = -self.positions_to_command[i][0]  # Index 0
                self.positions_to_command[i][4] = -self.positions_to_command[i][4]  # Index 4
                self.positions_to_command[i][8] = -self.positions_to_command[i][8]  # Index 4
                self.positions_to_command[i][12] = -self.positions_to_command[i][12]  # Index 12
                self.positions_to_command[i][13] = -self.positions_to_command[i][13]  # Index 13

        # Initialize index for current position
        self.current_pose_index = 0

        # Create publisher for joint states
        self.pub_joint_states = self.create_publisher(JointState, '/' + self.hand_name + '/command_joint_states', 10)

        # Create timer to publish joint states every 5 seconds
        self.timer = self.create_timer(0.5, self.publish_joint_states)

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
