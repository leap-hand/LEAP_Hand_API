#!/usr/bin/env python3

import os
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from leap_hand.srv import LeapPosition

class Telekinesis(Node):
    def __init__(self):
        super().__init__('telekinesis_node')
        
        # Wait for the service to be available
        self.cli = self.create_client(LeapPosition, 'leap_position')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service leap_position not available, waiting again...')
        
        self.req = LeapPosition.Request()
        self.pub_hand = self.create_publisher(JointState, 'leaphand_node/cmd_ones', 10) 
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)  # 30Hz

    def timer_callback(self):
        # Call the service to get the current position
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            curr_pos = np.array(future.result().position)
            self.get_logger().info(f'Current Position: {curr_pos}')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

        print(curr_pos)
        # Set the position of the hand when you're done
        stater = JointState()
        stater.position = np.zeros(16)
        self.pub_hand.publish(stater)  # Choose the right embodiment here

def main(args=None):
    rclpy.init(args=args)
    telekinesis_node = Telekinesis()
    rclpy.spin(telekinesis_node)
    telekinesis_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
