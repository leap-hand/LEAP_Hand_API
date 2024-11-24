#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from leap_hand.srv import LeapPosition, LeapPosVelEff
import time
import numpy as np

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(LeapPosition, '/leap_position')
        self.cli = self.create_client(LeapPosVelEff, '/leap_pos_vel_eff')
        ##Note if you need to read multiple values this is faster than calling each service individually for the motors
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LeapPosVelEff.Request()
        self.pub_hand = self.create_publisher(JointState, '/cmd_ones', 10) 

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    x = list(np.zeros(16))
    y = 0.025
    while True:
        response = minimal_client.send_request()
        print(response)  ##Receive 
        time.sleep(0.05)
        stater = JointState()
        x[0] = x[0] + y
        if x[0] > 0.1:
            y = - 0.025
        if x[0] < -1:
            y = 0.025
        stater.position = x  ##You can set the position this way
        minimal_client.pub_hand.publish(stater)  # Choose the right embodiment here
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()