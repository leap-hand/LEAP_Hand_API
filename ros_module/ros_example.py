#!/usr/bin/env python3
import os
import sys
import time
import numpy as np
import rospy

from sensor_msgs.msg import JointState

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import *

# This is example code, it reads the position from LEAP Hand and commands it
# Be sure to query the services only when you need it
# This way you don't clog up the communication lines and you get the newest data possible
# If you are looking for position and velocity or pos, vel and curr, I recommend you used the combined services instead of calling them individually, its faster.
class Telekinesis:
    def __init__(self):        
        rospy.wait_for_service('/leap_position')
        self.leap_position = rospy.ServiceProxy('/leap_position', leap_position)
        #self.leap_velocity = rospy.ServiceProxy('/leap_velocity', leap_velocity)
        #self.leap_effort = rospy.ServiceProxy('/leap_effort', leap_effort)
        self.pub_hand = rospy.Publisher("/leaphand_node/cmd_ones", JointState, queue_size = 3) 
        r = rospy.Rate(30)   #30hz rate
        while not rospy.is_shutdown():
            r.sleep()
            ###only run these services if you need them.
            curr_pos = np.array(self.leap_position().position)
            print(curr_pos)
            ##If you want multiple, for instance position, velocity and current, please use the combined services (pos_vel_cur etc.) because they are faster on the motors to query.
            ###This is a fresh position, now do some policy stuff here etc.
            
            #Set the position of the hand when you're done
            stater = JointState()
            stater.position = np.zeros(16)
            self.pub_hand.publish(stater)  ##choose the right embodiment here
if __name__ == "__main__":
    rospy.init_node("ros_example")
    telekinesis_node = Telekinesis()
