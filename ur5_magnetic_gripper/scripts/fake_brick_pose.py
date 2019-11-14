#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import numpy as np
import PyKDL #Vector, Rotation, Quaternion, Frame

from mbzirc_comm_objs.srv import Magnetize, MagnetizeResponse
from mbzirc_comm_objs.msg import GripperAttached
from ur_msgs.srv import SetIO, SetIORequest
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String

from math import sqrt, pow

class node():

    def __init__(self):

        #self.set_io = rospy.ServiceProxy('ur_driver/set_io', SetIO)
        rospy.Service('magnetize', Magnetize, self.magnetize_cb)

        self.pub = rospy.Publisher('attached', GripperAttached, queue_size=1)
        self.pub2 = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=1)
        self.sub = rospy.Subscriber('/wrench', WrenchStamped, self.wrench_cb)

        self.attached = False
        self.threshold = 25  #N



    def get_link_pose_cb(self,req):


def main():

    rospy.init_node('ur5_magnetic_gripper')
    n = node()
    rospy.spin()

if __name__ == '__main__':
    main()
