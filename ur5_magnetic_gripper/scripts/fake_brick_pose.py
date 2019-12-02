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


from math import sqrt, pow

import gazebo_msgs.srv
import gazebo_ros_link_attacher.srv
import std_msgs.msg
import geometry_msgs.msg

class node():

    def __init__(self):


        rospy.wait_for_service('/link_attacher_node/attach')
        self.link_attacher_client= rospy.ServiceProxy('/link_attacher_node/attach',
                                            gazebo_ros_link_attacher.srv.Attach)
        rospy.loginfo("Found service 'link_attacher'")

        rospy.wait_for_service('/link_attacher_node/detach')
        self.link_detacher_client= rospy.ServiceProxy('/link_attacher_node/detach',
                                            gazebo_ros_link_attacher.srv.Attach)
        rospy.loginfo("Found service 'link_detacher'")

        rospy.wait_for_service('/gazebo/set_link_state')
        self.set_link_client= rospy.ServiceProxy('/gazebo/set_link_state',
                                            gazebo_msgs.srv.SetLinkState)
        rospy.loginfo("Found service 'gazebo/set_link_state'")

        rospy.Subscriber("~attach", std_msgs.msg.String, self.attach_cb)

        rospy.Subscriber("~detach", std_msgs.msg.String, self.detach_cb)



    def attach_cb(self,msg):

        self.target_brick = msg.data
        print self.target_brick


        req = gazebo_msgs.msg.LinkState()
        req.link_name= self.target_brick +"_link"
        req.pose.position.y=0.3
        req.pose.orientation.w=1
        req.reference_frame = "wrist_3_link"

        resp = self.set_link_client(req)
        print resp

        resp2 = self.link_attacher_client("mbzirc2020_0", "wrist_3_link", self.target_brick, self.target_brick + "_link")

    def detach_cb(self,msg):

        self.target_brick = msg.data

        resp2 = self.link_detacher_client("mbzirc2020_0", "wrist_3_link", self.target_brick, self.target_brick + "_link")


def main():

    rospy.init_node('fake_brick_pose_node')
    n = node()
    rospy.spin()

if __name__ == '__main__':
    main()
