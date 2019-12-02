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

class Pick():

    def __init__(self):


        rospy.wait_for_service('/gazebo/get_link_state')
        self.get_link_client= rospy.ServiceProxy('/gazebo/get_link_state',
                                            gazebo_msgs.srv.GetLinkState)
        rospy.loginfo("Found service 'gazebo/get_link_state'")


        rospy.Subscriber("~pick_target_brick", std_msgs.msg.String, self.execute_cb)

        self.target_pose_pub = rospy.Publisher("/armIKpipeline/target_pose", geometry_msgs.msg.PoseStamped, queue_size=1)



        # subscriber para o get_link_state do gazebo

        # publisher para o IKarm_pose...
        # pubishr para os 2 event_in

        # publisher ara o attach com o nome do brick
        # publisher para o detach com o nome do brick


    def execute_cb(self,msg):

        self.target_brick = msg.data
        print self.target_brick

        # req = gazebo_msgs.msg.LinkState()
        # req.link_name= self.target_brick +"_link"
        # req.reference_frame = "wrist_3_link"

        resp = self.get_link_client(self.target_brick +"_link", "mbzirc2020_0")
        print resp.link_state.pose

        target_pose=geometry_msgs.msg.PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "base_link"
        target_pose.pose=resp.link_state.pose

        target_pose.pose.position.z=0.3

        quat=target_pose.pose.orientation

        x_rot=0.5
        y_rot=-0.5
        z_rot=0.5
        w_rot=-0.5

        # target_pose.pose.orientation.w = quat.w*w_rot - quat.x*x_rot - quat.y*y_rot - quat.z*z_rot
        # target_pose.pose.orientation.x = quat.w*x_rot + quat.x*w_rot + quat.y*z_rot - quat.z*y_rot
        # target_pose.pose.orientation.y = quat.w*y_rot - quat.x*z_rot + quat.y*w_rot + quat.z*x_rot
        # target_pose.pose.orientation.z = quat.w*z_rot + quat.x*y_rot - quat.y*x_rot + quat.z*w_rot

        
        target_pose.pose.orientation.x = -0.525169741105
        target_pose.pose.orientation.y = 0.473493683723
        target_pose.pose.orientation.z = -0.473494469766
        target_pose.pose.orientation.w = 0.525169935928

      #    x: -0.525169741105
      # y: 0.473493683723
      # z: -0.473494469766
      # w: 0.525169935928









        self.target_pose_pub.publish(target_pose)






def main():

    rospy.init_node('pick_node')
    n = Pick()
    #n.execute()

    rospy.spin()

if __name__ == '__main__':
    main()
