
import sys
import rospy
import numpy as np
import tf

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovariance
from std_msgs.msg import String
from mbzirc_comm_objs.msg import Object, ObjectList


class object_listener:

  def __init__(self):

    rospy.init_node('object_listener', anonymous=True)
    self.L_wall = PoseWithCovariance()
    self.red_pile = PoseWithCovariance()
    self.green_pile = PoseWithCovariance()
    self.blue_pile = PoseWithCovariance()
    self.orange_pile = PoseWithCovariance()
    self.tf_pub = tf.TransformBroadcaster()
    self.control = [False,False,False,False,False]
    self.object_sub = rospy.Subscriber('/estimated_objects', ObjectList, self.callback_objects, queue_size=1)
    rospy.wait_for_message("/estimated_objects", Image)

  def callback_objects(self,data):
    for item in data.objects:
      if (item.type == 1) and (item.sub_type == 1):
        self.L_wall = item
        self.control[0] = True
      if (item.type == 0) and (item.sub_type == 1) and (pose.pose.position.x < 20):
        if item.color ==  1:
          self.red_pile = item
          self.control[0] = True
        elif item.color ==  2:
          self.green_pile = item
          self.control[0] = True
        elif item.color ==  3:
          self.blue_pile = item
          self.control[0] = True
        elif item.color ==  4:
          self.orange_pile = item
          self.control[0] = True
        else:
          pass

  def pub_tf(self, cov_pose, frame_id):
    self.tf_pub.sendTransform((cov_pose.pose.pose.position.x, cov_pose.pose.pose.position.y, cov_pose.pose.pose.position.z),
                     (cov_pose.pose.pose.orientation.x, cov_pose.pose.pose.orientation.y, cov_pose.pose.pose.orientation.z, cov_pose.pose.pose.orientation.w),
                     rospy.Time.now(),
                     frame_id,
                     cov_pose.header)

  def execute(self):
    if self.control[0]:
      pub_tf(self.L_wall, "/L_frame")
    if self.control[1]:
      pub_tf(self.red_pile, "/red_pile")
    if self.control[2]:
      pub_tf(self.green_pile, "/green_pile")
    if self.control[3]:
      pub_tf(self.blue_pile,  "/blue_pile")
    if self.control[4]:
      pub_tf(self.orange_pile, "/orange_pile")

    for x in self.control:
      if x == False:
        return False
    return True
