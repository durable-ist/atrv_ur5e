import roslib
import rospy
import actionlib
import moveit_commander
import sys
import tf
from moveit_commander.exception import MoveItCommanderException
import moveit_msgs
from math import pi
from std_msgs.msg import String, Bool
from orange import edge_detector_orange
from geometry_msgs.msg import PoseStamped, WrenchStamped
from mbzirc_comm_objs.srv import Magnetize
from std_srvs.srv import Trigger


# main class
class Place_first:

  def __init__(self):
        #members
    self.brick2place_color = "red"
    self.t = tf.TransformListener() 
    self.first = edge_detector_orange("right", "red", 1)

    self.pubTarget = rospy.Publisher('armIKpipeline/target_pose', PoseStamped, queue_size=1)
    self.pub_start_pregrasp = rospy.Publisher('pregrasp_pipeline_event_in', String, queue_size=1)
    self.pub_start_planned_motion = rospy.Publisher('pregrasp_pipeline_move_arm_planned_motion/event_in', String, queue_size=1)
    self.sub_wrench = rospy.Subscriber('wrench', WrenchStamped, self.wrench_cb, queue_size=1)
    self.attach = rospy.Subscriber("attached", GripperAttached, self.attached_cb,queue_size=10)
    self.magnetize = rospy.ServiceProxy('actuators_system/magnetize_gripper', Trigger)
    self.demagnetize = rospy.ServiceProxy('actuators_system/demagnetize_gripper', Trigger)
    self.group = moveit_commander.MoveGroupCommander("manipulator") 
    self.robot = moveit_commander.RobotCommander()

  def wrench_cb(self, data):
    self.wrench = data

  def go_to_joint_pose(self, arm_configuration):
    try:
      self.group.set_joint_value_target(arm_configuration)
      self.group.go(wait=True)
      return "success"
    except MoveItCommanderException, e:
      print 'can not set goal pose'
      print e
      return 

  def arm_vertical(self, inc):
    try:
      self.group.shift_pose_target(2, inc)
      self.group.go(wait=True)
      return "success"      
    except MoveItCommanderException, e:
      print 'can not set move vertically pose'
      print e
      return

  def create_pose(self, x, y, z, frame):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(pi/2, pi, pi/2)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = frame
    return pose

  def compute_place_pose(self, edge_point): #receives edge point in base_link
    if self.brick2place_color == "red":
      offset = 0.15
    elif (self.brick2place_color == "green"):
      offset = 0.3
    elif (self.brick2place_color == "blue"):
      offset = 0.6
    elif (self.brick2place_color == "orange"):
      offset = 0.9
    else:
      print "wrong color code"
      return
    y = edge_point.point.y - offset
    height = 0.2 
    return self.create_pose(edge_point.point.x, y, height, "/base_link")

  def execute(self):

    top_view = [-1.5938509146319788, -1.0618065160563965, 0.22519189516176397, -0.2647755903056641, -1.6169903914081019, 0]

    if not self.go_to_joint_pose(top_view):
      print "could not move arm to top view position"
      return 
    rospy.sleep(1)
    
    lado = self.edge.execute()
    for i in range(3):
      if lado:
        break
      print "edge detector did not find a result. Trying again"
      lado = self.edge.execute()
    if not lado:
      print "Aborting edge detector"
      return

    try:
      lado = self.t.transformPoint("/base_link", lado)
    except:
      print "could not transform lado"
      return

    target = self.compute_place_pose(lado)
    if not target:
      print "Could not compute place pose"
      return

    target.pose.position.z += 0.2 
    self.pubTarget.publish(target)
    rospy.sleep(0.5)
    self.pub_start_pregrasp.publish('e_start')
    rospy.sleep(0.5)
    self.pub_start_planned_motion.publish('e_start')
    rospy.sleep(4)

    if not self.arm_vertical(-0.2):
      print "could not move arm downwards"

    try:
      wrench = self.wrench.wrench.force.z
    except:
      print "wrench topic is not publishing"
      return

    while abs(self.wrench.wrench.force.z) < 1.2*abs(wrench): 
      if not self.arm_vertical(-0.008):
        print "could not move arm downwards"
      rospy.sleep(0.4)
    rospy.sleep(1)
    result = True
    while self.gripper_attached == True:
      self.demagnetize()
      rospy.sleep(0.2)

    if not self.arm_vertical(0.2):
      print "could not move arm upwards"
    rospy.sleep(1)

    return 'success'
