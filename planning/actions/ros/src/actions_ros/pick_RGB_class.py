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
from plate_detector_class import plate_detector
from geometry_msgs.msg import PoseStamped
from mbzirc_comm_objs.msg import GripperAttached
from std_srvs.srv import Trigger


class PickRGB:

  def __init__(self):
    #members
    self.gripper_attached = False

    self.pubTarget = rospy.Publisher('/armIKpipeline/target_pose', PoseStamped, queue_size=1)
    self.pub_start_pregrasp = rospy.Publisher('/pregrasp_pipeline_event_in', String, queue_size=1)
    self.pub_start_planned_motion = rospy.Publisher('/pregrasp_pipeline_move_arm_planned_motion/event_in', String, queue_size=1)
    self.group = moveit_commander.MoveGroupCommander("manipulator") 
    self.robot = moveit_commander.RobotCommander()
    self.attach = rospy.Subscriber("/attached", GripperAttached, self.attached_cb,queue_size=10)
    self.detector = plate_detector()
    self.magnetize = rospy.ServiceProxy('actuators_system/magnetize_gripper', Trigger)
    self.demagnetize = rospy.ServiceProxy('actuators_system/demagnetize_gripper', Trigger)

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

  def attached_cb(self, msg):
    self.gripper_attached = msg.attached

  def execute(self):
    self.magnetize()
    top_view = [-1.5938509146319788, -1.0618065160563965, 0.22519189516176397, -0.2647755903056641, -1.6169903914081019, 0]

    if not self.go_to_joint_pose(top_view):
      print "could not move arm to top view position"
      return 
    rospy.sleep(3)
    
    target = self.detector.execute()
    for i in range(3):
      if target:
        break
      print "plate detector did not find a result. Trying again"
      target = self.detector.execute()
    if not target:
      print "Aborting plate detector"
      return
    rospy.sleep(2)
    self.pubTarget.publish(target)

    rospy.sleep(0.5)
    self.pub_start_pregrasp.publish('e_start')
    rospy.sleep(0.5)
    self.pub_start_planned_motion.publish('e_start')
    rospy.sleep(3)
    self.gripper_attached = False

    while self.gripper_attached == False:
      if not self.arm_vertical(-0.008):
        print "could not move arm downwards"
        return
      rospy.sleep(0.8)
    rospy.sleep(1)

    if not self.arm_vertical(0.05):
      print "could not move arm upwards"

    if not self.go_to_joint_pose(top_view):
      print "could not move arm to top view position"
      return 
    rospy.sleep(3)
    return 'success'