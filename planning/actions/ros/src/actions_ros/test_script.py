import roslib
import rospy
import actionlib
import moveit_commander
import sys
import tf
from moveit_commander.exception import MoveItCommanderException
import moveit_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi
from std_msgs.msg import String, Bool
from pick_RGB_class import PickRGB
from place_algorithm_class import Place
from laser_allign_class import Allign_Pick, Allign_Place
from geometry_msgs.msg import PoseStamped
# from laser_brick_detection import Allign


# main class CRIAR PLACE ALGORITHM CLASS E PICK RGB CLASS
class Pick_Place:

  def __init__(self, side, wall_brick_color, level, over, brick2place_color, current_wall_len):
        #members
    rospy.init_node('pick_and_place', anonymous=True)
    # self.brick2place_color = brick2place_color
    # self.over = over
    # self.side = side
    # self.wall_brick_color = wall_brick_color
    # self.level = level
    # self.current_wall_len = current_wall_len
    self.pick_class = PickRGB()
    self.allign_pick=Allign_Pick(brick2place_color)
    self.allign_place=Allign_Place(brick2place_color, current_wall_len, level, over)
    self.place_class = Place(side, wall_brick_color, level, over, brick2place_color)
    self.rest = [-1.5535, -2.6237, 2.382, -1.4844, -1.55, -1.949]
    self.t = tf.TransformListener() 

# def pile_frame_calculation(brick_to_pick_color):
#   #MUDAR ORIENTATION NO CREATE
#   if (brick_to_pick_color == "orange"):
#     goal_pose = self.create_pose(0.9, -1.5, 0, "/pileframe")
#   elif (brick_to_pick_color == "blue"):
#     goal_pose = self.create_pose(2.9, -1.5, 0, "/pileframe")
#   elif (brick_to_pick_color == "green"):
#     goal_pose = self.create_pose(4.65, -1.5, 0, "/pileframe")
#   elif (brick_to_pick_color == "red"):
#     goal_pose = self.create_pose(6.55, -1.5, 0, "/pileframe")
#   else:
#     return
#   return goal_pose

  def movebase_goal(self, x, y, yaw, frame): #yaw in degree
    pose = MoveBaseGoal()
    pose.target_pose.pose.position.x = x
    pose.target_pose.pose.position.y = y
    pose.target_pose.pose.position.z = 0
    yaw = yaw * pi/180
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    pose.target_pose.pose.orientation.x = quaternion[0]
    pose.target_pose.pose.orientation.y = quaternion[1]
    pose.target_pose.pose.orientation.z = quaternion[2]
    pose.target_pose.pose.orientation.w = quaternion[3]
    pose.target_pose.header.stamp = rospy.Time(0)
    pose.target_pose.header.frame_id = frame
    return pose

  def movebase_client(self, goal):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
    #main function
  def execute(self):
    
    #self.pick_class.go_to_joint_pose(self.rest)

    # goal = self.movebase_goal(1, 0, 0, "/odom")
    # result = self.movebase_client(goal)
    # if result:
    #   print "move base success"
    # else:
    #   print "move base failed to execute"
    #   exit()

    # # 
    # result = self.allign_pick.execute()
    # if result:
    #   print "allign to pick success"
    # else:
    #   print "allign to pick failed to execute"
    #   exit()


    result = self.pick_class.execute()
    if result:
      print "picking brick success"
    else:
      print "picking brick failed to execute"
      exit()

    # goal = self.movebase_goal(-1, 0, 180, "/odom")
    # result = self.movebase_client(goal)
    # if result:
    #   print "move base success"
    # else:
    #   print "move base failed to execute"
    #   exit()

    # # rospy.sleep(3)
    # result = self.allign_place.execute()
    # if result:
    #  print "allign to place success"
    # else:
    #  print "allign to place failed to execute"
    #  exit()

    
    # result = self.place_class.execute()
    # if result:
    #   print "placing brick success"
    # else:
    #   print "placing brick failed to execute"
    #   exit()

#RESET CLASS

def main(args):
  #(self, side, wall_brick_color, level, over, brick2place_color, current_wall_len
  ic = Pick_Place("right", "blue", 1, False, "green", 1.2)
  try:
    ic.execute()
    #   while not rospy.is_shutdown():
    #     ic.rate.sleep()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
  main(sys.argv)
