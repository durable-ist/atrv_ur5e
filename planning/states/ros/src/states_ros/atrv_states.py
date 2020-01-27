#!/usr/bin/env python

import smach
import rospy
import tf
import rospy
from rospy import Duration, Time
from numpy import array
from numpy.linalg import norm as euclidean

from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, PoseStamped, Point

import os

#import navigation actions
import actions_ros.navigation as navigation_file

navigation = navigation_file.Navigation()

fire_topic = '/depth_point'

class ClearCostmaps(smach.State):
    '''
    description: calls service for clearing all costmaps
    input: None
    outcomes: 'success'
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        self.clear_srv = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
    def execute(self, userdata):
        self.clear_srv.call()
        return 'success'

class MoveTo(smach.State):
    '''
    description: mbot will move to the position desired
    input: String/PoseStamped data, data can be a predefined position (string) or can be a pose(PoseStamped)
    outcomes: 'success' or 'failure'
    '''

    def __init__(self, goal, frame='map', timeout=120.0):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['in_data'])
        self.goal = goal
        self.frame = frame
        self.timeout = timeout
      
    def execute(self, userdata):
        if isinstance(self.goal, str):
            if self.goal in navigation.get_available_locations():
                success = navigation.go_to_location(self.goal, frame_id=self.frame, timeout=self.timeout)
            else:
                rospy.logerr("Position is not in the list available locations {}".format(navigation.get_available_locations()))
                rospy.sleep(0.1)
                return 'failure'
        else:
            success = navigation.go_to_pose(self.goal, frame_id=self.frame, timeout=self.timeout)

        if success:
            rospy.loginfo("Robot moved to the desired position with success")
            return 'success'
        else:
            rospy.loginfo("Server responded with error, moving to the position desired")
            return 'failure'
        
class SwitchLocalization(smach.State):
    '''
    description: robot will switch to either indoors or outdoors localization
    input: String, that says either indoors or outdoors
    outcomes: 'success' or 'failure'
    '''

    def __init__(self, env, location='ENTRANCE_DOOR_1'):
        smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['in_data'])
        self.env = env
        self.location = location
    
    def execute(self, userdata):
        if not isinstance(self.env, str):
            rospy.logerr("Switch location state needs a string as input as either outdoors or indoors")
            rospy.sleep(0.1)
            return 'failure'
        elif self.env not in ["indoors", "outdoors"]:
            rospy.logerr("Switch location state string needs to be either outdoors or indoors")
            rospy.sleep(0.1)
            return 'failure'
        else:
            navigation.switch_location(self.env)
            if self.env=="indoors":
                navigation.localize_in_location(self.location)
            return 'success'



class DetectFire(smach.State):
    '''
    description: robot will return success if it is detecting a fire, failure if it is not
    outcomes: 'success' or 'failure'
    '''

    def __init__(self, timeout=5.0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.timeout = rospy.Duration(timeout)
        self.has_fire = False
        self.fire_position = None
        
        #subscribe to fire topic
        self.fire_topic_sub = rospy.Subscriber(fire_topic, PointStamped, self.fireCallback)
        
    def execute(self, userdata):
        time_start = rospy.Time.now()
        
        while not rospy.is_shutdown() and rospy.Time.now() - time_start < self.timeout:
            if self.has_fire:
                self.fire_topic_sub.unregister()
                robot_pose = navigation.get_current_pose(ref_frame='map')
                robot_position = Point()
                robot_position.x = robot_pose[0]
                robot_position.y = robot_pose[1]
                robot_position.z = 0.0
                goal = navigation.chooseGoal(self.fire_position,robot_position)
                orientation = navigation.getAngle(goal, self.fire_position)
                final_goal = [0 ,0 ,0]
                final_goal[0] = goal.x
                final_goal[1] = goal.y
                final_goal[2] = orientation
                navigation.go_to_pose(final_goal, 'map')
                return 'success'
        self.fire_topic_sub.unregister()
        return 'failure'
    
    def fireCallback(self, data):
        self.has_fire = True
        self.fire_position = Point()
        self.fire_position = data.point
        
class PumpWater(smach.State):
    '''
    description: robot will turn on pumps, wait some time and turn off pumps
    outcomes: 'success' or 'failure'
    '''
    def __init__(self, timeout=20.0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.timeout = rospy.Duration(timeout)
        
    def execute(self, userdata):
        #Call service to turn on pumps
        rospy.loginfo("Pumping water")
        #Wait some time
        rospy.sleep(5)
        #Call service to turn off pumps
        rospy.loginfo("Turning off pumps")
        rospy.sleep(1)
        return 'success'
class MoveToFire(smach.State):
    '''
    description: robot will choose a pose near the fire and move towards it
    outcomes: 'success' or 'failure'
    '''
    def __init__(self, timeout=120.0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.timeout = rospy.Duration(timeout)
        
    def execute(self, userdata):
        #subscrivbe to fire detector to get the point of the fire
        #Move towards the fire until the fire is at a pre determined distance
        #Choose that distance acording to heigght of fire
        #then return true
        rospy.loginfo("Moving to fire")
        return 'success'