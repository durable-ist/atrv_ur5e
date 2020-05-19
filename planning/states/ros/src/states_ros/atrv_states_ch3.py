#!/usr/bin/env python

import smach
import rospy
import tf
import rospy
import math
from rospy import Duration, Time
from numpy import array
from numpy.linalg import norm as euclidean
from scipy.spatial.distance import cdist
from std_srvs.srv import Trigger, TriggerRequest

import roslib
import actionlib
import moveit_commander
import sys
from moveit_commander.exception import MoveItCommanderException
import moveit_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse

from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, PoseStamped, Point

import os

#import navigation actions
import actions_ros.navigation_ch3 as navigation_file
import actions_ros.terabee_threshold as terabee_file

    # import actions_ros.pick_RGB_class as pick_RGB_class
    # import actions_ros.place_algorithm_class as place_algorithm_class
    # import actions_ros.laser_allign_class as laser_allign_class
    # import actions_ros.place_first as place_first
    # import actions_ros.object_listener as object_listener

navigation = navigation_file.Navigation()
terabee = terabee_file.Terabee()

fire_topic = '/depth_point'
fire_waiting_duration = 15
number_fires = 5
proximity_fire_threshold = 0.2



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
        
# class SwitchLocalization(smach.State):
#     '''
#     description: robot will switch to either indoors or outdoors localization
#     input: String, that says either indoors or outdoors
#     outcomes: 'success' or 'failure'
#     '''

#     def __init__(self, env, location='ENTRANCE_DOOR_1'):
#         smach.State.__init__(self, outcomes=['success', 'failure'], input_keys=['in_data'])
#         self.env = env
#         self.location = location
    
#     def execute(self, userdata):
#         if not isinstance(self.env, str):
#             rospy.logerr("Switch location state needs a string as input as either outdoors or indoors")
#             rospy.sleep(0.1)
#             return 'failure'
#         elif self.env not in ["indoors", "outdoors"]:
#             rospy.logerr("Switch location state string needs to be either outdoors or indoors")
#             rospy.sleep(0.1)
#             return 'failure'
#         else:
#             navigation.switch_location(self.env)
#             if self.env=="indoors":
#                 rospy.loginfo("Localizing in " +  self.location)
#                 navigation.localize_in_location(self.location)
#             return 'success'



class DetectFire(smach.State):
    '''
    description: robot will return success if it is detecting a fire, failure if it is not
    outcomes: 'success' or 'failure'
    '''

    def __init__(self, test = False ,timeout=5.0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.timeout = timeout
        self.has_fire = False
        self.test = test
        
    def execute(self, userdata):
        if terabee.execute(self.timeout):
            rospy.loginfo("Detected a fire")
            return 'success'
        else:
            if self.test:
                rospy.logwarn("Did not detect a fire")
                return 'success'
            else:
                return 'failure'
        
class PumpWater(smach.State):
    '''
    description: robot will turn on pumps, wait some time and turn off pumps
    outcomes: 'success' or 'failure'
    '''
    def __init__(self, duration = 5.0, timeout=20.0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.timeout = rospy.Duration(timeout)
        self.duration = duration
        
    def execute(self, userdata):
        #Call service to turn on pumps
        rospy.loginfo("Pumping water")
        # wait for this sevice to be running
        rospy.wait_for_service('/actuators_system/start_pump')
        
        # Create the connection to the service. Remember it's a Trigger service
        sos_service_start = rospy.ServiceProxy('/actuators_system/start_pump', Trigger)
        
        # Create an object of the type TriggerRequest. We nned a TriggerRequest for a Trigger service
        sos_start = TriggerRequest()
        
        # Now send the request through the connection
        result_start = sos_service_start(sos_start)
        rospy.loginfo("The result of turning on pumps service is: ")
        print(result_start)
        #Wait some time
        rospy.sleep(self.duration)
        #Call service to turn off pumps
        rospy.loginfo("Turning off pumps")
        # wait for this sevice to be running
        rospy.wait_for_service('/actuators_system/stop_pump')
        
        # Create the connection to the service. Remember it's a Trigger service
        sos_service_stop = rospy.ServiceProxy('/actuators_system/stop_pump', Trigger)
        
        # Create an object of the type TriggerRequest. We nned a TriggerRequest for a Trigger service
        sos_stop = TriggerRequest()
        
        # Now send the request through the connection
        result = sos_service_stop(sos_stop)
        rospy.loginfo("The result of turning on pumps service is: ")
        print(result)

        return 'success'

class MoveArm(smach.State):
    '''
    description: robot will move the arm to put out the fire
    '''
    def __init__(self, fire):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        # self.group = moveit_commander.MoveGroupCommander("manipulator") 
        # self.robot = moveit_commander.RobotCommander()
        self.fire = fire
        self.config1 = [-1.560801331146969202, -1.6985074482359828, 1.526722256337301, -2.496563573876852, -1.5618837515460413, 0.10650110244750977]
        self.config2 = [-1.5910709539996546, -1.1811741155437012, 0.44416505495180303, -2.3148914776244105, -1.5589693228351038, 0.10683679580688477]

    def go_to_joint_pose(self, arm_configuration):
        try:
            # self.group.set_joint_value_target(arm_configuration)
            # self.group.go(wait=True)
            rospy.loginfo("Moved the arm to " + str(self.fire))
            return "success"
        except MoveItCommanderException, e:
            rospy.logerr("Could not move the arm!")
            return "failure"

    def execute(self, userdata):

        if self.fire == "1":
            result = self.go_to_joint_pose(self.config1)
        if self.fire == "2":
            result = self.go_to_joint_pose(self.config2)
        return result

        return 'success'

class Stop(smach.State):
    '''
    description: robot will stop
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        
    def execute(self, userdata):
        if navigation.stop_movement():
            return 'success'
        else:
            return 'failure'