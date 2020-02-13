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

from std_srvs.srv import Empty
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, PoseStamped, Point

import os

#import navigation actions
import actions_ros.navigation as navigation_file

navigation = navigation_file.Navigation()

fire_topic = '/depth_point'
fire_waiting_duration = 30
number_fires = 5
proximity_fire_threshold = 0.2
try_counter_limit = 4

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
                rospy.loginfo("Localizing in " +  self.location)
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
        self.fire_pointstamped = []
        self.fire_frame = None
        self.tried_to_move = False
        
        self.listener = tf.TransformListener()
        
        #subscribe to fire topic
        self.fire_topic_sub = rospy.Subscriber(fire_topic, PointStamped, self.fireCallback)
        
    def execute(self, userdata):
        #Resets the variables to clean any detections made while moving
        self.has_fire = False
        self.fire_pointstamped = []
        try_counter = 0
        initial_pose = navigation.get_current_pose(ref_frame='map')
        
        time_start = rospy.Time.now()
        rospy.loginfo("Checking fire")
        while not rospy.is_shutdown() and rospy.Time.now() - time_start < self.timeout:
            if self.has_fire:
                if self.moveToFire(time_start):
                    if self.amInearTheFire(proximity_fire_threshold):
                        rospy.loginfo("Fire is within " + str(proximity_fire_threshold) + " mtrs of the desired distance")
                        return 'success'
                    elif try_counter < try_counter_limit:
                        #Resets timer to try and find fire again
                        rospy.logwarn("Fire is not close, moving again towards it!")
                        time_start = rospy.Time.now()
                        self.has_fire = False
                        self.fire_pointstamped = []
                        try_counter += 1
                    else:
                        #Resets timer to try and find fire again, moving to the waypoint where it saw fire firstly
                        rospy.logwarn("Trials exceeded, moving again to detect fire waypoint!")
                        if not navigation.go_to_pose(initial_pose, 'map', timeout=30):
                            rospy.logerr("Timeout moving to detect fire waypoint!")
                            if not navigation.stop_movement():
                                rospy.logerr("Could not stop the movement after moving to detect fire pose error")
                            return 'failure'
                        time_start = rospy.Time.now()
                        self.has_fire = False
                        self.fire_pointstamped = []
                        try_counter = 0
                else:
                    return 'failure'
        rospy.logwarn("Did not find any fire")
        return 'failure'
    
    def moveToFire(self, time_start):
        rospy.loginfo("Fire Detected! Moving towards it.")
        timeout = True
        while not rospy.is_shutdown() and rospy.Time.now() - time_start < rospy.Duration(fire_waiting_duration):
            if len(self.fire_pointstamped) >= number_fires:
                timeout = False
                break
        #if the cicle ends and did not see number_fires of fires, the variable timeout will remain true and return failure since it timedout
        if timeout:
            rospy.logwarn("Returned failure in detect fire after timeout in detecting " + str(number_fires) + " fires")
            return False
        
        #get the median of the measurements of the fire
        fire_points = []
        for fire in self.fire_pointstamped:     
            fire_pointstamped = PointStamped()
            fire_pointstamped = self.listener.transformPoint('map', fire)
        
            fire_point = Point()
            fire_point = fire_pointstamped.point
            
            fire_points.append(fire_point)
        median_fire_position = self.geometric_median(fire_points)
        
        #get the robot position
        robot_pose = navigation.get_current_pose(ref_frame='map')
        # rospy.loginfo(robot_pose)
        robot_position = Point(x=robot_pose[0], y=robot_pose[1], z=0.0)
        
        #Choose the goal to shoot water from (the distance is set in the state machine launch file)
        goal = navigation.chooseGoal(median_fire_position,robot_position)
        
        orientation = navigation.getAngle(goal, median_fire_position)
        
        final_goal = [goal.x, goal.y, orientation]
        #Visualization
        # rospy.loginfo(final_goal)
        
        if navigation.go_to_pose(final_goal, 'map', timeout=60):
            return True
        else:
            rospy.logerr("Timeout moving to fire!")
            if not navigation.stop_movement():
                rospy.logerr("Could not stop the movement after moving to fire error")
            return True
        
    def amInearTheFire(self, threshold):
        #Resets the variables to clean any detections made while moving
        self.has_fire = False
        self.fire_pointstamped = []
        try:
            fire_dist = rospy.get_param('/mbzirc2020_0/fire_dist')
        except e:
            rospy.logerr("Error in importing param of fire dist: " + e)
        
        time_start = rospy.Time.now()
        timeout = True
        while not rospy.is_shutdown() and rospy.Time.now() - time_start < rospy.Duration(fire_waiting_duration-5):
            if len(self.fire_pointstamped) >= (number_fires - 3):
                timeout = False
                break
        #if the cicle ends and did not see number_fires of fires, the variable timeout will remain true and return failure since it timedout
        if timeout:
            rospy.logwarn("Returned failure in detect fire after timeout in double check")
            return False
        
        #get the median of the measurements of the fire
        fire_points = []
        for fire in self.fire_pointstamped:     
            fire_pointstamped = PointStamped()
            fire_pointstamped = self.listener.transformPoint('map', fire)
        
            fire_point = Point()
            fire_point = fire_pointstamped.point
            
            fire_points.append(fire_point)
        median_fire_position = self.geometric_median(fire_points)
        
        #get the robot position
        robot_pose = navigation.get_current_pose(ref_frame='map')
        
        distance = self.dist2D(median_fire_position.x, median_fire_position.y, robot_pose[0], robot_pose[1])
        rospy.logerr("Distance from robot_pose to fire is " + str(distance) + " | limits:["+str(fire_dist - threshold) + "," + str(fire_dist + threshold) + "]")
        if  distance < fire_dist + threshold and distance > fire_dist - threshold:
            return True
        else:
            return False
        
        
    def geometric_median(self, array_points):
        array = []
        for element in array_points:
            array.append([element.x, element.y, element.z])
    
        distances = []
    
        for point in array:
            points = []
            points.append(point)
            dist_vec = cdist(points, array)
            dist = dist_vec.sum()
            distances.append(dist)
            
        final = array[distances.index(min(distances))]
        return Point(x=final[0], y=final[1], z= final[2])
    
    def dist2D(self,x1,y1,x2,y2):
        return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
    
    def fireCallback(self, data):
        self.has_fire = True
        pointstamped = PointStamped()
        pointstamped = data
        self.fire_pointstamped.append(pointstamped)
        self.fire_frame = data.header.frame_id
        #rospy.loginfo("Detected fire at: (" + str(data.point.x) + "," + str(data.point.y) + ")")
        
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