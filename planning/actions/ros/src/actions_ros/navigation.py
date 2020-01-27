import rospy
import numpy
import tf
import time
import math
#for rviz visualization
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

# msg imports
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Point, Quaternion, Pose2D, Pose
from std_msgs.msg import String, Header
from std_srvs.srv import Empty
from actionlib_msgs.msg import GoalID, GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid

from tf.transformations import quaternion_from_euler

# seed the pseudorandom number generator
from random import seed
from random import random


class Navigation(object):
    '''
    Wraps all methods related to navigation
    '''
    def __init__(self):
        ns = '/mbzirc2020_0'
        seed(3)
        #Variables
        self.__goals_status = []
        self.fire_dist = rospy.get_param('/mbzirc2020_0/fire_dist')
        self.map_ = None
        self.map_info = None
        self.convert_offset = 0.0
        self.availabilityThreshold = 90

        # services
        self.__global_localization_srv = rospy.ServiceProxy("/global_localization", Empty)

        # publishers
        self.__switch_location = rospy.Publisher(ns + '/switch_to', String, queue_size=10)
        self.__move_base_pub = rospy.Publisher(ns + '/move_base_simple/goal', PoseStamped, queue_size = -1)
        self.__move_base_safe_pub = rospy.Publisher(ns + '/move_base/goal', MoveBaseActionGoal, queue_size = -1)
        self.__cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=2)
        self.__pose_pub = rospy.Publisher("/mbzirc2020_0/initialpose", PoseWithCovarianceStamped, queue_size=10)
        #Topic to draw in rviz
        self.marker_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=5)
        
        # subscribers
        rospy.Subscriber(ns + '/move_base/status', GoalStatusArray, self.__callback_goal_status)
        #Subscribe to Costmap
        self.__sub_costmap_2d = rospy.Subscriber("/mbzirc2020_0/move_base/global_costmap/costmap",OccupancyGrid, self.costmapCallback)

    def get_available_locations(self):
        '''
        description: get a list of current available prerecorded points of interest that
                     the robot is capable of visiting
        input: None
        output (return): list available_locations, a list with the robot available locations,
                         i.e. ["KITCHEN", "BEDROOM", "LIVING_ROOM"]
        '''
        return dict.keys(rospy.get_param('/mbzirc2020_0/move_base/navigation_goals'))

    def get_available_poses(self):
        '''
        description: get a dictionary with the current available prerecorded points of interest that
                     the robot is capable of visiting associated with the respective prerecorded pose
        input: None
        output (return): a dictionary with the robot available locations and respective poses
                         i.e. {'BATHROOM': [-6.102936, -13.723022, -2.842757],
                               'BEDROOM': [-6.363801, -15.167635, -2.613795],
                               'CORRIDOR': [-6.124564, -12.038472, 1.478612],
                               'DINING_ROOM': [-1.673256, -13.470115, 0.869393]}
        '''
        return rospy.get_param('/mbzirc2020_0/move_base/navigation_goals')

    def get_current_pose(self, ref_frame, timeout=10):
        '''
        description: get the current pose of the robot
        input: reference frame
        output (return): a list with the robot current 2D pose as [x, y, yaw] in map frame,
                         i.e. [0.014844779089409413, 0.014529479227721263, 0.004583476270987115]
        '''
        listener = tf.TransformListener()
        if not isinstance(ref_frame, str):
            ref_frame = 'odom'
        global_reference_frame = ref_frame
        sensor_reference_frame = 'base_link'
        wait_for_transform = 0.1
        tf_received = False

        timeout_counter = 0
        # get transformation between map and base_link
        while not tf_received:
            try:
                # wait for transform to become available
                listener.waitForTransform (global_reference_frame, sensor_reference_frame, rospy.Time(0), rospy.Duration(wait_for_transform))
                # get sensor pose transform
                (translation, rotation) = listener.lookupTransform (global_reference_frame, sensor_reference_frame, rospy.Time(0))

                tf_received = True

                timeout_counter = timeout_counter + 0.1
            except Exception, e:
                rospy.sleep(0.1)
                tf_received = False
                if timeout_counter > timeout:
                    rospy.logerr('[get_current_pose] Timeout of wating for transform excedded! Returned empty pose list : {}'.format(e))
                    return []

        # transform quaternion into euler using tf library
        quaternion = (rotation[0], rotation[1], rotation[2], rotation[3])
        euler = tf.transformations.euler_from_quaternion (quaternion)

        # return sensor pose (x, y, yaw)
        return [translation[0], translation[1], euler[2]]

    def get_nearest_location(self):
        '''
        description: from all available locations get the closest one in straight line to the robot
        input: None
        output (return): string with the name of the nearest location
                         i.e. 'CORRIDOR'
        '''

        # get available poses
        available_poses = self.get_available_poses()
        # get current location
        robot_pose = self.get_current_pose()

        prev_dist = 1000000
        for key, value in available_poses.iteritems():
            dist = numpy.sqrt( pow(value[0]-robot_pose[0],2)+ pow(value[1]-robot_pose[1],2) )

            if dist < prev_dist:
                nearest_location = key
                prev_dist = dist

        return nearest_location

    def robot_distance_to_waypoint(self, wp):
        '''
        description: check the distance to a waypoint
        input: string wp
        output: None if failed, or float distance
        '''
        try:
            wp_location = self.get_available_poses()[wp]
        except KeyError:
            rospy.logerr('{} is not a valid waypoint from this list: {}'.format(wp, self.get_available_poses()))
            return None

        robot_pose = self.get_current_pose()

        return numpy.sqrt( pow(wp_location[0]-robot_pose[0],2)+ pow(wp_location[1]-robot_pose[1],2) )

    def switch_location(self, location):
        self.__switch_location.publish(String(data=location))

    def go_to_location(self, location, frame_id, timeout=50.0):
        '''
        description: move the robot to the desired location
        input: string location, the name of the location that you want the robot to go
               double timeout, max time to allow to reach the location before returning
               string frame_id, frame where the pose is located

               NOTE: the location needs to be loaded in the parameter server in advance, to get a list
               of availbale locations use mbot.get_available_locations() function
        output: bool success, whether if the robot was succesful or not at reaching the goal
        '''
        timeout_ros = rospy.Duration(timeout)
        time_start = rospy.Time.now()

        try:
            location_pose = self.get_available_poses()[location]
        except KeyError:
            rospy.logerr('{} is not a valid waypoint from this list: {}'.format(location, self.get_available_poses()))
            return None

        goal = PoseStamped()
        
        if not isinstance(frame_id, str):
            frame_id = 'odom'
        goal.header.frame_id = frame_id
        goal.header.stamp = rospy.Time.now()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, location_pose[2])
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        goal.pose.position.x = location_pose[0]
        goal.pose.position.y = location_pose[1]
        goal.pose.position.z = 0

        final_goal = MoveBaseActionGoal()
        final_goal.header = goal.header
        final_goal.goal_id.stamp = rospy.Time.now()
        final_goal.goal_id.id = str(random())
        goal_id = final_goal.goal_id
        final_goal.goal.target_pose = goal

        self.__move_base_safe_pub.publish(final_goal)

        rospy.loginfo("Sending goal to move_base_simple, destination : " + location)
        
        while not rospy.is_shutdown() and rospy.Time.now() - time_start < timeout_ros:
            if self.__goals_status:
                for i in self.__goals_status:
                    if i.goal_id == goal_id and i.status == 3:
                        return True
        return False


    def go_to_pose(self, location_pose, frame_id, timeout=50):
        '''
        description: move the robot to the desired pose
        needs: mir_move_base_safe and mbot_actions move_base_server
        input: PoseStamped location_pose, x, y, yaw
               double timeout, max time to allow to reach the location before returning
               string frame_id, frame where the pose is located
        output: bool success, whether if the robot was succesful or not at reaching the goal
        '''
        timeout_ros = rospy.Duration(timeout)
        time_start = rospy.Time.now()
        goal = PoseStamped()
        
        if not isinstance(frame_id, str):
            frame_id = 'odom'
        goal.header.frame_id = frame_id
        goal.header.stamp = rospy.Time.now()
        quaternion = tf.transformations.quaternion_from_euler(0, 0, location_pose[2])
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        goal.pose.position.x = location_pose[0]
        goal.pose.position.y = location_pose[1]
        goal.pose.position.z = 0
        
        final_goal = MoveBaseActionGoal()
        final_goal.header = goal.header
        final_goal.goal_id.stamp = rospy.Time.now()
        final_goal.goal_id.id = str(random())
        goal_id = final_goal.goal_id
        final_goal.goal.target_pose = goal

        self.__move_base_safe_pub.publish(final_goal)

        rospy.loginfo("Sending goal to move_base_simple, destination : (" + str(location_pose[0]) + "," + str(location_pose[1]) + ") with yaw: " + str(location_pose[2]))
        
        while not rospy.is_shutdown() and rospy.Time.now() - time_start < timeout_ros:
            if self.__goals_status:
                for i in self.__goals_status:
                    if i.goal_id == goal_id and i.status == 3:
                        return True
        return False

    def stop_movement(self, timeout=5.0):
        '''
        description: cancels the goal currently being pursued
        needs: mir_move_base_safe and mbot_actions move_base_server
        input: tdouble timeout, max time to allow to reach the location before returning
        output: bool success, whether the cancelation was successful or not
        '''
        cancel_msg = GoalID()
        cancel_msg.stamp = rospy.Time.now()
        if self.__goals_status:
            for i in self.__goals_status:
                if self.__goals_status[i].status == 0 or self.__goals_status[i].status == 1:
                    goal_id = self.__goals_status[i].goal_id
                    cancel_msg.id = goal_id
                    self.__cancel_pub.publish(cancel_msg)
                    return True
        return False
        

    def localize_in_location(self, location_name):
        '''
        Localizes the base at the pose of a location, similar to using rviz's "2D Pose Estimate"
        :param location_name: string identifying the location, e.g.: location_name='CHARGING_STATION'
        :return: None
        '''      
        # get the pose2D relative to the location
        location_to_pose_2d_dict = self.get_available_poses()
        if location_name not in location_to_pose_2d_dict:
            rospy.logerr("Location {} not found in locations dict", location_name)
            return

        pose_2d = Pose2D(*location_to_pose_2d_dict[location_name])

        # compute the covariance matrix. The diagonal elements 0, 1, 5 are the squared std_dev of x, y, theta respectively
        cov_mat = numpy.zeros((6, 6))
        cov_mat[0, 0] = 0.2**2
        cov_mat[1, 1] = 0.2**2
        cov_mat[5, 5] = (numpy.pi/12)**2

        pose_cov = PoseWithCovarianceStamped(header=Header(stamp=rospy.Time.now(), frame_id='map'))
        pose_cov.pose.pose.position = Point(x=pose_2d.x, y=pose_2d.y, z=0.0)
        pose_cov.pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, pose_2d.theta))
        pose_cov.pose.covariance = cov_mat.flatten()

        rospy.sleep(0.5)  # Wait for publisher to be created
        self.__pose_pub.publish(pose_cov)
        rospy.sleep(0.5)  # Wait for message to be received

    def try_global_localization(self):
        '''
        description: mbot will trigger global localization in amcl, then rotate and walk around a bit to try and get a correct estimate
        input: None
        output: None
        TODO: modify amcl to provide an estimate of the estimation quality using the sensor model, see: https://answers.ros.org/question/217999/amcl-estimation-quality/
        '''
        rospy.logwarn("PERFORMING GLOBAL LOCALIZATION: HOLD YOUR JOYPAD!")
        self.__global_localization_srv.call()
        
    #Function used to sort the points on the circle around the target goal by distance to the robot
    def distanceToRobot(self,point):
        return self.dist2D(point.x, point.y, self.robot_position.x, self.robot_position.y)

    def dist2D(self,x1,y1,x2,y2):
        return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) 
        
    #Function that determines if a certain point on the map is available to be set as goal
    def isCellAvailable(self, point, occupancy):
        #get the costmap by subscribing to nav_msgs/OccupancyGrid.msg
        [x,y]=self.worldToMap(point.x,point.y)
        cost = self.map_[int(round(x))][int(round(y))]
        # rospy.loginfo("cost is: " + str(cost))
        if cost <= occupancy:
            return True
        else:
            return False
            
    #Function to tranform a world coordinate into a map one
    def worldToMap(self, wx, wy):
        if self.map_info is None:
            rospy.logerr("Change costmap subscriber topic name mbzirc2020_0")
        origin_x = self.map_info.info.origin.position.x
        origin_y = self.map_info.info.origin.position.y
        resolution = self.map_info.info.resolution

        if (wx < origin_x or wy < origin_y):
            return None

        mx = (wx - origin_x) / resolution - self.convert_offset
        my = (wy - origin_y) / resolution - self.convert_offset

        if (mx < self.map_info.info.width and my < self.map_info.info.height):
            return [mx , my]

        return None

	#Several points in a straight line will be computed and checked to determine if there is a free straight path to the target
    def isPathToTargetAvailable(self, origin, target):
        result = True
		#equation of the line y=mx+b
        dx = target.x - origin.x
        dy = target.y - origin.y
        if dx != 0:
            m = dy/dx
        else:
            return False #fix this, maybe by making an equation x=my+b
        b = target.y - m*target.x

		#Calculate the interval of x between  2 consecutive points in the straight line
        number_of_points = int(round(20 * self.fire_dist))
        dx=float(dx)
        interval = dx/number_of_points

        line = []
        for i in range(1,number_of_points/2):
            point = Point()
            point.x = origin.x + interval*i
            point.y = m * point.x + b
            point.z = 0
            line.append(point)
            if not self.isCellAvailable(point=point,occupancy=self.availabilityThreshold):
                result = False
		# self.show_spheres_in_rviz(line)
        # print("result of line is " + str(result))
        return result

    def getAngle(self, origin, target):
        angle_raw = math.atan((target.y- origin.y) / abs(target.x-origin.x))
        #conditions to counter the decrease of the angle
        if target.x-origin.x < 0 and angle_raw < 0:
            angle_refined = - (math.pi + angle_raw)
        elif target.x-origin.x < 0 and angle_raw > 0:
            angle_refined = math.pi - angle_raw
        else:
            angle_refined = angle_raw
            
        angle = angle_refined
        
        #condition to solve the case where the calculated angle and the robot rotation are in different multiples of 2*pi
        if angle > math.pi:
            angle = angle - 2 * math.pi
        elif angle < -math.pi:
            angle = angle + 2 * math.pi
        
        return angle
        
    def chooseGoal(self,target_position, robot_position):
        try:
            self.fire_dist = rospy.get_param('/mbzirc2020_0/fire_dist')
        except e:
            rospy.logerr("Error in importing param of fire dist: " + e)
        #number of points to check in the circumference
        number_points = int(self.fire_dist * 25)
        #radius of the circle
        r = self.fire_dist
        
        #To be used in the sorting function
        self.robot_position = robot_position
        
        if r > 0:
            #make a circle of positions, check which ones are closer to the robot, and if it is reachable
            circle = []
            for i in range(0,number_points):
                point = Point()
                point.x = round(math.cos(2*math.pi/number_points*i)*r,2) + target_position.x
                point.y = round(math.sin(2*math.pi/number_points*i)*r,2) + target_position.y
                point.z = 0
                circle.append(point)
            #order the circle variable by proximity to the ROBOT
            circle.sort(key=self.distanceToRobot)
            self.show_spheres_in_rviz(circle)
            #iterate the ordered set check if it is a free cell with a clear path to the goal
            while not rospy.is_shutdown() and len(circle) != 0:
                if not self.isCellAvailable(circle[0],occupancy=0):
                    del circle[0]
                    rospy.loginfo("deleting possible location")
                else:
                    if not self.isPathToTargetAvailable(circle[0],target_position):
                        del circle[0]
                        rospy.loginfo("deleting because of line")
                    else:
                        rospy.loginfo("Point can be chosen in: " + str(circle[0].x) + "," + str(circle[0].y) + ")")
                        break
        else:
            rospy.loginfo("Radius of circle around fire is wrong")
            
        # rospy.loginfo("len of circle is :" + str(len(circle)))
        #return goal, that is the closest point to the robot that was not removed from the list by the previous conditions
        if len(circle) > 0:
            return circle[0]
        else:
            return target_position

    def show_spheres_in_rviz(self, points):
        marker_array = []
        for i in range(0,len(points)):
            marker = Marker(
                    type=Marker.SPHERE,
                    id=i,
                    lifetime=rospy.Duration(60),
                    pose=Pose(Point(points[i].x, points[i].y, points[i].z), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.1, 0.1, 0.1),
                    header=Header(frame_id='/map'),
                    color=ColorRGBA(0.0, 0.0, 1.0, 0.8))
            marker_array.append(marker)
        self.marker_publisher.publish(marker_array)
        
    def __callback_goal_status(self, data):
        self.__goals_status = data.status_list
        
    #Callback for the costmap
    def costmapCallback(self, data):
        self.map_info = data		#TODO: maybe smthg more
        
        #self.map_ = np.arrange(data.info.width*data.info.height).reshape(data.info.width,data.info.height)
        
        self.map_ = numpy.zeros((data.info.width,data.info.height))
        
        for i in range(0,data.info.height):
            for j in range(0,data.info.width):
                self.map_[j][i] = data.data[i*data.info.width + j]
                
        self.map_info.data = None

