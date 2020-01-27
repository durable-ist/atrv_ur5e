#!/usr/bin/env python
import roslib
import rospy
import math
import numpy as np
import tf
from tf import transformations as t
import std_msgs
import rospkg
import string
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import dynamic_reconfigure.client

from rospy import loginfo as log

class PoseWithYaw:
	x = 0.0
	y = 0.0
	yaw = 0.0
 	def __init__(self,raw):
		self.raw = raw #PoseWithCovarianceStamped()

class Switch(object):
	def __init__(self):
		rospy.init_node('switch')

		self.listener = tf.TransformListener()
		self.broadcaster = tf.TransformBroadcaster()

		ns = '/mbzirc2020_0/localization_switch/'

		#Get outdoor and indoor topic names
		self.outdoors_topic = rospy.get_param(ns + 'outdoors_topic')
		self.indoors_topic = rospy.get_param(ns + 'indoors_topic')
		self.control_topic = rospy.get_param(ns + 'control_topic')
		self.pub_topic = rospy.get_param(ns + 'pub_topic')
		self.origin_x = rospy.get_param(ns + 'origin_x')
		self.origin_y = rospy.get_param(ns + 'origin_y')
		self.origin_yaw = rospy.get_param(ns + 'origin_yaw')
		self.amcl_enable_param = rospy.get_param(ns + 'amcl_enable_tf_param')
		self.topics_namespace = rospy.get_param('switch/namespace')

		#Subscribe to indoors pose topic
		self.sub_indoors_topic = rospy.Subscriber(self.topics_namespace + self.indoors_topic, PoseWithCovarianceStamped , self.indoorsCallback)

		#Subscribe to outdoors pose topic
		self.sub_outdoors_topic = rospy.Subscriber(self.topics_namespace + self.outdoors_topic, Odometry , self.outdoorsCallback)

		#Subscribe to control topic
		self.sub_control = rospy.Subscriber(self.topics_namespace + self.control_topic, String, self.controlCallback)

	 	# #Topic to publish the pose you want to reach
		self.pose_publisher = rospy.Publisher(self.topics_namespace + self.pub_topic, PoseWithCovarianceStamped, queue_size=5)
		self.amcl_switch_publisher = rospy.Publisher(ns + "amcl_broadcast", Bool, queue_size=1)
		
		# Initialize variables
		self.indoors_sensor_reading = PoseWithYaw(None)
		self.outdoors_sensor_reading = PoseWithYaw(None)
		self.is_indoors = False

		self.rate = rospy.Rate(10.0)

	def destroySubscribers(self):
		self.sub_indoors_topic.unregister()
		self.sub_outdoors_topic.unregister()

		log("Hope you enjoyed our service! Come back any time!")
	

	def run(self):
		
		while not rospy.is_shutdown():	
			#Depending on what it is wanted the node will suply the pose of indoors nav or outdoors nav
			if self.is_indoors:
				if self.indoors_sensor_reading.raw is not None:
					self.pose_publisher.publish(self.indoors_sensor_reading.raw)
			else:
				if self.outdoors_sensor_reading.raw is not None:
					self.updateTFoutdoors(self.outdoors_sensor_reading.raw)
					self.pose_publisher.publish(self.outdoors_sensor_reading.raw)

			self.rate.sleep()

	
	def dist2D(self,x1,y1,x2,y2):
		return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) 

	#update the tf when the pose comes in relation to map
	# def updateTFindoors(self,raw):
	# 	try:
	# 		(trans_odom_base, rot_odom_base) = self.listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
	# 	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	# 		log("Exception ocurred transforming from base_link to odom in localization switch updateTFindoors")

	# 	(trans_map_base, rot_map_base) = self.invertTF(
    #   									(raw.pose.pose.position.x,raw.pose.pose.position.y,0),
    #            							(raw.pose.pose.orientation.x,raw.pose.pose.orientation.y,raw.pose.pose.orientation.z,raw.pose.pose.orientation.w)
    # 				)
	# 	# print(trans_odom_base, trans_map_base)
	# 	trans_odom_map = trans_odom_base - trans_map_base
	# 	# print(trans_odom_map)
  
	# 	#Convert the quaternions to yaws and subtract as well
	# 	yaw_odom_base = tf.transformations.euler_from_quaternion(rot_odom_base)[2]
	# 	yaw_map_base = tf.transformations.euler_from_quaternion(rot_map_base)[2]
  
	# 	yaw_odom_map = yaw_map_base - yaw_odom_base #TO DOOOOOOOOOOOOOOOOOOO
  
	# 	rot_odom_map = tf.transformations.quaternion_from_euler(0,0,yaw_odom_map)
  
	# 	self.broadcaster.sendTransform(
	# 					trans_odom_map,
	# 					rot_odom_map,
	# 					rospy.Time.now(),
	# 					"odom",
	# 					"map"
	# 				)

	# def enableAMCL(self, state):
	# 	rospy.set_param(self.amcl_enable_param, state)
		
  
	#Transforms the static transform from map to odom and forwards the result from ekf
	def updateTFoutdoors(self,raw):
		self.broadcaster.sendTransform(
						(self.origin_x, self.origin_y, 0.0),
						tf.transformations.quaternion_from_euler(0,0,self.origin_yaw),
						rospy.Time.now(),
						"odom",
						"map"
					)
  
  		self.broadcaster.sendTransform(
						(raw.pose.pose.position.x, raw.pose.pose.position.y, 0),
						(raw.pose.pose.orientation.x, raw.pose.pose.orientation.y, raw.pose.pose.orientation.z, raw.pose.pose.orientation.w),
						rospy.Time.now(),
						"base_link",
						"odom"
					)
  
	#Invert a transform tf
	def invertTF(self, trans, rot):
		transform = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
  		inversed_transform = t.inverse_matrix(transform)
		translation = t.translation_from_matrix(inversed_transform)
		quaternion = t.quaternion_from_matrix(inversed_transform)
		return (translation, quaternion)

	def vectortToPoint(self,vector):
		point = geometry_msgs.Point()
		point.x = vector[0]
		point.y = vector[1]
		point.z = vector[2]
		return point

	def quaternionToArray(self, quaternion):
		array = []
		array.append(quaternion.x)
		array.append(quaternion.y)
		array.append(quaternion.z)
		array.append(quaternion.w)
		return array

	def controlSwitcher(self, input, previous):
		switcher = {
			"outdoors": False,
			"outdoor": False,
			"outside": False,
			"indoors": True,
			"indoor": True,
			"inside": True
		}

		return switcher.get(input, previous)
  
  
	#gets the pose from the indoor pose estimator and saves it
	def indoorsCallback(self, data):
		self.indoors_sensor_reading.x = data.pose.pose.position.x
		self.indoors_sensor_reading.y = data.pose.pose.position.y
		self.indoors_sensor_reading.yaw = tf.transformations.euler_from_quaternion(self.quaternionToArray(data.pose.pose.orientation))[2]
		self.indoors_sensor_reading.raw = PoseWithCovarianceStamped()
		self.indoors_sensor_reading.raw = data

	#gets the pose from the outdoor pose estimator and saves it
	def outdoorsCallback(self, data):
		self.outdoors_sensor_reading.x = data.pose.pose.position.x
		self.outdoors_sensor_reading.y = data.pose.pose.position.y
		self.outdoors_sensor_reading.yaw = tf.transformations.euler_from_quaternion(self.quaternionToArray(data.pose.pose.orientation))[2]
		self.outdoors_sensor_reading.raw = PoseWithCovarianceStamped()
		self.outdoors_sensor_reading.raw.header = data.header
		self.outdoors_sensor_reading.raw.pose = data.pose

	#gets a string that says if the robot is indoors or outdoors. If any other string is sent it will not change it status
	def controlCallback(self, data):
		self.is_indoors = self.controlSwitcher(data.data.lower(), self.is_indoors)
		if self.is_indoors:
			rospy.loginfo("[Localization_switch] switching to indoors")
			# self.enableAMCL('true')
			self.amcl_switch_publisher.publish(True)
		else:
			rospy.loginfo("[Localization_switch] switching to outdoors")
			# self.enableAMCL('false')
			self.amcl_switch_publisher.publish(False)
	

def main():
	# create object of the class DynGoal (constructor will get executed!)
	my_object = Switch()
	# call run method of class DynGoal
	my_object.run()
	# destroy subscriptions
	my_object.destroySubscribers()

if __name__ == '__main__':
	main()
