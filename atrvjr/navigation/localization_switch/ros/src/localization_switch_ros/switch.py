#!/usr/bin/env python
import roslib
import rospy
import math
import numpy as np
import tf
import std_msgs
import rospkg
import string
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from rospy import loginfo as log

class PoseWithYaw:
	x = 0.0
	y = 0.0
	yaw = 0.0
	raw = PoseWithCovarianceStamped()

# "header:
# 	seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# pose:
#   pose:
#     position: {x: 0.0, y: 0.0, z: 0.0}
#     orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
#   covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#     0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 

class Switch(object):
	def __init__(self):
		rospy.init_node('switch')

		self.listener = tf.TransformListener()

		#Get outdoor and indoor topic names
		self.outdoors_topic = rospy.get_param('/mbzirc2020_0/localization_switch/outdoors_topic')
		self.indoors_topic = rospy.get_param('/mbzirc2020_0/localization_switch/indoors_topic')
		self.control_topic = rospy.get_param('/mbzirc2020_0/localization_switch/control_topic')
		self.pub_topic = rospy.get_param('/mbzirc2020_0/localization_switch/pub_topic')

		#Subscribe to indoors pose topic
		self.sub_indoors_topic = rospy.Subscriber(self.indoors_topic, PoseWithCovarianceStamped , self.indoorsCallback)

		#Subscribe to outdoors pose topic
		self.sub_outdoors_topic = rospy.Subscriber(self.outdoors_topic, Odometry , self.outdoorsCallback)

		#Subscribe to control topic
		self.sub_control = rospy.Subscriber(self.control_topic, String, self.controlCallback)

	 	# #Topic to publish the pose you want to reach
		self.pose_publisher = rospy.Publisher(self.pub_topic, PoseWithCovarianceStamped, queue_size=1)
		
		# Initialize variables
		self.indoors_sensor_reading = PoseWithYaw()
		self.outdoors_sensor_reading = PoseWithYaw()
		self.is_indoors = False

		self.rate = rospy.Rate(10.0)

	def destroySubscribers(self):
		self.sub_indoors_topic.unregister()
		self.sub_outdoors_topic.unregister()

		log("Hope you enjoyed our service! Come back any time!")
	

	def run(self):
		
		while not rospy.is_shutdown():
			log("Doin ma thang")
			
			# if self.is_indoors:
			# 	self.pose_publisher.publish(self.indoors_sensor_reading.raw)
			# else:
			# 	self.pose_publisher.publish(self.outdoors_sensor_reading.raw)

			self.rate.sleep()

	
	def dist2D(self,x1,y1,x2,y2):
		return math.sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) 

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

	def controlSwitcher(input):
		switcher = {
			"outdoors": False,
			"outdoor": False,
			"outside": False,
			"indoors": True,
			"indoor": True,
			"inside": True
		}

		return switcher.get(input, False)
  
  

	def indoorsCallback(self, data):
		self.indoors_sensor_reading.x = data.pose.pose.position.x
		self.indoors_sensor_reading.y = data.pose.pose.position.y
		self.indoors_sensor_reading.yaw = tf.transformations.euler_from_quaternion(self.quaternionToArray(data.pose.pose.orientation))[2]
		self.indoors_sensor_reading.raw = data

	def outdoorsCallback(self, data):
		self.outdoors_sensor_reading.x = data.pose.pose.position.x
		self.outdoors_sensor_reading.y = data.pose.pose.position.y
		self.outdoors_sensor_reading.yaw = tf.transformations.euler_from_quaternion(self.quaternionToArray(data.pose.pose.orientation))[2]
		self.outdoors_sensor_reading.raw.header = data.header
		self.outdoors_sensor_reading.raw.pose = data.pose

	
	def controlCallback(self, data):
		self.is_indoors = self.controlSwitcher(data.data.lower())

	

def main():
	# create object of the class DynGoal (constructor will get executed!)
	my_object = Switch()
	# call run method of class DynGoal
	my_object.run()
	# destroy subscriptions
	my_object.destroySubscribers()

if __name__ == '__main__':
	main()
