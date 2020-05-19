#!/usr/bin/env python
import sys
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import  PointStamped
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Float64MultiArray



RATE = 1
OFFSET = 10
TEMPERATURE_TRESHOLD = 50 #degrees
PIXEL_TRESHOLD = 1

class Terabee:

  def __init__(self):
    self.temp_array_sub= rospy.Subscriber("teraranger_evo_thermal/raw_temp_array", Float64MultiArray, self.terabee_cb)

  def terabee_cb(self, msg):
    self.temp_array = msg.data

  def check_hot_zone(self):
    counter = 0
    for item in self.temp_array:
      if item > TEMPERATURE_TRESHOLD:
        rospy.loginfo("Temperature above threshold: " + str(item))
        counter += 1
    if counter > PIXEL_TRESHOLD:
      rospy.loginfo("Number of hot pixels: " + str(counter))
      return True
    else:
      return False

  def execute(self, timeout = 5.0):
    timeout_ros = rospy.Duration(timeout)
    time_start = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() - time_start < timeout_ros:
      if self.check_hot_zone():
        return True
      rospy.sleep(RATE)
    return False

# def main():
#   #cv2.namedWindow('image')
#   ic = terabee()
#   try:
#     ic.execute()
#   except KeyboardInterrupt:
#     print("Shutting down")

# if __name__ == '__main__':
#   main()
