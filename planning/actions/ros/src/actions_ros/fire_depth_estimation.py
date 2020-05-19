#!/usr/bin/env python
import sys
import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import  PointStamped
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes



RATE = 0.5
OFFSET = 10


class fire_depth:

  def __init__(self):

    rospy.init_node('fire_depth', anonymous=True)
    self.rate = rospy.Rate(RATE) 
    self.bridge = CvBridge()
    rospy.wait_for_message('camera/color/camera_info', CameraInfo)
    self.box_sub = rospy.Subscriber("darknet_ros/bounding_boxes",BoundingBoxes,self.callback_box, queue_size=1)
    self.camera_sub = rospy.Subscriber('camera/color/camera_info', CameraInfo, self.callback_camera, queue_size=1)
    self.depth_image = rospy.Subscriber("camera/aligned_depth_to_color/image_raw",Image,self.callback_depth, queue_size=1)
    self.image_sub = rospy.Subscriber("camera/color/image_raw",Image,self.callback_color, queue_size=1)
    self.depth_pub = rospy.Publisher("depth_point", PointStamped,queue_size=5)

    rospy.sleep(1)

  def callback_camera(self,data):
    self.frame = data.header.frame_id
    self.cam_height = data.height
    self.cam_width = data.width
    self.inv_fx = 1/data.K[0]
    self.inv_fy = 1/data.K[4]
    self.center_x = data.K[2]
    self.center_y = data.K[5]

  def callback_depth(self,data):
    try:
      self.depth = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)

  def callback_color(self,data):
    try:
      self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      #self.frame = data.header.frame_id
    except CvBridgeError as e:
      print(e)
    
  def callback_box(self, data):
    max_box = max(data.bounding_boxes, key=lambda x: x.probability)
    self.frame = data.image_header.frame_id
    self.execute(max_box)
    

  def execute(self, box):
    self.update_img = self.image
    crop_img = self.depth[box.ymin+OFFSET:box.ymax-OFFSET, box.xmin+OFFSET:box.xmax-OFFSET]
    cv2.rectangle(self.update_img,(box.xmin+OFFSET,box.ymin+OFFSET),(box.xmax-OFFSET,box.ymax-OFFSET),(0,255,255),2)
    mean = crop_img[np.nonzero(crop_img)].mean()
    p_x, p_y = self.converte_xy((box.xmin+box.xmax)/2, (box.ymin+box.ymax)/2, mean/1000)
    self.depth_pub.publish(self.create_point(p_x, p_y, mean/1000, self.frame))
    # cv2.imshow("image", self.update_img)
    # cv2.waitKey(3)

  def create_point(self, x, y, z, frame):
    point = PointStamped()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    point.header.stamp = rospy.Time(0)
    point.header.frame_id = frame
    return point

  def converte_xy(self, pixel_x, pixel_y, z = 0):
    pixel_x = max(0, pixel_x)
    pixel_x = min(639, pixel_x)
    pixel_y = max(0, pixel_y)
    pixel_y = min(479, pixel_y)
    if z == 0:
      z = float(self.depth_image[int(pixel_y),int(pixel_x)])/1000
    rx =(pixel_x-self.center_x)*z*self.inv_fx #conversao de imagem para ponto real
    ry =(pixel_y-self.center_y)*z*self.inv_fy
    return rx, ry

def main():
  #cv2.namedWindow('image')
  ic = fire_depth()
  # try:
    # while not rospy.is_shutdown():
    #   ic.execute()
  # cv2.imshow("image", ic.update_img)
  # cv2.waitKey(3)
    #   ic.rate.sleep()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main()