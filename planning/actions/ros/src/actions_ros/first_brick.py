
import sys
import rospy
import numpy as np
import cv2
import tf
import heapq

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

yellowLower = (0, 10, 150)
yellowUpper = (100, 255, 255)
purpleLower = (154, 40, 100)
purpleUpper = (174, 255, 255)

class first_brick:

  def __init__(self):

    self.bridge = CvBridge()
    rospy.wait_for_message('camera/color/camera_info', CameraInfo)
    self.image_sub = rospy.Subscriber("camera/color/image_rect_color",Image,self.callback_color, queue_size=1)
    self.depth_sub = rospy.Subscriber("camera/depth/image_rect_raw",Image,self.callback_depth, queue_size=1)
    self.camera_sub = rospy.Subscriber('camera/color/camera_info', CameraInfo, self.callback_camera, queue_ize=1)
    self.image = Image()
    self.tf_pub = tf.TransformBroadcaster()
    self.list_tf = tf.TransformListener() 
    self.depth_image = Image()
    rospy.wait_for_message("camera/color/image_rect_color", Image)
    rospy.sleep(1)

  def callback_camera(self,data):
    # set values for local variables
    self.frame = data.header.frame_id
    self.cam_height = data.height
    self.cam_width = data.width
    self.inv_fx = 1/data.K[0]
    self.inv_fy = 1/data.K[4]
    self.center_x = data.K[2]
    self.center_y = data.K[5]

  def callback_depth(self,depth_data):
    try:
      self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")
    except CvBridgeError as e:
      print(e)

  def callback_color(self,data):
    try:
      self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.frame = data.header.frame_id
    except CvBridgeError as e:
      print(e)

  def converte_xy(self, pixel_x, pixel_y):
    z = float(self.depth_image[int(pixel_y),int(pixel_x)])
    rx =(pixel_x-self.center_x)*z*self.inv_fx/1000 #conversao de imagem para ponto real
    ry =(pixel_y-self.center_y)*z*self.inv_fy/1000
    return rx, ry, z/1000

  def create_pose(self, x, y, z, frame, angle):
    point = PoseStamped()
    point.pose.position.x = x
    point.pose.position.y = y
    point.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(-np.pi, 0, angle*0.0175)
    point.pose.orientation.x = quaternion[0]
    point.pose.orientation.y = quaternion[1]
    point.pose.orientation.z = quaternion[2]
    point.pose.orientation.w = quaternion[3]
    point.header.stamp = rospy.Time(0)
    point.header.frame_id = frame
    return point


  def create_point(self, x, y, z, frame, angle):
    point = PointStamped()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    point.header.stamp = rospy.Time(0)
    point.header.frame_id = frame
    return point
  
  def closest_point(self, box, ctr):
    M = cv2.moments(ctr)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    closest = np.inf
    index = 0
    for i in range(4):
      dist = np.sqrt((cx-box[i][0])**2 + (cy-box[i][1])**2)
      if dist < closest:
        closest = dist
        index = i
    return box[index], index

  def side(self, points, index):
    x = []
    for i in box:
      x.append(i[index])
    pts_x = heapq.nlargest(2, x)
    return (box[x.index(pts_x[0])][0]+box[x.index(pts_x[1])][0])/2, (box[x.index(pts_x[0])][1]+box[x.index(pts_x[1])][1])/2

  def check_corner(self, x, y, width, height):
    if (x == 0 and y == 0):
      return False
    if (x == 0 and y == height -1):
      return False
    if (x == width -1 and y == 0):
      return False
    if (x == width -1 and y == height -1):
      return False
    return True

  def execute(self):
    imhsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    mask = (cv2.inRange(imhsv, yellowLower, yellowUpper)+cv2.inRange(imhsv, purpleLower, purpleUpper))
    if mask == []:
      print "depth filter failed"
      return
    _, ctr, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if ctr != []:
      contorno = max(ctr, key=cv2.contourArea)
      if cv2.contourArea(contorno) < MIN_CTR:
        return
      rect = cv2.minAreaRect(contorno)
      box = cv2.boxPoints(rect)
      box = np.int0(box)
      # get width and height of the detected rectangle
      width = int(rect[1][0])
      height = int(rect[1][1])
      src_pts = box.astype("float32")
      # corrdinate of the points in box points after the rectangle has been
      # straightened
      dst_pts = np.array([[0, height-1],
                          [0, 0],
                          [width-1, 0],
                          [width-1, height-1]], dtype="float32")
      try:
        # the perspective transformation matrix
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        # directly warp the rotated rectangle to get the straightened rectangle
        self.warped = cv2.warpPerspective(self.update_img, M, (width, height))
        gray = cv2.cvtColor(self.warped, cv2.COLOR_BGR2GRAY)
        black_pixels = width*height - cv2.countNonZero(gray)
      except:
        return
      if (cv2.contourArea(contorno) < 0.7*(width*height - black_pixels)): 
        print "its a corner"
        corner, idx = self.closest_point(box, contorno)
        if self.check_corner(corner[0],corner[1]):
          return
        ponto = self.converte_xy(corner[0],corner[1])
        new_pose = self.create_pose(ponto[0],ponto[1],ponto[2], self.frame, rect[2]+90*idx)
        self.list_tf.waitForTransform(self.frame, "/odom", new_pose.header.stamp,rospy.Duration(1))
        new_pose2 = self.list_tf.transformPose("/odom",new_pose)
        self.tf_pub.sendTransform((new_pose2.pose.position.x, new_pose2.pose.position.y, new_pose2.pose.position.z),
                     (new_pose2.pose.orientation.x,new_pose2.pose.orientation.y,new_pose2.pose.orientation.z,new_pose2.pose.orientation.w),
                     rospy.Time.now(),
                     "/L_shape",
                     "/odom")
        return
      else:
        # lado_x, lado_y, self.z = self.converte_xy(self.side(box, 0))
        print "no corner"
        # return self.create_point(lado_x, lado_y, self.z, self.frame)
    else:
      return
