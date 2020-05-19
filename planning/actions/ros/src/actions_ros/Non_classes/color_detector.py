
import sys
import rospy
import numpy as np
import cv2

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError



CTR_TRESHOLD = 2000
RATE = 1
DEFAULT_Y_MIN = -10
DEFAULT_Y_MAX = 10
DEFAULT_X_MIN = -10
DEFAULT_X_MAX = 10
#HSV values ToDo: put in a yaml file
redLower = (0, 50, 0)
redUpper = (12, 255, 255)
redLower2 = (170, 50, 0)
redUpper2 =(180, 255, 255)
blueLower = (95, 180, 0)
blueUpper = (118, 255, 255)
orangeLower = (15, 180, 140)
orangeUpper = (20, 255, 235)
#greenLower = (30, 50, 50)
#greenUpper = (70, 255, 255)

greenLower = (180, 50, 50)
greenUpper = (180, 255, 255)
#whiteLower = (0, 0, 150)
#whiteUpper = (255, 155, 255)
whiteLower = (0, 0, 180)
whiteUpper = (255, 70, 255)

class color_detector:

  def __init__(self):

    rospy.init_node('color_detector', anonymous=True)    
    self.rate = rospy.Rate(RATE) 
    self.bridge = CvBridge()
    rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
    self.image_sub = rospy.Subscriber("/camera/color/image_rect_color",Image,self.callback_color, queue_size=1)
    self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw",Image,self.callback_depth, queue_size=1)
    self.camera_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.callback_camera, queue_size=1)
    #self.detector_pub = rospy.Publisher("/color_detection", String)
    #self.box_pub = rospy.Publisher("/box_dims", Float32MultiArray)

    
    self.image = Image()
    self.depth_image = Image()
    self.colors = ["Red","Blue","Orange","Green", "white"]
    rospy.wait_for_message("/camera/color/image_rect_color", Image)
    self.update_img = self.image

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
    except CvBridgeError as e:
      print(e)
    
  def execute(self):
    BOX_MARGIN = 5
    pub_msg = "no countour found"
    mask = [] #order is Red, Blue, Orange, Green
    imhsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
    mask.append(cv2.inRange(imhsv, redLower, redUpper)+cv2.inRange(imhsv, redLower2, redUpper2))
    mask.append(cv2.inRange(imhsv, blueLower, blueUpper))
    mask.append(cv2.inRange(imhsv, orangeLower, orangeUpper))
    mask.append(cv2.inRange(imhsv, greenLower, greenUpper))
    mask.append(cv2.inRange(imhsv, whiteLower, whiteUpper))
    for idx, area in enumerate(mask):
      self.update_img = self.image
      area = cv2.erode(area, None, iterations=2)
      area = cv2.dilate(area, None, iterations=2)
      _, ctr, hierarchy = cv2.findContours(area, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      if ctr != []:
        # try: hierarchy = hierarchy[0]
        # except: hierarchy = []
        # min_x = 639
        # min_y = 479
        # max_x = max_y = 0
        # area = 0
        # for contour, hier in zip(ctr, hierarchy):
        #   (x,y,w,h) = cv2.boundingRect(contour)
        #   min_x, max_x = min(x, min_x), max(x+w, max_x)
        #   min_y, max_y = min(y, min_y), max(y+h, max_y)
        #   area = cv2.contourArea(contour) + area
        contorno = max(ctr, key=cv2.contourArea)
        #print area
        if cv2.contourArea(contorno) < CTR_TRESHOLD:
          pass
        else:
          #cv2.rectangle(self.update_img, (min_x, min_y), (max_x, max_y), (255, 0, 0), 2)
          # x,y,z,h = cv2.boundingRect(contorno)
          # cv2.rectangle(self.update_img,(x,y),(x+z,y+h),(0,255,0),2)
          rect = cv2.minAreaRect(contorno)
          box = cv2.boxPoints(rect)
          box = np.int0(box)
          cv2.drawContours(self.update_img,[box],0,(0,0,255),2)
          cv2.putText(self.update_img,self.colors[idx], (box[1,0],box[1,1]), cv2.FONT_HERSHEY_SIMPLEX, 1, 0)
          x,y,z,h = cv2.boundingRect(contorno)
          cv2.rectangle(self.update_img,(x+z-30,y+40),(x+z-15,y+h-40),(255,255,255),2)
          # pt1 = self.converte_xy(box[0,0],box[0,1])
          # pt2 = self.converte_xy(box[1,0],box[1,1])
          # pt3 = self.converte_xy(box[2,0],box[2,1])
          # pt4 = self.converte_xy(box[3,0],box[3,1])
          # min_x, max_x = min(box[:,0]), max(box[:,0])
          # min_y, max_y = min(box[:,1]), max(box[:,1])
          # x_min_real, y_min_real = self.converte_xy(min_x-BOX_MARGIN,min_y-BOX_MARGIN)
          # x_max_real, y_max_real = self.converte_xy(max_x+BOX_MARGIN,max_y+BOX_MARGIN)
          # message = Float32MultiArray()
          # message.data = [pt1,pt2,pt3,pt4]
          # message.layout.dim = 2 

          # self.box_pub.publish(message)
          # pub_msg = self.colors[idx]
          # #break
        #config = self.client.update_configuration(params)
        if idx == 3:
          cv2.drawContours(self.image, ctr, -1, (0,255,0), 3)
    #self.detector_pub.publish(pub_msg)

  def converte_xy(self, pixel_x, pixel_y):
    pixel_x = max(0, pixel_x)
    pixel_x = min(639, pixel_x)
    pixel_y = max(0, pixel_y)
    pixel_y = min(479, pixel_y)
    z = self.depth_image[int(pixel_y),int(pixel_x)]
    rx =(pixel_x-self.center_x)*z*self.inv_fx/1000 #conversao de imagem para ponto real
    ry =(pixel_y-self.center_y)*z*self.inv_fy/1000
    return rx, ry

def main(args):
  cv2.namedWindow('image')
  ic = color_detector()
  try:
    while not rospy.is_shutdown():
      ic.execute()
      cv2.imshow("image", ic.update_img)
      cv2.waitKey(3)
      ic.rate.sleep()
  # try:
  #   rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
