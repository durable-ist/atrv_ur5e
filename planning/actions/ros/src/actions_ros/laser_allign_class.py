#!/usr/bin/env python

import roslib
import rospy
import laser_line_extraction.msg
import numpy
import math
import geometry_msgs.msg




color_dict = {"Red" : 0.3,
              "Green" : 0.6,
              "Blue" : 1.2,
              "Orange" : 1.8,
              "green" : 0.6,
              "red" : 0.3,
              "blue" : 1.2,
              "orange": 1.8
              }

class Allign_Pick():

  def __init__(self):

      self.sub = rospy.Subscriber('/line_segments', laser_line_extraction.msg.LineSegmentList, self.line_segments_cb)
      self.cmd_vel_pub = rospy.Publisher("/rflex/atrvjr_node/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

      # self.brick = color_dict[rospy.get_param('brick_to_place')]
      self.brick = color_dict['orange']



      self.counter = 0
      self.condition_stop = 12
      self.no_line_counter=0

      self.executee = False
      self.finish = False
      self.flag=1

  def line_segments_cb(self, msg):

    self.line_segment_list = msg

  def execute(self):

    while not rospy.is_shutdown():

      #self.brick = color_dict[rospy.get_param('brick_to_place')]

      #rospy.sleep(0.2)

      rate = rospy.Rate(5)
      rate.sleep()

      self.line_segs = self.line_segment_list.line_segments

      vel_msg=geometry_msgs.msg.Twist()

      if len(self.line_segs) == 0:
        self.flag = 6

      elif len(self.line_segs) == 0 and self.flag > 1:
        print "angle_goal"
        self.flag = 7

      elif self.line_segs[0].radius < 0.4 and self.flag < 3 or self.flag ==8:

        print "line is super close"

        self.angle_goal = self.line_segs[0].angle
        self.flag = 8

      else:
        closest_line = 999 #random big number
        line_populated = False
          
        for line in self.line_segs:

          if line.radius<2.5:
            start = line.start   #start is left
            end = line.end     #end is right

            diff_x=abs(start[0]-end[0])
            diff_y=abs(start[1]-end[1])
            length = numpy.sqrt(diff_x**2 + diff_y**2)

            if length <= self.brick+0.1:
              middle_point_x=(start[0]+end[0])/2
              middle_point_y=(start[1]+end[1])/2  

              distance_middle_point = numpy.sqrt(middle_point_x**2 + middle_point_y**2)
              print "ANGULO", line.angle

              try:
                if distance_middle_point < closest_line and self.angle_commited-0.3 <= abs(line.angle) <= self.angle_commited+0.3:
                  print "COMMITED", self.angle_commited
                  closest_line = distance_middle_point
                  chosen_line = line
                  line_populated = True

              except:
                print "foi para o except pq nao tem angle_commited"
                if distance_middle_point < closest_line:
                  closest_line = distance_middle_point
                  chosen_line = line
                  line_populated = True

        print "Closest_line", closest_line

        if line_populated:

          start = chosen_line.start   #start is left
          end = chosen_line.end     #end is right

          diff_x=abs(start[0]-end[0])
          diff_y=abs(start[1]-end[1])
          length = numpy.sqrt(diff_x**2 + diff_y**2)

          middle_point_x=(start[0]+end[0])/2
          middle_point_y=(start[1]+end[1])/2  

          print "X", middle_point_x, "Y", middle_point_y
          print "length", length
          print "line radius", chosen_line.radius
          print "line angle", chosen_line.angle

        elif not line_populated and self.flag > 2:
          print "line not populated because no line satisfied the conditions ANYMORE"
          self.flag = 7

        else:
          print "not populated and no commited line"
          self.no_line_counter=self.no_line_counter+1
          if self.no_line_counter > 5:
            self.flag = -1

      print "FLAG", self.flag

      if self.flag == 1:

        try:
          vector_y=middle_point_x - start[0]
          vector_x=middle_point_y - start[1]
          if vector_x > 0:
            vector_x = -vector_x
          print middle_point_x + vector_x/2
          print middle_point_y + vector_y/2

          diff_angle = math.atan2(middle_point_y + vector_y*(0.3/(length/2)), middle_point_x + vector_x*(0.3/(length/2)))
          self.angle_goal = - diff_angle + chosen_line.angle

          print "angle goal", self.angle_goal
          self.flag=2
          rospy.sleep(1)
        except:
          pass

      if self.flag == 2:

        if chosen_line.angle > self.angle_goal+0.1:
          vel_msg.linear.x = 0
          vel_msg.angular.z = -0.15
          self.angle_commited= abs(chosen_line.angle)
        if chosen_line.angle < self.angle_goal-0.1:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0.15
          self.angle_commited= abs(chosen_line.angle)
        if chosen_line.angle>self.angle_goal-0.1 and chosen_line.angle<self.angle_goal+0.1: #este 0.03 depende da vel.angular
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0
          self.flag=3
          self.angle_commited= abs(chosen_line.angle)
        
        self.cmd_vel_pub.publish(vel_msg)

      if self.flag == 3:

        vel_msg.linear.x = 0.2

        stop = 0.3
        if chosen_line.radius < (stop + abs(chosen_line.angle)/12):
          vel_msg.linear.x = 0
          self.flag = 4
          self.angle_commited= None

        self.cmd_vel_pub.publish(vel_msg)

      if self.flag == 4:

        if chosen_line.angle < - 0.035:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0.15
        if chosen_line.angle > 0.035:
          vel_msg.linear.x = 0
          vel_msg.angular.z = -0.15
        if chosen_line.angle > -0.035 and chosen_line.angle<0.035:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0
          self.flag= 5
        self.cmd_vel_pub.publish(vel_msg)

      if self.flag == 7: 

          self.counter=self.counter+1
          vel_msg.linear.x = 0.2
          self.cmd_vel_pub.publish(vel_msg)
          
          print "Counter", self.counter
          print "Angle goal", self.angle_goal

          if self.counter > abs(self.angle_goal) * self.condition_stop:
            self.angle_commited = None
            try:
                #if self.line_segs[0].radius< 0.3:
                #self.angle_goal = self.line_segs[0].angle
              print "esta no try na flag 7"
              print "chosen_line angle", chosen_line.angle
              self.angle_goal = chosen_line.angle
            except:
              pass

            if self.angle_goal < - 0.025:
              vel_msg.linear.x = 0
              vel_msg.angular.z = 0.15
            if self.angle_goal > 0.025:
              vel_msg.linear.x = 0
              vel_msg.angular.z = -0.15
            if self.angle_goal > -0.025 and self.angle_goal<0.025:
              vel_msg.linear.x = 0
              vel_msg.angular.z = 0
              self.flag= 5
            self.cmd_vel_pub.publish(vel_msg)

      if self.flag == 8:

        print "angle_goal", self.angle_goal

        if self.angle_goal < - 0.025:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0.15
        if self.angle_goal > 0.025:
          vel_msg.linear.x = 0
          vel_msg.angular.z = -0.15
        if self.angle_goal > -0.025 and self.angle_goal<0.025:
          vel_msg.linear.x = 0
          vel_msg.angular.z = 0
          self.flag= 5
        self.cmd_vel_pub.publish(vel_msg)
    
      if self.flag == 6:
        print "no line detected"
        self.finish = True 
        return

      if self.flag == 5:
        self.finish = True
        self.flag = 1
        rospy.set_param('approach_wall', True)
        return "success"

      print "-------------------------------------------------------------------------------------"




class Allign_Place():

  def __init__(self):

      self.sub = rospy.Subscriber('line_segments', laser_line_extraction.msg.LineSegmentList, self.line_segments_cb)
      self.cmd_vel_pub = rospy.Publisher("rflex/atrvjr_node/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
      
      self.brick_to_place = 1.5
      self.wall_length = 1.8
      self.level = 1
      self.line_length=1.8
      self.over = False 

      self.counter = 0
      self.no_line_counter = 0
      self.condition_stop = 18
      self.executee = False
      self.finish = False

      if self.level > 1 or self.over:
      	self.line_length = 3.6   # length of a complete layer

      self.flag=0

  def line_segments_cb(self, msg):

    self.line_segment_list = msg

  def execute(self):

    while not rospy.is_shutdown():


        #self.brick_to_place = color_dict[rospy.get_param('brick_to_place')]
        #self.wall_length = rospy.get_param('wall_len')
        #self.level = rospy.get_param('level')
        #self.line_length = rospy.get_param('wall_len')
        #self.over = rospy.get_param('over')
    
        print "BRICK TO PLACEEEEEE: " + str(self.brick_to_place)
        print "WALL LENGTHHHHH: " + str(self.wall_length)
        print "LEVELLLL: " + str(self.level)
        print "OVERRRR: " + str(self.over)

        rate = rospy.Rate(5)
        rate.sleep()

        self.line_segs = self.line_segment_list.line_segments

        vel_msg=geometry_msgs.msg.Twist()

        if len(self.line_segs) == 0 and not self.angle_goal:
          self.flag = 6

        elif len(self.line_segs) == 0 and self.flag > 1:  #nao ve nenhuma linha mas ja tem um angle_goal
          print "angle_goal"
          self.flag = 7

        elif self.line_segs[0].radius < 0.4 and self.flag < 3 or self.flag ==8:

          print "line is super close"
          print "line.radius", self.line_segs[0].radius
          print "line.angle", self.line_segs[0].angle

          self.angle_goal = self.line_segs[0].angle
          self.flag = 8

        else:
          closest_line = 999 #random big number
          line_populated = False
            
          for line in self.line_segs:

            if line.radius<2.5:
              start = line.start   #start e o ponto mais a esquerda
              end = line.end     #end  o ponto mais a direita

              diff_x=abs(start[0]-end[0])
              diff_y=abs(start[1]-end[1])
              length = numpy.sqrt(diff_x**2 + diff_y**2)

              if length <= self.line_length+0.1:
                middle_point_x=(start[0]+end[0])/2
                middle_point_y=(start[1]+end[1])/2  

                distance_middle_point = numpy.sqrt(middle_point_x**2 + middle_point_y**2)
                print "ANGULO", line.angle

                try:
                  if distance_middle_point < closest_line and self.angle_commited-0.3 <= abs(line.angle) <= self.angle_commited+0.3:
                    print "COMMITED", self.angle_commited
                    closest_line = distance_middle_point
                    chosen_line = line
                    line_populated = True

                except:
                  print "foi para o except pq nao tem angle_commited"
                  if distance_middle_point < closest_line:
                    closest_line = distance_middle_point
                    chosen_line = line
                    angle_commited = None
                    line_populated = True

          print "Closest_line", closest_line

          if line_populated:

            start = chosen_line.start   #start is left
            end = chosen_line.end     #end is right

            diff_x=abs(start[0]-end[0])
            diff_y=abs(start[1]-end[1])
            length = numpy.sqrt(diff_x**2 + diff_y**2)

            middle_point_x=(start[0]+end[0])/2
            middle_point_y=(start[1]+end[1])/2  

            print "X", middle_point_x, "Y", middle_point_y
            print "length", length
            print "line radius", chosen_line.radius
            print "line angle", chosen_line.angle

          elif not line_populated and self.flag > 2:  #line not populatedw
            print "line not populated because no line satisfied the conditions of distance_middle_point or angle_commited"
            self.flag = 7

          else:
            print "not populated and no commited line"
            self.no_line_counter=self.no_line_counter+1
            if self.no_line_counter > 6:
              self.flag = -1


        print "FLAG", self.flag

        if self.flag == 0:

          if chosen_line.angle < - 0.03:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.15
          if chosen_line.angle > 0.03:
            vel_msg.linear.x = 0
            vel_msg.angular.z = -0.15
          if chosen_line.angle > -0.03 and chosen_line.angle<0.03:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.flag= 1

          self.cmd_vel_pub.publish(vel_msg)

        if self.flag == 1:

          try:

            vector_y=middle_point_x - start[0]
            vector_x=middle_point_y - start[1]
            if vector_x > 0:
              vector_x = -vector_x
            print middle_point_x + vector_x/2
            print middle_point_y + vector_y/2

            side = start[1]
            ###########################reference_side, reference_brick, brick_to_place, wall_length###########################################

            point_value_y = - (side + self.wall_length + self.brick_to_place/2) 

            diff_angle = math.atan2(point_value_y, middle_point_x - 0.35)
            self.angle_goal = diff_angle

            print "angle goal", self.angle_goal
            self.flag=2
            rospy.sleep(1)

          except:
            pass

        if self.flag == 2:

          if chosen_line.angle > self.angle_goal+0.1:
            vel_msg.linear.x = 0
            vel_msg.angular.z = -0.15
            self.angle_commited= abs(chosen_line.angle)
          if chosen_line.angle < self.angle_goal-0.1:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.15
            self.angle_commited= abs(chosen_line.angle)
          if chosen_line.angle>self.angle_goal-0.1 and chosen_line.angle<self.angle_goal+0.1: #este 0.03 depende da vel.angular
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.flag=3
            self.angle_commited= abs(chosen_line.angle)
          self.cmd_vel_pub.publish(vel_msg)

        if self.flag == 3:

          vel_msg.linear.x = 0.2
     
          if chosen_line.radius < 0.35:
            vel_msg.linear.x = 0
            self.flag = 4
            self.angle_commited= None

          self.cmd_vel_pub.publish(vel_msg)

        if self.flag == 4:

          if chosen_line.angle < - 0.035:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.15
          if chosen_line.angle > 0.035:
            vel_msg.linear.x = 0
            vel_msg.angular.z = -0.15
          if chosen_line.angle > -0.035 and chosen_line.angle<0.035:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.flag= 5
          self.cmd_vel_pub.publish(vel_msg)

        if self.flag == 7: 

          self.counter=self.counter+1
          
          print "Counter", self.counter
          print "Angle goal", self.angle_goal

          if self.counter > abs(self.angle_goal) * self.condition_stop:
            self.angle_commited = None
            try:
                #if self.line_segs[0].radius< 0.3:
                #self.angle_goal = self.line_segs[0].angle
                print "esta no try na flag 7"
                print "chosen_line angle", chosen_line.angle
                if abs(chosen_line.angle) < 0.5: 
                  self.angle_goal = chosen_line.angle
                print "angle goal do try", self.angle
            except:
              pass

            if self.angle_goal < - 0.04:
              vel_msg.linear.x = 0
              vel_msg.angular.z = 0.15
            if self.angle_goal > 0.04:
              vel_msg.linear.x = 0
              vel_msg.angular.z = -0.15
            if self.angle_goal > -0.04 and self.angle_goal<0.04:
              vel_msg.linear.x = 0
              vel_msg.angular.z = 0
              self.flag= 5
            self.cmd_vel_pub.publish(vel_msg)

          else:
            vel_msg.linear.x = 0.2
            self.cmd_vel_pub.publish(vel_msg)
            
        if self.flag == 8:

          if self.angle_goal < - 0.04:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0.15
          if self.angle_goal > 0.04:
            vel_msg.linear.x = 0
            vel_msg.angular.z = -0.15
          if self.angle_goal > -0.04 and self.angle_goal<0.04:
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.flag= 5
          self.cmd_vel_pub.publish(vel_msg)
      
        if self.flag == 6:
          print "no line detected"
          self.finish = True
          return 

        if self.flag == 5:
          self.finish = True
          rospy.set_param('approach_wall', False)

          self.flag = 0
          return "success"


        print "-------------------------------------------------------------------------------------"


def main():

    rospy.init_node('laser_brick_detection')
    n = Allign_Pick()
    n.execute()
    # def __init__(self, brick_to_place, wall_length, level, over):
if __name__ == '__main__':
    main()
