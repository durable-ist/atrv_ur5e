#!/usr/bin/env python

import roslib
import rospy
import laser_line_extraction.msg
import numpy
import math
import geometry_msgs.msg

class line_node():

    def __init__(self):

       # self.pub = rospy.Publisher('attached', GripperAttached, queue_size=1)
        self.sub = rospy.Subscriber('/line_segments', laser_line_extraction.msg.LineSegmentList, self.line_segments_cb)

        self.cmd_vel_pub = rospy.Publisher("/mbzirc2020_0/base_controller/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        
        self.flag=0

    def line_segments_cb(self, msg):

	self.line_segment_list = msg

	self.line_segs = self.line_segment_list.line_segments

	for line in self.line_segs:

		if line.radius < 3 and abs(line.angle) < 0.5:

			print line

			start = line.start   #start e o ponto mais a esquerda
			end = line.end     #end e o ponto mais a direita
			
			diff_x=abs(start[0]-end[0])
			diff_y=abs(start[1]-end[1])

			middle_point_x=(start[0]+end[0])/2
			middle_point_y=-(start[1]+end[1])/2 #aqui esta menos pq o frame do hokuyo esta invertido

			point_30_y = -(end[1]+0.3)

			print "X", middle_point_x, "Y", middle_point_y

			length = numpy.sqrt(diff_x**2 + diff_y**2)

			print "length", length
			print "line radius", line.radius
			print "line angle", line.angle

			vel_msg=geometry_msgs.msg.Twist()

			if self.flag == 0:

				if line.angle < - 0.02:

					vel_msg.linear.x = 0
					vel_msg.angular.z = 0.15

				if line.angle > 0.02:

					vel_msg.linear.x = 0
					vel_msg.angular.z = -0.15

				if line.angle > -0.02 and line.angle<0.02:

					vel_msg.linear.x = 0
					vel_msg.angular.z = 0
					self.flag= 1

				self.cmd_vel_pub.publish(vel_msg)

			print self.flag

			if self.flag == 1:

				diff_angle = math.atan2(middle_point_y,middle_point_x-0.2)

				# diff_angle = math.atan2(point_30_y,middle_point_x-0.2)    #0.2 para ficar 20cm

				# hipotenusa=sqrt(point_30_y**,(middle_point_x-0.2)**)

				self.angle_goal = diff_angle
				print "angle goal", self.angle_goal

				rospy.sleep(1)
				self.flag=2
				

			if self.flag == 2:

				if line.angle > self.angle_goal:

					vel_msg.linear.x = 0
					vel_msg.angular.z = -0.15

				if line.angle < self.angle_goal:

					vel_msg.linear.x = 0
					vel_msg.angular.z = 0.15

				if abs(line.angle)>abs(self.angle_goal)-0.03: #este 0.03 depende da vel.angular

					vel_msg.linear.x = 0
					vel_msg.angular.z = 0
					self.flag=3

				self.cmd_vel_pub.publish(vel_msg)

			if self.flag == 3:

				vel_msg.linear.x = 0.2

				# if abs(middle_point_y)-0.15 <= middle_point_x <= abs(middle_point_y)+0.05:   #CHANGE THIS

				if line.radius < 0.3:
					vel_msg.linear.x = 0
					self.flag = 4



				self.cmd_vel_pub.publish(vel_msg)

			if self.flag == 4:

				if line.angle < - 0.02:

					vel_msg.linear.x = 0
					vel_msg.angular.z = 0.15

				if line.angle > 0.02:

					vel_msg.linear.x = 0
					vel_msg.angular.z = -0.15

				if line.angle > -0.02 and line.angle<0.02:

					vel_msg.linear.x = 0
					vel_msg.angular.z = 0
					self.flag= 5

				self.cmd_vel_pub.publish(vel_msg)


			print "acabou o ciclo forrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"

def main():

    rospy.init_node('laser_brick_detection')
    n = line_node()
    rospy.spin()

if __name__ == '__main__':
    main()

