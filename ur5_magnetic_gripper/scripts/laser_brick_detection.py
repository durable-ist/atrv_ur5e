#!/usr/bin/env python

import roslib
import rospy
import laser_line_extraction.msg
import numpy
import math
import geometry_msgs.msg


#INPUTS
# brick_to_be_placed (0.1 for pick)
# diff_angle (point_value_y for placing on the side; change the value at the end for increasing/decreasing distance to line)
# self.condition_stop (for stoping when no line)


class line_node():

    def __init__(self):

       # self.pub = rospy.Publisher('attached', GripperAttached, queue_size=1)
        self.sub = rospy.Subscriber('/line_segments', laser_line_extraction.msg.LineSegmentList, self.line_segments_cb)

        self.cmd_vel_pub = rospy.Publisher("/mbzirc2020_0/base_controller/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)

        self.goal_pose_pub = rospy.Publisher("/mbzirc2020_0/laser_brick_detection/goal_pose", geometry_msgs.msg.PoseStamped, queue_size=1)
        
        self.flag=1

        self.approach_wall = True
        self.counter = 0
        self.brick_to_be_placed = 1.2
        self.condition_stop = 18

    def line_segments_cb(self, msg):

	self.line_segment_list = msg

	self.line_segs = self.line_segment_list.line_segments

	if len(self.line_segs) == 0:

		print "line segments list is EMPTY"

		self.flag = 6

		print self.angle_goal

	else:

		lowest = 999 #random big number
		line_populated = False
		vel_msg=geometry_msgs.msg.Twist()

		goal_pose=geometry_msgs.msg.PoseStamped()

		goal_pose.header.stamp=rospy.Time.now()
		goal_pose.header.frame_id = "hokuyo"
		goal_pose.pose.position.z= 0
		goal_pose.pose.orientation.x= 0
		goal_pose.pose.orientation.y= 0
		goal_pose.pose.orientation.z= 0
		goal_pose.pose.orientation.w= 1

		for line in self.line_segs:

			if line.radius<3:

				start = line.start   #start e o ponto mais a esquerda
				end = line.end     #end e o ponto mais a direita
				
				diff_x=abs(start[0]-end[0])
				diff_y=abs(start[1]-end[1])

				middle_point_x=(start[0]+end[0])/2
				middle_point_y=(start[1]+end[1])/2 #aqui esta menos pq o frame do hokuyo esta invertido

				distance_middle_point = numpy.sqrt(middle_point_x**2 + middle_point_y**2)

				print "ANGULO", line.angle


				try:

					if distance_middle_point < lowest and self.angle_commited-0.05 <= abs(line.angle) <= self.angle_commited+0.05:

						print "COMMITED", self.angle_commited

						lowest = distance_middle_point
						chosen_line = line
						line_populated = True
				except:
					print "foi para o except pq nao tem angle_commited"

					if distance_middle_point < lowest:

						lowest = distance_middle_point
						chosen_line = line
						line_populated = True

		print "lowest", lowest

		if line_populated:

			start = chosen_line.start   #start e o ponto mais a esquerda
			end = chosen_line.end     #end e o ponto mais a direita
			
			diff_x=abs(start[0]-end[0])
			diff_y=abs(start[1]-end[1])
			length = numpy.sqrt(diff_x**2 + diff_y**2)
			length=1

			middle_point_x=(start[0]+end[0])/2
			middle_point_y=(start[1]+end[1])/2 #aqui esta menos pq o frame do hokuyo esta invertido

			# diff_x_mid= abs(start[0]-middle_point_x)
			# diff_y_mid= abs(start[1]-middle_point_y)
			# length_mid = numpy.sqrt(diff_x_mid**2 + diff_y_mid**2)

			# goal_point_x= middle_point_x + (0.25*(start[1]-middle_point_y))/(length/2)
			# goal_point_y= middle_point_y + (0.25*(middle_point_x-start[0]))/(length/2)

			point_value_y = -(start[1] - self.brick_to_be_placed/2)  

			print "X", middle_point_x, "Y", middle_point_y
			print "length", length
			print "line radius", chosen_line.radius
			print "line angle", chosen_line.angle

		else:

			print "line not populated"
			self.flag = 6

	print "FLAG", self.flag

	# if self.flag == 0:

	# 	if chosen_line.angle < - 0.02:
	# 		vel_msg.linear.x = 0
	# 		vel_msg.angular.z = 0.15
	# 	if chosen_line.angle > 0.02:
	# 		vel_msg.linear.x = 0
	# 		vel_msg.angular.z = -0.15
	# 	if chosen_line.angle > -0.02 and chosen_line.angle<0.02:
	# 		vel_msg.linear.x = 0
	# 		vel_msg.angular.z = 0
	# 		self.flag= 1

	# 	self.cmd_vel_pub.publish(vel_msg)

	if self.flag == 1:

		vector_y=middle_point_x - start[0]
		vector_x=middle_point_y - start[1]

		if vector_x > 0:
			vector_x = -vector_x
		# if vector_x < 0:
		# 	vector_y = -vector_y

		print middle_point_x + vector_x/2
		print middle_point_y + vector_y/2


		diff_angle = math.atan2(middle_point_y + vector_y*(0.25/0.5), middle_point_x + vector_x*(0.25/0.5))  #AQUI JA NAO PODE SER SO EM X

		goal_pose.pose.position.y=middle_point_y + vector_y*(0.25/0.5)
		goal_pose.pose.position.x=middle_point_x + vector_x*(0.25/0.5)

		self.goal_pose_pub.publish(goal_pose)
		# diff_angle = math.atan2(point_value_y,middle_point_x-0.3)    #0.2 para ficar 20cm
		
		self.angle_goal = - diff_angle + chosen_line.angle
		print "angle goal", self.angle_goal
	
		self.flag=2
		rospy.sleep(1)
		

	if self.flag == 2:

		if chosen_line.angle > self.angle_goal:
			vel_msg.linear.x = 0
			vel_msg.angular.z = -0.15
			self.angle_commited= abs(chosen_line.angle)
		if chosen_line.angle < self.angle_goal:
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0.15
			self.angle_commited= abs(chosen_line.angle)
		if chosen_line.angle>self.angle_goal-0.03 and chosen_line.angle<self.angle_goal+0.03: #este 0.03 depende da vel.angular
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			self.flag=3
			self.angle_commited= abs(chosen_line.angle)

		self.cmd_vel_pub.publish(vel_msg)

	if self.flag == 3:

		vel_msg.linear.x = 0.2

		if chosen_line.radius < 0.3 + abs(chosen_line.angle)/10:
			vel_msg.linear.x = 0
			self.flag = 4
			self.angle_commited= None


		self.cmd_vel_pub.publish(vel_msg)

	if self.flag == 4:

		if chosen_line.angle < - 0.02:
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0.15
		if chosen_line.angle > 0.02:
			vel_msg.linear.x = 0
			vel_msg.angular.z = -0.15
		if chosen_line.angle > -0.02 and chosen_line.angle<0.02:
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			self.flag= 5

		self.cmd_vel_pub.publish(vel_msg)

	if self.flag == 6 and self.approach_wall:

		self.counter=self.counter+1

		vel_msg.linear.x = 0.2
		self.cmd_vel_pub.publish(vel_msg)

		print "Counter", self.counter
		print "Angle goal", self.angle_goal

		if self.counter > abs(self.angle_goal) * self.condition_stop *self.brick_to_be_placed:
			
			vel_msg.linear.x = 0
			self.cmd_vel_pub.publish(vel_msg)

	print "-------------------------------------------------------------------------------------"

def main():

    rospy.init_node('laser_brick_detection')
    n = line_node()
    rospy.spin()

if __name__ == '__main__':
    main()

