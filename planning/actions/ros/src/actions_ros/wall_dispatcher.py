import sys
import rospy
import numpy as np
import cv2
import tf
import math
from std_srvs.srv import Trigger, TriggerResponse

class Wall_dispatcher:

  def __init__(self, wall_txt):

    #rospy.init_node('iteration_counter', anonymous=True)
    self.wall_txt = wall_txt
    self.wall = []
    # self.wall = [["red", "red", "blue", "red", "green", "green", "red"], #level5 highest
    #              ["red", "green", "red", "red", "green", "blue", "red"], #level4
    #              ["red", "red", "red", "green", "red", "green", "blue"], #level3
    #              ["red", "red", "blue", "red", "red", "green", "green"], #level2
    #              ["green", "red", "red", "green", "red", "blue", "green"]] #level1 ground level
    self.colors = ["Red","Green","Blue","green", "red", "blue"]
    self.color_dict = {"Red" : 0.3,
                       "Green" : 0.6,
                       "Blue" : 1.2,
                       "green" : 0.6,
                       "red" : 0.3,
                       "blue" : 1.2
                      }
    self.translate_wall_dict = { "R" : "red",
                                 "G" : "green",
                                 "B" : "blue" 
                               }

    print "in init wall dispatcher"
    self.read_wall_from_txt()
    self.wall_check()
    print self.wall
    self.next = rospy.Service("next_iteration", Trigger, self.trigger)
    self.side = "right" #set this to fixed value
    self.counter = 0
    self.variables = []
    self.dispatcher_execute()
    self.update_vars()

  def read_wall_from_txt(self):
    f = open(self.wall_txt,"r")
    for line in f:
      new_line = []
      split = line.split()
      for item in split:
        new_line.append(self.translate_wall_dict[item])
      self.wall.append(new_line)
    f.close()

  def wall_check(self):
    for x in self.wall:
      if len(self.wall) != 5:
        print "wrong number of layers"
        exit()
      if len(x) != 7:
        print "wrong number of bricks in layer"
        exit()
      for y in x:
        if y not in self.colors:
          print "wrong wall format"
          exit()

  def opposite_side(self, side):
    if side == "left":
      return "right"
    if side == "right":
      return "left"

  def dispatcher_execute(self):
    lenght = self.color_dict[self.wall[-1][0]]
    reference_brick = self.wall[-1][0]
    first = True
    level = 1
    over = False
    side = self.side
    for x in reversed(self.wall):
      for y in x:
        if first:
          first = False
          continue
        line = []
        line.append(side)
        line.append(reference_brick)
        line.append(level)
        line.append(over)
        line.append(y)
        line.append(lenght)
        self.variables.append(line)
        lenght += self.color_dict[y]
        reference_brick = y #previous is reference for next
        if over:
          level += 1 
          side = self.side
          over = False
      over = True
      side = self.opposite_side(side)
      reference_brick = x[0] #first brick of the previous level is the reference for the next level
      lenght = 0

  def update_vars(self):
    
    rospy.set_param('side', self.variables[self.counter][0])
    rospy.set_param('reference_brick', self.variables[self.counter][1])
    rospy.set_param('level', self.variables[self.counter][2])
    rospy.set_param('over', self.variables[self.counter][3])
    rospy.set_param('brick_to_place', self.variables[self.counter][4])
    rospy.set_param('wall_len', self.variables[self.counter][5])
    self.counter += 1

  def trigger(self, data):
    self.update_vars()
    return TriggerResponse(success=True, message = "variables updated")
