#!/usr/bin/env python

import rospy
import sys
import smach
import smach_ros
import threading
from smach import State,StateMachine

#states & actions
import actions_ros.navigation as navigation
import states_ros.atrv_states as atrv_states
import actions_ros.wall_dispatcher as wall_dispatcher 





def ch2_sm():
    color_dict = {"Red" : 0.3,
                       "Green" : 0.6,
                       "Blue" : 1.2,
                       "green" : 0.6,
                       "red" : 0.3,
                       "blue" : 1.2,
                       "orange" : 1.8,
                       "Orange" : 1.8
                      }
    wall_dict = {"Red" : "/red_pile",
                       "Green" : "/green_pile",
                       "Blue" : "/blue_pile",
                       "green" : "/green_pile",
                       "red" : "/red_pile",
                       "blue" : "/blue_pile",
                       "orange" : "/orange_pile",
                       "Orange" : "/orange_pile"
                      }
    wd = wall_dispatcher.Wall_dispatcher("/home/mbzirc/ros_ws/src/atrv_ur5e/planning/planning/ros/src/planning_ros/Wall1.txt")
    sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILURE'])

    with sm:
       #  sm.add('OBJECT_LISTENER', atrv_states.Listen_to_Objects()),
       #         transitions={'success': 'GO_TO_PILE_FIRST',
       #                      'failure': 'OVERALL_SUCCESS'})

       #  sm.add('GO_TO_PILE_FIRST', atrv_states.MoveTo(rospy.get_param('brick_to_place'), frame='odom'),
       #         transitions={'success': 'ALLIGN_BRICK_FIRST',
       #                      'failure': 'OVERALL_SUCCESS'})

       #  sm.add('ALLIGN_BRICK_FIRST', atrv_states.Allign_Pick(),
       #         transitions={'success': 'PICK_BRICK_FIRST',
       #                      'failure': 'OVERALL_SUCCESS'})

        sm.add('PICK_BRICK_FIRST', atrv_states.Pick(),#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ
               transitions={'success': 'OVERALL_SUCCESS',
                            'failure': 'OVERALL_SUCCESS'})

       #  sm.add('GO_TO_WALL_FIRST', atrv_states.MoveTo([-0.4,4-color_dict[rospy.get_param('brick_to_place')]/2,0], frame='L_frame'),
       #         transitions={'success': 'PLACE_FIRST_BRICK',
       #                      'failure': 'OVERALL_SUCCESS'})

        # sm.add('PLACE_FIRST_BRICK', atrv_states.Place_first(), #AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ
        #        transitions={'success': 'OVERALL_SUCCESS',
        #                     'failure': 'OVERALL_SUCCESS'})

       #  sm.add('GO_TO_PILE', atrv_states.MoveTo(rospy.get_param('brick_to_place'), frame='odom'),
       #         transitions={'success': 'ALLIGN_BRICK',
       #                      'failure': 'OVERALL_SUCCESS'})

       #  sm.add('ALLIGN_BRICK', atrv_states.Allign_Pick(),
       #         transitions={'success': 'GO_TO_WALL',
       #                      'failure': 'OVERALL_SUCCESS'})

       #  sm.add('PICK_BRICK', atrv_states.Pick(),
       #         transitions={'success': 'GO_TO_WALL',
       #                      'failure': 'OVERALL_SUCCESS'})

       #  sm.add('GO_TO_WALL', atrv_states.MoveTo([-1.5,4-color_dict[rospy.get_param('brick_to_place')]/2-rospy.get_param('wall_len'),0], frame='L_frame'),
       #         transitions={'success': 'ALLIGN_WALL',
       #                      'failure': 'OVERALL_SUCCESS'})

       #  sm.add('ALLIGN_WALL', atrv_states.Allign_Place(),
       #         transitions={'success': 'GO_TO_PILE',
       #                      'failure': 'OVERALL_FAILURE'})

        # sm.add('PLACE_BRICK', atrv_states.Place(),#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ#AJGFJSDAGFSHUHDOIUAHUFHJ
        #        transitions={'success': 'OVERALL_SUCCESS',
        #                     'failure': 'OVERALL_SUCCESS'})


    # Execute SMACH plan
    # outcome = sm.execute()
    # rospy.loginfo('Final outcome: %s' % outcome)

    # Create and start the introspection server (Smach viewer)
    sis = smach_ros.IntrospectionServer('ch2_sm_viewer', sm, '/CH2_SM')
    sis.start()

    # Execute the state machine
    #outcome = sm.execute()

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c to stop the application
    rospy.spin()

    rospy.logwarn("ctrl + c detected!!! preempting smach execution")

    sis.stop()

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()


def main():

    rospy.init_node('ch2_sm', anonymous=False)
    rospy.sleep(0.5)
    ch2_sm()
