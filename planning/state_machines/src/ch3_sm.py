#!/usr/bin/env python

import rospy
import sys
import smach
import smach_ros
import threading
from smach import State,StateMachine

#states & actions
import ../../actions/navigation
import ../../states/atrv_states


def ch3_sm():

    sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS'])

    with sm:
        sm.add('GO_TO_ENTRANCE', atrv_states.MoveTo('ENTRANCE_DOOR_1', frame='map'),
               transitions={'success': 'SWITCH_LOCALIZATION',
                            'failure': 'GO_TO_ENTRANCE'})

        sm.add('SWITCH_LOCALIZATION', atrv_states.SwitchLocalization("indoors"),
               transitions={'success': 'WAYPOINT_1',
                            'failure': 'SWITCH_LOCALIZATION'})
        
        sm.add('WAYPOINT_1', atrv_states.MoveTo('INSIDE_DOOR', frame='map'),
               transitions={'success': 'OVERALL_SUCCESS',
                            'failure': 'WAYPOINT_1'})



    # Execute SMACH plan
    # outcome = sm.execute()
    # rospy.loginfo('Final outcome: %s' % outcome)

    # Create and start the introspection server (Smach viewer)
    sis = smach_ros.IntrospectionServer('ch3_sm_viewer', sm, '/CH3_SM')
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

    rospy.init_node('ch3_sm', anonymous=False)
    rospy.sleep(0.5)
    ch3_sm()
