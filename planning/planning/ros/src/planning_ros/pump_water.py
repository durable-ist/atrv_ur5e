#!/usr/bin/env python

import rospy
import sys
import smach
import smach_ros
import threading
from smach import State,StateMachine

#states & actions
import states_ros.atrv_states as atrv_states

def pump_water_sm():

    sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS'])

    with sm:
                
        sm.add('PUMP_WATER', atrv_states.PumpWater(28),
               transitions={'success': 'OVERALL_SUCCESS',
                            'failure': 'PUMP_WATER'})




    # Execute SMACH plan
    # outcome = sm.execute()
    # rospy.loginfo('Final outcome: %s' % outcome)

    # Create and start the introspection server (Smach viewer)
    sis = smach_ros.IntrospectionServer('pump_water_sm_viewer', sm, '/PUMP_WATER_SM')
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
    pump_water_sm()
