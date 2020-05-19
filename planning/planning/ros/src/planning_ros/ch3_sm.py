#!/usr/bin/env python

import rospy
import sys
import smach
import smach_ros
import threading
from smach import State,StateMachine

#states & actions
import states_ros.atrv_states_ch3 as atrv_states

def ch3_sm():

    sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS'])

    with sm:
        sm.add('CHECK_FIRE_1', atrv_states.MoveTo('CHECK_FIRE_1', frame='map'),
               transitions={'success': 'DETECT_FIRE_1',
                            'failure': 'CHECK_FIRE_2'})

        sm.add('DETECT_FIRE_1', atrv_states.DetectFire(),
               transitions={'success': 'ARM_FIRE_1',
                            'failure': 'CHECK_FIRE_2'})

        sm.add('ARM_FIRE_1', atrv_states.MoveArm('1'),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'CHECK_FIRE_1'})

        sm.add('CHECK_FIRE_2', atrv_states.MoveTo('CHECK_FIRE_2', frame='map'),
               transitions={'success': 'DETECT_FIRE_2',
                            'failure': 'CHECK_FIRE_1'})

        sm.add('DETECT_FIRE_2', atrv_states.DetectFire(),
               transitions={'success': 'ARM_FIRE_2',
                            'failure': 'CHECK_FIRE_1'})

        sm.add('ARM_FIRE_2', atrv_states.MoveArm('2'),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'CHECK_FIRE_2'})
                
        sm.add('PUMP_WATER', atrv_states.PumpWater(duration=5),
               transitions={'success': 'ORIGIN',
                            'failure': 'PUMP_WATER'})
       
        sm.add('ORIGIN',atrv_states.MoveTo('ORIGIN', frame='map'),
               transitions={'success': 'OVERALL_SUCCESS',
                            'failure': 'STOP'})

        sm.add('STOP', atrv_states.Stop(),
               transitions={'success': 'OVERALL_SUCCESS',
                            'failure': 'STOP'})



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
