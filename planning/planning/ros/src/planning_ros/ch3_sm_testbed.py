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

def ch3_sm():

    sm = smach.StateMachine(outcomes=['OVERALL_SUCCESS'])

    with sm:
        sm.add('GO_TO_ENTRANCE', atrv_states.MoveTo('ENTRANCE_DOOR_1', frame='map'),
               transitions={'success': 'SWITCH_LOCALIZATION',
                            'failure': 'GO_TO_ENTRANCE'})

        sm.add('SWITCH_LOCALIZATION', atrv_states.SwitchLocalization("indoors"),
               transitions={'success': 'INSIDE_DOOR',
                            'failure': 'SWITCH_LOCALIZATION'})
        
        sm.add('INSIDE_DOOR', atrv_states.MoveTo('INSIDE_DOOR', frame='map'),
               transitions={'success': 'ROOM_1',
                            'failure': 'GO_TO_ENTRANCE'})
              
        sm.add('ROOM_1', atrv_states.MoveTo('ROOM_1', frame='map'),
               transitions={'success': 'DETECT_FIRE_1',
                            'failure': 'INSIDE_DOOR'})

        sm.add('DETECT_FIRE_1', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'ROOM_2'})

        sm.add('ROOM_2', atrv_states.MoveTo('ROOM_2', frame='map'),
               transitions={'success': 'DETECT_FIRE_2',
                            'failure': 'INSIDE_DOOR'})

        sm.add('DETECT_FIRE_2', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'ROOM_3'})

        sm.add('ROOM_3', atrv_states.MoveTo('ROOM_3', frame='map'),
               transitions={'success': 'DETECT_FIRE_3',
                            'failure': 'INSIDE_DOOR'})

        sm.add('DETECT_FIRE_3', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'ROOM_4'})

        sm.add('ROOM_4', atrv_states.MoveTo('ROOM_4', frame='map'),
               transitions={'success': 'DETECT_FIRE_4',
                            'failure': 'INSIDE_DOOR'})

        sm.add('DETECT_FIRE_4', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'INSIDE_DOOR_KITCHEN'})

        sm.add('INSIDE_DOOR_KITCHEN', atrv_states.MoveTo('INSIDE_DOOR', frame='map'),
               transitions={'success': 'DETECT_FIRE_INSIDE_DOOR',
                            'failure': 'KITCHEN_1'})

        sm.add('DETECT_FIRE_INSIDE_DOOR', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'KITCHEN_1'})

        sm.add('KITCHEN_1', atrv_states.MoveTo('KITCHEN_1', frame='map'),
               transitions={'success': 'DETECT_FIRE_KITCHEN_1',
                            'failure': 'INSIDE_DOOR_KITCHEN'})

        sm.add('DETECT_FIRE_KITCHEN_1', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'KITCHEN_2'})

        sm.add('KITCHEN_2', atrv_states.MoveTo('KITCHEN_2', frame='map'),
               transitions={'success': 'DETECT_FIRE_KITCHEN_2',
                            'failure': 'INSIDE_DOOR_KITCHEN'})

        sm.add('DETECT_FIRE_KITCHEN_2', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'KITCHEN_3'})

        sm.add('KITCHEN_3', atrv_states.MoveTo('KITCHEN_3', frame='map'),
               transitions={'success': 'DETECT_FIRE_KITCHEN_3',
                            'failure': 'INSIDE_DOOR_KITCHEN'})

        sm.add('DETECT_FIRE_KITCHEN_3', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'KITCHEN_4'})

        sm.add('KITCHEN_4', atrv_states.MoveTo('KITCHEN_4', frame='map'),
               transitions={'success': 'DETECT_FIRE_KITCHEN_4',
                            'failure': 'INSIDE_DOOR_KITCHEN'})

        sm.add('DETECT_FIRE_KITCHEN_4', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'KITCHEN_5'})

        sm.add('KITCHEN_5', atrv_states.MoveTo('KITCHEN_5', frame='map'),
               transitions={'success': 'DETECT_FIRE_KITCHEN_5',
                            'failure': 'INSIDE_DOOR_KITCHEN'})

        sm.add('DETECT_FIRE_KITCHEN_5', atrv_states.DetectFire(),
               transitions={'success': 'PUMP_WATER',
                            'failure': 'INSIDE_DOOR'})

                
        sm.add('PUMP_WATER', atrv_states.PumpWater(),
               transitions={'success': 'ENTRANCE_DOOR_1_FINAL',
                            'failure': 'PUMP_WATER'})

        sm.add('ENTRANCE_DOOR_1_FINAL', atrv_states.MoveTo('ENTRANCE_DOOR_1', frame='map'),
               transitions={'success': 'SWITCH_LOCALIZATION_FINAL',
                            'failure': 'ENTRANCE_DOOR_1_FINAL'})

        sm.add('SWITCH_LOCALIZATION_FINAL', atrv_states.SwitchLocalization("outdoors"),
               transitions={'success': 'ORIGIN',
                            'failure': 'SWITCH_LOCALIZATION_FINAL'})
       
        sm.add('ORIGIN',atrv_states.MoveTo('ORIGIN', frame='map'),
               transitions={'success': 'OVERALL_SUCCESS',
                            'failure': 'ORIGIN'})



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
