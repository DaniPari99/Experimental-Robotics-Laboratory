#! /usr/bin/env python

"""
.. module:: assignment_fsm
   :platform: Unix
   :synopsis: Python module for the finite state machine

.. moduleauthor:: Daniele Martino Parisi

Clients:
    /client: for connecting with Armor server

The node implements the finite state machine which drives the robot through the locations of the map according to the stimulus
"""

import roslib
import rospy
import smach
import smach_ros
import time
import random
import sys
import helper

from std_msgs.msg import Bool
from armor_api.armor_client import ArmorClient
from std_msgs.msg import String
from arch_skeleton import architecture_name_mapper as anm
from os.path import dirname, realpath
from helper import HelperInterface

from assignment_1.srv import *

path = dirname(realpath(__file__))
path = path + "/../"
client = ArmorClient("test5","ontology5")
# Initializing with buffered manipulation and reasoning
client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)
room_chosen = 0
# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.3
LOOP_STATE_TIME = 3

# define state Wait
class Wait(smach.State):
    def __init__(self, helper_interface):
        """
        Constructor function of the Wait class.
        Args:
            self
            helper_interface: object created by the class 'HelperInterface'
        """
        self._helper = helper_interface
        smach.State.__init__(self,
                             outcomes=['loaded','rested', 'tired', 'visited', 'decided'],
                             input_keys=['wait_counter_in'],
                             output_keys=['wait_counter_out'])

    def execute(self, userdata):
        """
        Function executed only at the first state (WAIT) of the finite state machine
        Args:
            self
            userdata: variable passed between the states of the fsm
        """
        print("loading ontology...")
        #self._helper.LoadOntology()

        rospy.loginfo('Executing state Wait (users = %f)'%userdata.wait_counter_in)
        return 'loaded'

class Sleep(smach.State):
    def __init__(self, helper_interface):
        """
        Constructor function of the Sleep class.
        Args:
            self
            helper_interface: object created by the class 'HelperInterface'
        """
        self._helper = helper_interface
        smach.State.__init__(self,
                             outcomes=['loaded','rested', 'tired', 'visited', 'decided'],
                             input_keys=['sleep_counter_in'],
                             output_keys=['sleep_counter_out'])

    def execute(self, userdata):
        """
        Function executed whenever the fsm goes in the 'Sleep' state
        Args:
            self
            userdata: variable passed between the states of the fsm
        """
        rospy.loginfo('Executing state SLEEP (users = %f)'%userdata.sleep_counter_in)

        while not rospy.is_shutdown():
            self._helper.mutex.acquire()
            try:
                robot_position = self._helper.CheckRobotPosition()
                if robot_position != "E":
                    print("robot go for recharging")
                    self._helper.MoveRobot("E")

                time.sleep(LOOP_STATE_TIME)
                if self._helper.is_battery_low():
                    return 'tired'

                elif not self._helper.is_battery_low():
                    return 'rested'

            finally:
                self._helper.mutex.release()

            rospy.sleep(LOOP_SLEEP_TIME)

class Decide(smach.State):
    def __init__(self, helper_interface):
        """
        Constructor function of the Decide class.
        Args:
            self
            helper_interface: object created by the class 'HelperInterface'
        """
        self._helper = helper_interface
        smach.State.__init__(self,
                             outcomes=['loaded','rested', 'tired', 'visited', 'decided'],
                             input_keys=['decide_counter_in'],
                             output_keys=['decide_counter_out', 'room_chosen'])

    def execute(self, userdata):
        """
        Function executed whenever the fsm goes in the 'Decide' state
        Args:
            self
            userdata: variable passed between the states of the fsm
        """
        rospy.loginfo('Executing state DECIDE (users = %f)'%userdata.decide_counter_in)
        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                if self._helper.is_battery_low():
                    return 'tired'

                elif not self._helper.is_battery_low():

                    userdata.room_chosen = self._helper.Choose()

                    time.sleep(LOOP_STATE_TIME)

                    return 'decided'
            finally:
                self._helper.mutex.release()

            rospy.sleep(LOOP_SLEEP_TIME)

class Visit(smach.State):
    def __init__(self, helper_interface):
        """
        Constructor function of the Visit class.
        Args:
            self
            helper_interface: object created by the class 'HelperInterface'
        """
        self._helper = helper_interface
        smach.State.__init__(self,
                             outcomes=['loaded','rested', 'tired', 'visited', 'decided'],
                             input_keys=['visit_counter_in', 'room_chosen'],
                             output_keys=['visit_counter_out'])

    def execute(self, userdata):
        """
        Function executed whenever the fsm goes in the 'Visit' state
        Args:
            self
            userdata: variable passed between the states of the fsm
        """
        rospy.loginfo('Executing state VISIT (users = %f)'%userdata.visit_counter_in)
        # Wait for stimulus from the other nodes of the architecture.
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                room_chosen = userdata.room_chosen
                time.sleep(LOOP_STATE_TIME)
                if self._helper.is_battery_low():
                    print("robot go fo recharging while it was visiting")
                    return 'tired'

                self._helper.MoveRobot(room_chosen)
                print("I am in " + room_chosen)

                i = 0
                while not self._helper.is_battery_low() and i < 5:
                    self._helper.VisitLocation(room_chosen)
                    i += 1
                    if self._helper.is_battery_low():
                        return 'tired'

                print(room_chosen + " visited")
                return 'visited'

            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

def main():

    rospy.init_node('robot_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    helper = HelperInterface()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT', Wait(helper),
                               transitions={'loaded':'SLEEP',
                                            'rested':'WAIT',
                                            'tired':'WAIT',
                                            'visited':'WAIT',
                                            'decided':'WAIT'},
                               remapping={'wait_counter_in':'sm_counter',
                                          'wait_counter_out':'sm_counter'})

        # Add states to the container
        smach.StateMachine.add('SLEEP', Sleep(helper),
                               transitions={'loaded':'SLEEP',
                                            'rested':'DECIDE',
                                            'tired':'SLEEP',
                                            'visited':'SLEEP',
                                            'decided':'SLEEP'},
                               remapping={'sleep_counter_in':'sm_counter',
                                          'sleep_counter_out':'sm_counter'})

        # Add states to the container
        smach.StateMachine.add('DECIDE', Decide(helper),
                               transitions={'loaded':'DECIDE',
                                            'rested':'DECIDE',
                                            'tired':'SLEEP',
                                            'visited':'DECIDE',
                                            'decided':'VISIT'},
                               remapping={'decide_counter_in':'sm_counter',
                                          'decide_counter_out':'sm_counter'})

        # Add states to the container
        smach.StateMachine.add('VISIT', Visit(helper),
                               transitions={'loaded':'VISIT',
                                            'rested':'VISIT',
                                            'tired':'SLEEP',
                                            'visited':'DECIDE',
                                            'decided':'VISIT'},
                               remapping={'visit_counter_in':'sm_counter',
                                          'visit_counter_out':'sm_counter'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
