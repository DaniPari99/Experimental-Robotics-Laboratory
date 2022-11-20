#! /usr/bin/env python
"""
.. module:: helper
   :platform: Unix
   :synopsis: Python module for helping the finite stete machine module

.. moduleauthor:: Daniele Martino Parisi

Subscribes to:
    'state/battery_low'

Clients:
    /controller
    /client: for connecting with Armor server

The node implements a class which contains a lot of functions that can help the finite state machine node in performing its task.
"""

import rospy
import random
import time

from armor_api.armor_client import ArmorClient
from os.path import dirname, realpath
from arch_skeleton import architecture_name_mapper as anm
from std_msgs.msg import Bool
from assignment_1.srv import *
from threading import Lock
from actionlib import SimpleActionClient

path = dirname(realpath(__file__))
path = path + "/../"
"""
Relative path of the .owl file
"""
client = ArmorClient("test5","ontology5")
"""
Client which sends the requests to Armor server
"""
# Initializing with buffered manipulation and reasoning
client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)

# Class for helping the assignment_fsm in order to execute the states
class HelperInterface:

    def __init__(self):
         """
         Constructor function of the HelperInterface class.
         Args:
             self
         """
         # Create a shared mutex to synchronize action clients and subscribers.
         # Note that, based on different assumptions, further optimization can be done to make the different threads
         # blocking for a less amount of time in the same mutex.
         self.mutex = Lock()

         self.reset_states()

         # Define the callback associated with the battery low ROS subscribers.
         rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self.BatteryLowCallback)

    # Function to reset the battery state
    def reset_states(self):
        """
        Function to reset the battery state
        Args:
            self
        """
        self._battery_low = True

    # Function callback called whenever a new data is published in the topic battery_low by the node robot_states
    def BatteryLowCallback(self, msg):
        """
        Function callback called whenever a new data is published in the topic battery_low by the node robot_states
        Args:
            msg(Battery_state): the battery state
        """
        # Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class.
            self._battery_low = msg.data
        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
            self.mutex.release()

    # Function to get the state variable encoded in this class that concerns the battery level.
    # The returning value will be `True` if the battery is low, `False` otherwise.
    def is_battery_low(self):
        """
        Function called by the finite state machine node for knowing the current battery state
        Args:
            self
        Returns:
            _battery_low: The state of the battery
        """
        return self._battery_low

    # Function to load the ontology of the given map
    def LoadOntology(self):
        """
        Function called by the finite state machine node for loading the map of the ontology
        Args:
            self
        """
        # define the relative path of the .owl file
        path = dirname(realpath(__file__))
        path = path + "/../"
        # Declare the client
        client = ArmorClient("test5","ontology5")

        # Initializing with buffered manipulation and reasoning
        client.utils.load_ref_from_file(path + "topological_map.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)
        client.utils.mount_on_ref()
        client.utils.set_log_to_terminal(True)

        # Add all our individuals
        client.manipulation.add_ind_to_class("R1", "LOCATION")
        print("Added R1 to LOCATION")
        client.manipulation.add_ind_to_class("R2", "LOCATION")
        print("Added R2 to LOCATION")
        client.manipulation.add_ind_to_class("R3", "LOCATION")
        print("Added R3 to LOCATION")
        client.manipulation.add_ind_to_class("R4", "LOCATION")
        print("Added R4 to LOCATION")
        client.manipulation.add_ind_to_class("C1", "LOCATION")
        print("Added C1 to LOCATION")
        client.manipulation.add_ind_to_class("C2", "LOCATION")
        print("Added C2 to LOCATION")
        client.manipulation.add_ind_to_class("E", "LOCATION")
        print("Added E to LOCATION")
        client.manipulation.add_ind_to_class("D1", "DOOR")
        print("Added D1 to DOOR")
        client.manipulation.add_ind_to_class("D2", "DOOR")
        print("Added D2 to DOOR")
        client.manipulation.add_ind_to_class("D3", "DOOR")
        print("Added D3 to DOOR")
        client.manipulation.add_ind_to_class("D4", "DOOR")
        print("Added D4 to DOOR")
        client.manipulation.add_ind_to_class("D5", "DOOR")
        print("Added D5 to DOOR")
        client.manipulation.add_ind_to_class("D6", "DOOR")
        print("Added D6 to DOOR")
        client.manipulation.add_ind_to_class("D7", "DOOR")
        print("Added D7 to DOOR")

        # Disjoint of the individuals of the classes
        client.manipulation.disj_inds_of_class("LOCATION")
        client.manipulation.disj_inds_of_class("DOOR")
        print("all individuals are disjointed")

        # Add properties to objects
        # Distinction between rooms and corridors
        client.manipulation.add_objectprop_to_ind("hasDoor", "R1", "D1")
        client.manipulation.add_objectprop_to_ind("hasDoor", "R2", "D2")
        client.manipulation.add_objectprop_to_ind("hasDoor", "R3", "D3")
        client.manipulation.add_objectprop_to_ind("hasDoor", "R4", "D4")
        client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D1")
        client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D2")
        client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D5")
        client.manipulation.add_objectprop_to_ind("hasDoor", "C1", "D7")
        client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D3")
        client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D4")
        client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D5")
        client.manipulation.add_objectprop_to_ind("hasDoor", "C2", "D6")
        client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D6")
        client.manipulation.add_objectprop_to_ind("hasDoor", "E", "D7")

        # Connections between locations
        client.manipulation.add_objectprop_to_ind("connectedTo", "R1", "C1")
        client.manipulation.add_objectprop_to_ind("connectedTo", "R2", "C1")
        client.manipulation.add_objectprop_to_ind("connectedTo", "C1", "C2")
        client.manipulation.add_objectprop_to_ind("connectedTo", "R3", "C2")
        client.manipulation.add_objectprop_to_ind("connectedTo", "R4", "C2")
        client.manipulation.add_objectprop_to_ind("connectedTo", "E", "C1")
        client.manipulation.add_objectprop_to_ind("connectedTo", "E", "C2")

        # Initialize the robot position
        client.manipulation.add_objectprop_to_ind("isIn", "Robot1", "E")

        # Initialize room and time stamps
        client.manipulation.add_dataprop_to_ind("visitedAt", "R1", 'Long', str(int(time.time())))
        print("R1 visited")
        client.manipulation.add_dataprop_to_ind("visitedAt", "R2", 'Long', str(int(time.time())))
        print("R2 visited")
        client.manipulation.add_dataprop_to_ind("visitedAt", "R3", 'Long', str(int(time.time())))
        print("R3 visited")
        client.manipulation.add_dataprop_to_ind("visitedAt", "R4", 'Long', str(int(time.time())))
        print("R4 visited")
        client.manipulation.add_dataprop_to_ind("visitedAt", "C1", 'Long', str(int(time.time())))
        print("C1 visited")
        client.manipulation.add_dataprop_to_ind("visitedAt", "C2", 'Long', str(int(time.time())))
        print("C2 visited")
        client.manipulation.add_dataprop_to_ind("visitedAt", "E", 'Long', str(int(time.time())))
        print("E visited")

        # Update the reasoner
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

        # save and exit
        client.utils.save_ref_with_inferences(path + "topological_map.owl")
        print("Map saved")
        #client.utils.save(path + "topological_map.owl", inference=False)

    # Function to move the robot from a given location to an other one
    def MoveRobot(self, new_loc):
        """
        Function called by the finite state machine node for moving the robot from the current location to the chosen one
        Args:
            self
            new_loc: location chosen in the 'decide' state of the finite state machine node
        """
        # save the prev position of the robot
        old_loc = client.query.objectprop_b2_ind("isIn", "Robot1")
        # delete the useless characters
        old_loc = old_loc[0]
        old_loc = old_loc[:len(old_loc)-1]
        old_loc = old_loc[32:]

        # query for retrieving the previous 'now'
        robot_prev_now = client.query.dataprop_b2_ind("now", "Robot1")

        # delete the useless characters
        robot_prev_now = robot_prev_now[0]
        robot_prev_now = robot_prev_now[:len(robot_prev_now)-11]
        robot_prev_now = robot_prev_now[1:]

        # query for retrieving the previous 'visitedAt'
        loc_prev_visitedAt = client.query.dataprop_b2_ind("visitedAt", new_loc)

        # delete the useless characters
        loc_prev_visitedAt = loc_prev_visitedAt[0]
        loc_prev_visitedAt = loc_prev_visitedAt[:len(loc_prev_visitedAt)-11]
        loc_prev_visitedAt = loc_prev_visitedAt[1:]

        # manipulation for changing the current location of the robot with the new one
        client.manipulation.replace_objectprop_b2_ind("isIn", "Robot1", new_loc, old_loc)

        # manipulation for changing the previous timestamp 'now' with the new one
        client.manipulation.replace_dataprop_b2_ind("now", "Robot1", 'Long', str(int(time.time())), robot_prev_now)

        # manipulation for changing the previous timestamp visitedAt with the new one
        client.manipulation.replace_dataprop_b2_ind("visitedAt", new_loc, 'Long', str(int(time.time())), loc_prev_visitedAt)

        # Update the reasoner after the manipulations
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

    # Function to retrieve the adiacent locations that the robot can reach directly from the current deleting the location 'E'
    def SetReachableLoc(self):
        """
        Function called by the Choose function in order to set the reachable location by deleting the location 'E'
        Args:
            self
        Returns:
            reachable_loc: the list of reachable locations
        """
        # query for knowing which adiacent locations can be reached
        reachable_loc = client.query.objectprop_b2_ind("canReach", "Robot1")

        # cancel the useless characters
        for i in range(len(reachable_loc)):
            reachable_loc[i] = reachable_loc[i][:len(reachable_loc[i])-1]
            reachable_loc[i] = reachable_loc[i][32:]

        # remove the location E from the list of the reachable, because we will go in E only if the battery goes low
        for i in reachable_loc.copy():
            if i == "E":
                reachable_loc.remove(i)
                break

        print("reachable locations")
        print(reachable_loc)

        return reachable_loc

    # Function to retrieve the list of the whole urgent locations deleting the corridors and location E
    def SetUrgentLoc(self):
        """
        Function called by the Choose function in order to set the urgent locations by deleting the location 'E' and the 2 corridors
        Args:
            self
        Returns:
            urgent_loc: the list of urgent locations
        """
        # query for knowing urgent locations
        urgent_loc = client.query.ind_b2_class("URGENT")
        for i in range(len(urgent_loc)):
            urgent_loc[i] = urgent_loc[i][:len(urgent_loc[i])-1]
            urgent_loc[i] = urgent_loc[i][32:]

        for i in urgent_loc.copy():

            if i == "C1":
                urgent_loc.remove(i)
                continue
            if i == "C2":
                urgent_loc.remove(i)
                continue
            if i == "E":
                urgent_loc.remove(i)
                continue

        print("urgent locations are:")
        print(urgent_loc)

        return urgent_loc

    # Function to retrieve the list of the whole corridor
    def SetCorridorLoc(self):
        """
        Function called by the Choose function in order to set the corridors
        Args:
            self
        Returns:
            corridor_loc: the list of corridors
        """
        corridor_loc = client.query.ind_b2_class("CORRIDOR")
        for i in range(len(corridor_loc)):
            corridor_loc[i] = corridor_loc[i][:len(corridor_loc[i])-1]
            corridor_loc[i] = corridor_loc[i][32:]

        return corridor_loc

    # Function to retrieve the list of the all rooms
    def SetRoomLoc(self):
        """
        Function called by the Choose function in order to set the Rooms
        Args:
            self
        Returns:
            room_loc: the list of rooms
        """
        room_loc = client.query.ind_b2_class("ROOM")
        for i in range(len(room_loc)):
            room_loc[i] = room_loc[i][:len(room_loc[i])-1]
            room_loc[i] = room_loc[i][32:]

        return room_loc

    # Function to choose the next location due to the reachable locations,
    # the urgent ones and knowing that the priority is: firstly we see
    # the reachable urgent locations, then the corridors and if we have
    # more than 1 possibilities I choose rondomly from a list of possibilities
    def Choose(self):
        """
        Function called by the finite state machine to decide in what location go next
        Args:
            self
        Returns:
            possible_choices: the list of possible next locations
        """
        reachable_loc = self.SetReachableLoc()
        urgent_loc = self.SetUrgentLoc()
        corridor_loc = self.SetCorridorLoc()
        room_loc = self.SetRoomLoc()
        possible_choices = []

        # check if among reachable locations there are some urgent ones.
        # In this case the possible choices are set
        for i in range(len(reachable_loc)):
            for j in range(len(urgent_loc)):
                if reachable_loc[i] == urgent_loc[j]:
                    possible_choices = possible_choices + [reachable_loc[i]]

        # in case that in the previous check the list of possible choices
        # is still empty (there are not reachable rooms among the urgent ones)
        # the list of possible choices is filled by the corridors
        if len(possible_choices) == 0:
            for i in range(len(reachable_loc)):
                for j in range(len(corridor_loc)):
                    if reachable_loc[i] == corridor_loc[j]:
                        possible_choices = possible_choices + [reachable_loc[i]]

        # in case that in the previous checks the list of possible choices
        # is still empty (there are not neither reachable rooms among
        # the urgent ones nor corridors among the reachable ones) the list of
        # possible choices is filled by the reachable rooms (not urgent)
        if len(possible_choices) == 0:
            for i in range(len(reachable_loc)):
                for j in range(len(room_loc)):
                    if reachable_loc[i] == room_loc[j]:
                        possible_choices = possible_choices + [reachable_loc[i]]

        print("possible choices are: ")
        print(possible_choices)

        # choose a random location among the list of the possible choices
        choice = random.choice(possible_choices)
        print("Decision:" + choice)
        return choice

    # Function to visit a location. It calls a server for only wasting time
    def VisitLocation(self, room_chosen):
        """
        Function called by the finite state machine to visit the room chosen
        Args:
            self
            room_chosen: room chosen to be visited
        """
        self.controller_client()

    # Function of client used to call the server controller
    def controller_client(self):
        """
        Function called by the visit location function in order to call the controller server for simulating the random motion of the robot in a room
        Args:
            self
        """
        rospy.wait_for_service('controller')
        try:
            controller = rospy.ServiceProxy('controller', Controller)
            resp1 = controller()
            return resp1.received
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    # Function to know the robot position
    def CheckRobotPosition(self):
        """
        Function called by the finite state machine node in order to know in which location the robot is
        Args:
            self
        Returns:
            robot_position: the current robot position
        """
        robot_position = client.query.objectprop_b2_ind("isIn", "Robot1")

        robot_position = robot_position[0]
        robot_position = robot_position[:len(robot_position)-1]
        robot_position = robot_position[32:]

        return robot_position
