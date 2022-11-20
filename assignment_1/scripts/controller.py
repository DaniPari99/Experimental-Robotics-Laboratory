#!/usr/bin/env python

"""
.. module:: controller
   :platform: Unix
   :synopsis: Python module for simulating the random movemente of the robot in a location

.. moduleauthor:: Daniele Martino Parisi

Servers:
    /controller

The node simulate the random robot motion in the location reached by only waste time
"""
import rospy
import random
import time

from assignment_1.srv import Controller


def MoveRandomCallBack(req):
	"""
	Function callback called whenever a client sends a request
	Args:
		self
	"""
	print("visiting")
	time.sleep(1)
	return 1

def my_controller_server():
	"""
	Function used for initializing the server 'my_controller_server'
	Args:
		self
	"""
	rospy.init_node('my_controller_server')
	s = rospy.Service('controller', Controller, MoveRandomCallBack)
	print("Service ready.")
	rospy.spin()

if __name__ == "__main__":
	 my_controller_server()
