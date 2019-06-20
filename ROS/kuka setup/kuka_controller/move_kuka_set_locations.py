#!/usr/bin/env python

# KUKA API for ROS

# Marhc 2016 Saeid Mokaram  saeid.mokaram@gmail.com
# Sheffield Robotics    http://www.sheffieldrobotics.ac.uk/
# The university of sheffield   http://www.sheffield.ac.uk/

# This script generats a ROS node for comunicating with KUKA iiwa
# Dependencies: conf.txt, ROS server, Rospy, KUKA iiwa java SDK, KUKA iiwa robot.

# This application is intended for floor mounted robots.
#######################################################################################################################
from client_lib import *
import rospy
import time
import tf
from std_msgs.msg import String
from std_msgs.msg import Float


object_pub = rospy.Publisher('object_to_grab', String, queue_size=10)


def setup():

	global my_client

	# Making a connection object.
	my_client = kuka_iiwa_ros_client()

	# Wait until iiwa is connected zzz!
	while (not my_client.isready): pass
	print('Started!')

	# Initializing Tool 1
	my_client.send_command('setTool tool1')

	# Initializing
	my_client.send_command('setJointAcceleration 0.1')  # If the JointAcceleration is not set, the defult value is 1.0.
	my_client.send_command('setJointVelocity 0.1')      # If the JointVelocity is not set, the defult value is 1.0.	my_client.send_command('setJointJerk 0.1')          # If the JointJerk is not set, the defult value is 1.0.
	#my_client.send_command('setCartVelocity 10000')     # If the CartVelocity is not set, the defult value is 100
	my_client.send_command('setCartVelocity 50')

	print "Going home"

	# Move close to a start position.
	my_client.send_command('setPosition -38.37 16.26 64.70 -57.43 -19.09 115.66 18.84')

	time.sleep(10)




if __name__ == '__main__':

	global my_client

	setup()

	print "Done robot setup"

	print "Picking up hammer"

	my_client.send_command('setPosition -34.58 74.05 64.69 -67.87 -67.34 75.37 25.28')

	time.sleep(10)

	my_client.close_grippers();

	print "Placing hammer"

	my_client.send_command('setPosition -35.01 44.27 64.70 -85.20 -42.12 78.10 47.29')

	my_client.send_command('setPosition -41.34 76.29 64.69 -60.92 -63.93 78.32 106.16')

	time.sleep(10)

	my_client.open_grippers();

	print "Picking up spanner"

	my_client.send_command('setPosition -35.01 44.27 64.70 -85.20 -42.12 78.10 47.29')

	my_client.send_command('setPosition -48.07 60.85 64.70 -109.04 -68.12 61.85 60.84')

	time.sleep(10)

	my_client.close_grippers();

	print "Placing spanner"

	my_client.send_command('setPosition -35.01 44.27 64.70 -85.20 -42.12 78.10 47.29')

	my_client.send_command('setPosition -72.69 57.76 64.70 -115.42 -65.18 58.75 113.75')

	time.sleep(10)

	my_client.open_grippers();

	time.sleep(2)

	print "Returning home"

	# Move close to a start position.
	my_client.send_command('setPosition -38.37 16.26 64.70 -57.43 -19.09 115.66 18.84')






