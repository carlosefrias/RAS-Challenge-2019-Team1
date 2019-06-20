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
import time as t
import tf
from std_msgs.msg import String
from std_msgs.msg import Float64


# Making a connection object.
my_client = kuka_iiwa_ros_client()

def setup():


	print "Connecting to robot"

	
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

	home_pos = [2.09, 23.84, 17.67, -57.30, -8.77, 106.15, 15.97]

	# Move close to a start position.
	my_client.send_command('setPosition 2.09 23.84 17.67 -57.30 -8.77 106.15 15.97')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in home_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1

	t.sleep(1)




if __name__ == '__main__':

	global my_client

	setup()

	print "Done robot setup"


	print "Picking up hammer"

	intermediate_pos = [-35.01, 44.27, 64.70, -85.20, -42.12, 78.10, 47.29]

	hammer_pos = [-27.77, 34.34, 18.18, -117.14, -18.35, 32.84, 88.12]
	#hammer_pos = [463, -62, 235, -180, 4.8, 178]

	my_client.send_command('setPosition -27.77 34.34 18.18 -117.14 -18.35 32.84 88.12')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in hammer_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1

	t.sleep(1)

	my_client.close_grippers();

	t.sleep(1)


	print "Moving to intermediate position"

	# MOVE ARM TO INTERMEDIATE POSITION
	my_client.send_command('setPosition -35.01 44.27 64.70 -85.20 -42.12 78.10 47.29')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in intermediate_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1


	print "Placing hammer"

	hammer_place_pos = [-41.34, 76.29, 64.69, -60.92, -63.93, 78.32, 106.16]

	my_client.send_command('setPosition -41.34 76.29 64.69 -60.92 -63.93 78.32 106.16')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in hammer_place_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1

	t.sleep(1)

	my_client.open_grippers();

	t.sleep(1)


	print "Moving to intermediate position"

	# MOVE ARM TO INTERMEDIATE POSITION
	my_client.send_command('setPosition -35.01 44.27 64.70 -85.20 -42.12 78.10 47.29')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in intermediate_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1


	print "Picking up spanner"

	spanner_pos = [-7.68, 48.99, 18.17, -87.38, -20.32, 47.89, 21.15]

	my_client.send_command('setPosition -7.68 48.99 18.17 -87.38 -20.32 47.89 21.15')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in spanner_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1

	t.sleep(1)

	my_client.close_grippers();

	t.sleep(1)


	print "Moving to intermediate position"

	# MOVE ARM TO INTERMEDIATE POSITION
	my_client.send_command('setPosition -35.01 44.27 64.70 -85.20 -42.12 78.10 47.29')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in intermediate_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1


	print "Placing spanner"

	spanner_place_pos =[-72.69, 57.76, 64.70, -115.42, -65.18, 58.75, 113.75]

	my_client.send_command('setPosition -72.69 57.76 64.70 -115.42 -65.18 58.75 113.75')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in spanner_place_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1

	t.sleep(1)

	my_client.open_grippers();

	t.sleep(1)

	print "Moving to intermediate position"

	# MOVE ARM TO INTERMEDIATE POSITION
	my_client.send_command('setPosition -35.01 44.27 64.70 -85.20 -42.12 78.10 47.29')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in intermediate_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1



	print "Picking up screwdriver"

	screwdriver_pos = [0.0, 51.71, 18.17, -80.86, -19.74, 51.70, 26.83]

	my_client.send_command('setPosition 0.0 51.71 18.17 -80.86 -19.74 51.70 26.83')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in screwdriver_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1

	t.sleep(1)

	my_client.close_grippers();

	t.sleep(1)


	print "Moving to intermediate position"

	# MOVE ARM TO INTERMEDIATE POSITION
	my_client.send_command('setPosition -35.01 44.27 64.70 -85.20 -42.12 78.10 47.29')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in intermediate_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1


	print "Placing screwdriver"

	screwdriver_place_pos = [582.37, 90.48, 231.78, -180, 1.04, -90.37]

	my_client.send_command('setPositionXYZABC 582.37 90.48 231.78 -180 1.04 -90.37')

	[X, Y, Z, A, B, C], time = my_client.ToolPosition 

	current_pos = [X, Y, Z, A, B, C]

	i = 0

	# LOOP THROUGH EACH AXIS AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 1mm
	for desired_pos in screwdriver_place_pos:
		while((current_pos[i] > desired_pos+1) or (current_pos[i] < desired_pos-1)):
			print "Waiting for axis " + str(current_pos[i]) + " to reach desired position"
			[X, Y, Z, A, B, C], time = my_client.ToolPosition 
			current_pos = [X, Y, Z, A, B, C]
			t.sleep(1)
		i += 1

	t.sleep(1)

	my_client.open_grippers();

	t.sleep(2)


	print "Returning home"

	home_pos = [2.09, 23.84, 17.67, -57.30, -8.77, 106.15, 15.97]

	# Move close to a start position.
	my_client.send_command('setPosition 2.09 23.84 17.67 -57.30 -8.77 106.15 15.97')

	[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 

	current_pos = [A1, A2, A3, A4, A5, A6, A7]

	i = 0

	# LOOP THROUGH EACH JOINT AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 0.5 DEGREES
	for desired_pos in home_pos:
		while((current_pos[i] > desired_pos+0.5) or (current_pos[i] < desired_pos-0.5)):
			print "Waiting for joint " + str(i) + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1

	t.sleep(1)

	print "*****************"
	print "****** END ******"
	print "*****************"






