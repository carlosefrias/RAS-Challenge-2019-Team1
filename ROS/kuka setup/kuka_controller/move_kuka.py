#!/usr/bin/env python

# KUKA API for ROS

# Elliott White  elliott-white@hotmail.co.uk

# This script generates a ROS node for comunicating with KUKA iiwa
# Dependencies: conf.txt, ROS server, Rospy, KUKA iiwa java SDK, KUKA iiwa robot.

# Waits on voice commands for an object to be picked up. Locates object in another script using a kinect then
# navigates to object and picks it up
#######################################################################################################################
from client_lib import *
import rospy
import time as t
import tf
import math
from std_msgs.msg import String
from std_msgs.msg import Float64


object_pub = rospy.Publisher('object_to_grab', String, queue_size=10)
	# Making a connection object.
my_client = kuka_iiwa_ros_client()

def setup():

	global my_client



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
			print "Waiting for joint " + i + " to reach desired position"
			[A1, A2, A3, A4, A5, A6, A7], time = my_client.JointPosition 
			current_pos = [A1, A2, A3, A4, A5, A6, A7]
			t.sleep(1)
		i += 1

	t.sleep(1)



def grab_objects():

	while True:

		received_transcript = False

		if(my_client.Transcript == "pick up the hammer" or my_client.Transcript == "pick up the Hammer"):

			# pick up hammer
			rospy.loginfo("Picking up hammer")

			object_msg = "hammer"
        	object_pub.publish(object_msg)
        	received_transcript = True

        if(my_client.Transcript == "pick up the spanner" or my_client.Transcript == "pick up the Spanner"):
        	rospy.loginfo("Picking up spanner")
        	object_msg = "spanner"
        	object_pub.publish(object_msg)
        	received_transcript = True

		if(my_client.Transcript == "pick up the screw driver" or my_client.Transcript == "pick up the Screw driver"):

			rospy.loginfo("Picking up screwdriver")
			object_msg = "screwdriver"
			object_pub.publish(object_msg)
			received_transcript = True

		if received_transcript:
			
			# CONFIRM THIS - OFFSET MUST BE IN MILLIMETRES
			print "Waiting for offset"
			offset = rospy.wait_for_message("/offset", String)

			offset = str(offset)

			offset = offset.split()

			print "Aligning camera"

			# GRAB TOOL POS THEN MOVE BETWEEN KINECT AND TOOL
			[X, Y, Z, A, B, C], time = my_client.ToolPosition 

			# ADJUST X, Y & A FOR OFFSET OF OBJECT
			X = X + offset[0]
			Y = Y + offset[1]
			
			camera_aligned_pos = [X, Y]

			position_message = "setPositionXYZABC " + X + " " + Y + " - - - -"

			# MOVE TO NEW LOCATION USING OFFSET SO CAMERA IS NOW CENTRED ABOVE OBJECT
			my_client.send_command(position_message)

			[X, Y, Z, _, _, _], time = my_client.ToolPosition 

			current_pos = [X, Y, Z]

			i = 0

			# LOOP THROUGH EACH AXIS AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 1mm
			for desired_pos in camera_aligned_pos:
				while((current_pos[i] > desired_pos+1) or (current_pos[i] < desired_pos-1)):
					print "Waiting for joint " + i + " to reach desired position"
					[X, Y, Z, A, B, C], time = my_client.ToolPosition 
					current_pos = [X, Y, Z, A, B, C]
					t.sleep(1)
				i += 1

			t.sleep(1)

			# GET TF FROM TOOL TO CAMERA
			listener = tf.TransformListener()

			try:
				(trans,rot) = listener.lookupTransform('kinect_frame', 'tool_tcp', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				

			# ADJUST X & Y FOR OFFSET OF CAMERA TO TOOL AND ADD IN ROTATION OF OBJECT
			X = X + trans[0]
			Y = Y + trans[1]
			A = A + offset[2]

			print "Aligning tool"
			
			tool_aligned_pos = [X, Y, Z, A]
			
			position_message = "setPositionXYZABC " + X + " " + Y + " - " + A + " - -"
			
			# MOVE TO NEW LOCATION USING OFFSET SO TOOL IS NOW CENTRED ABOVE OBJECT
			my_client.send_command(position_message)

			[X, Y, Z, A, _, _], time = my_client.ToolPosition 

			current_pos = [X, Y, Z, A]

			i = 0
			# LOOP THROUGH EACH AXIS AND WAIT FOR IT TO REACH DESIRED POSITION WITHIN 1mm
			for desired_pos in tool_aligned_pos:
				while((current_pos[i] > desired_pos+1) or (current_pos[i] < desired_pos-1)):
					print "Waiting for joint " + i + " to reach desired position"
					[X, Y, Z, A, B, C], time = my_client.ToolPosition 
					current_pos = [X, Y, Z, A, B, C]
					t.sleep(1)
			i += 1
			t.sleep(1)

			print "Moving to object"

			sponge_height = 20

			# NOW NEED TO MOVE DOWN TO OBJECT
			Z_target = 10 + sponge_height

			# set Z to 10mm + height of sponge
          	position_message = "setPositionXYZABC " + " - - " + Z + " - - -"

          	# MOVE GRIPPER DOWN TO OBJECT 10mm ABOVE SPONGE
          	my_client.send_command(position_message)

          	[_, _, Z, _, _, _], time = my_client.ToolPosition

          	current_pos = Z_current

          	# WAIT FOR Z AXIS TO REACH DESIRED POSITION WITHIN 1m
          	while((current_pos > Z_target+1) or (current_pos < Z_target-1)):
          		print "Waiting for joint " + i + " to reach desired position"
          		[_, _, Z, _, _, _], time = my_client.ToolPosition
          		current_pos = Z
          		t.sleep(1)

			my_client.close_grippers()

			t.sleep(1)

			# *****************************************
			# NOW MAYBE WAIT FOR ANOTHER VOICE COMMAND?
			# *****************************************

			# RESET TRANSCRIPT STRING SO IF STATEMENTS DON'T ENTER AGAIN
			my_client.Transcript = ''

			received_transcript = False



if __name__ == '__main__':

	global my_client

	setup()

	print "Done robot setup"

	grab_objects();
	




