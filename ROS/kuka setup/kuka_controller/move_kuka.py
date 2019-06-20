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
import time
import tf
import math
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



def grab_objects():

	while True:

		received_transcript = False

		if(my_client.Transcript == "pick up hammer"):

			# pick up hammer
			rospy.loginfo("Picking up hammer")

			object_msg = "hammer"
        	object_pub.publish(object_msg)

        	received_transcript = True

		elif(my_client.Transcript == "pick up screwdriver"):
			# pick up screw driver
			rospy.loginfo("Picking up screwdriver")

			object_msg = "screwdriver"
        	object_pub.publish(object_msg)

        	received_transcript = True

		elif(my_client.Transcript == "pick up spanner"):
			# pick up spanner
			rospy.loginfo("Picking up spanner")

			object_msg = "spanner"
        	object_pub.publish(object_msg)

        	received_transcript = True


        if received_transcript:
			
			# CONFIRM THIS - OFFSET MUST BE IN MILLIMETRES
			print "Waiting for offset_x"
			offset_x = rospy.wait_for_message("/offset", Float)
			print "Waiting for offset_y"
			offset_y = rospy.wait_for_message("/offset", Float)
			print "Waiting for offset_angle"
			offset_angle = rospy.wait_for_message("/offset", Float)

			# GRAB TOOL POS THEN MOVE BETWEEN KINECT AND TOOL
			X, Y, Z, A, B, C, time = my_client.ToolPosition 

			# ADJUST X, Y & A FOR OFFSET OF OBJECT
			X = X + offset_x
			Y = Y + offset_y
			
			position_message = "setPositionXYZABC " + X + " " + Y + " " + Z + " " + A + " " + B + " " + C

			# MOVE TO NEW LOCATION USING OFFSET SO CAMERA IS NOW CENTRED ABOVE OBJECT
			my_client.send_command(position_message)

			time.sleep(5)

			# GET TF FROM TOOL TO CAMERA
			listener = tf.TransformListener()
			try:
            	(trans,rot) = listener.lookupTransform('kinect2_rgb_optical_frame', 'tool_tcp', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            	continue

            # ADJUST X & Y FOR OFFSET OF CAMERA TO TOOL AND ADD IN ROTATION OF OBJECT
            X = X + trans[0]
            Y = Y + trans[1]
            A = A + math.radians(offset_angle)

            position_message = "setPositionXYZABC " + X + " " + Y + " " + Z + " " + A + " " + B + " " + C

			# MOVE TO NEW LOCATION USING OFFSET SO TOOL IS NOW CENTRED ABOVE OBJECT
			my_client.send_command(position_message)

			time.sleep(5)

			sponge_height = 20
          	
          	# NOW NEED TO MOVE DOWN TO OBJECT
          	Z = 10 + sponge_height		# set Z to 10mm + height of sponge

          	position_message = "setPositionXYZABC " + X + " " + Y + " " + Z + " " + A + " " + B + " " + C

			# MOVE GRIPPER DOWN TO OBJECT 10mm ABOVE SPONGE
			my_client.send_command(position_message)

			time.sleep(10)
			my_client.close_grippers();


			# *****************************************
			# NOW MAYBE WAIT FOR ANOTHER VOICE COMMAND?
			# *****************************************

			# RESET TRANSCRIPT STRING SO IF STATEMENTS DON'T ENTER AGAIN
			my_client.Transcript = ''











if __name__ == '__main__':

	global my_client

	setup()

	print "Done robot setup"

	grab_objects();
	




