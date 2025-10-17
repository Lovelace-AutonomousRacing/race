#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.5	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.9	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	print("annie was here")
	return 0.0



def callback(data):
	global forward_projection

	theta = 80 # you need to try different values for theta 50 +30
	a = getRange(data,theta) # obtain the ray distance for theta
	# starting with 50 then will experiment for better values
	b = getRange(data,30)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	#with FoV of 240 degrees 0 degrees actually 30 degrees
	swing = math.radians(theta)
	# TODO: implement
	# Compute Alpha, AB, and CD..and finally the error.
	radian_theta = math.radians(theta)
	theta_cos = math.cos(radian_theta)
	theta_sin = math.sin(radian_theta)
	alpha = math.atan(((a* theta_cos - b)/ (a * theta_sin)))
	AB = b * (math.cos(a))
	AC = 0.3
	CD = AB + 0.3 * math.sin(a)
	# Your code goes here to determine the projected error as per the algrorithm
    error = desired_distance - CD #desired_trjectory - CD
	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_5/scan",LaserScan,callback)
	rospy.spin()
