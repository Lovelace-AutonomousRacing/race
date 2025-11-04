#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np
# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 3	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = .8	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

#Disparities
def find_disparities(ranges, threshold=0.1):
	ranges = np.array(ranges)
	disparities = np.where(np.abs(np.diff(ranges)) > threshold)[0] #index of disparity
	car_tolerance = 0.05
	car_width = 0.25
	half_width = (car_width/2) + car_tolerance
	angle_increment = data.angle_increment
	closet =[]
	samples_needed =[]
	for i in range(len(disparities)):
		d = disparities[i]
		d1 = ranges[d]
		d2 = ranges[d+1]

		closer = min(d1, d2)
		closet.append(closer)
		theta = math.atan2(half_width, max(closer, 1e-3))   # radians
		samples = int(max(1, round(theta / angle_increment)))
		samples_needed.append(samples)

# based on the closest point you get the distance where it goes off you set car width plus safetly
	return disparities, np.array(closet), np.array(samples_needed)

def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	angle_rad = math.radians(angle)
	index = int ((angle_rad - data.angle_min)/data.angle_increment)
	index = max(0,min(index,len(data.ranges)-1))
	distance = data.ranges[index]
	if math.isinf(distance) or math.isnan(distance):
		distance = data.range_max
	return distance


def callback(data):#####
	global forward_projection

	theta = -20 # you need to try different values for theta 50 +30
	a = getRange(data,theta) # obtain the ray distance for theta
	# starting with 50 then will experiment for better values
	b = getRange(data,-90)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)
	print('b',b)
	#with FoV of 240 degrees 0 degrees actually 30 degrees
	# TODO: implement
	# Compute Alpha, AB, and CD..and finally the error.
	radian_theta = math.radians(theta + 90)
	theta_cos = math.cos(radian_theta)
	theta_sin = math.sin(radian_theta)
	alpha = math.atan(((a* theta_cos - b)/ (a * theta_sin)))
	print('alpha',alpha)
	AB = b * (math.cos(alpha))
	CD = AB + (forward_projection) * math.sin(alpha)
	print('CD',CD)
	# Your code goes here to determine the projected error as per the algrorithm
	error = CD - desired_distance  #desired_trjectory - CD
	print('error',error)
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