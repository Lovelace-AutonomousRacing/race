#!/usr/bin/env python
import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

# Some useful variable declarations.
servo_offset = 0.0
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
threshold = 0.1		# threshold starting at 0.1m needs further tuning
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
car_width = 0.25

max_vel = 15.0
min_vel = 5.0
tolerance = 0.1  # extra buffer for gap

command_pub = rospy.Publisher('/car_5/offboard/command', AckermannDrive, queue_size = 1)

def findDisparity(data):
    # data: single message from topic /scan
    angle_increment = data.angle_increment  # angle between each value in ranges
    ranges = np.array(data.ranges)

    disparities = np.where(np.abs(np.diff(ranges)) > threshold)[0]

    for idx in disparities:
        left = idx
        right = idx + 1

        if ranges[left] > ranges[right]:
            extend_right = False  # tells us whether to extend right or left
            close_dist = ranges[left]
            close_index = left
        else:
            extend_right = True
            close_dist = ranges[right]
            close_index = right

        numbers_scan = int(math.atan2(tolerance + car_width / 2.0, close_dist) / angle_increment) + 1

        # TODO: extend disparities by changing ranges
        if extend_right:
            for j in range(1, numbers_scan):
                if close_index + j < len(ranges):
                    if ranges[close_index + j] > close_dist:
                        ranges[close_index + j] = close_dist
        else:
            for j in range(1, numbers_scan):
                if close_index - j >= 0:
                    if ranges[close_index - j] > close_dist:
                        ranges[close_index - j] = close_dist

    # step 3 find the farthest reachable distance
    index = np.argmax(ranges)
    return data.angle_min + index * angle_increment, ranges[index]

def callback(data):#####
	#with FoV of 240 degrees 0 degrees actually 30 degrees
    best_angle, closest_dist = findDisparity(data)
    command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
    steering_angle = best_angle + servo_offset
    clip_steering_angle = max(min(steering_angle, 100), -100)
    command.steering_angle = clip_steering_angle
    
    dynamic_vel = max(min(max_vel, closest_dist * 2), min_vel)
    # TODO: implement based on gap??

    command.speed = dynamic_vel
    command_pub.publish(command)

if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('disparities_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_5/scan",LaserScan,callback)
	rospy.spin()
