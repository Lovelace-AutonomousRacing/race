#!/usr/bin/env python
import rospy
import math

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

# Some useful variable declarations.
servo_offset = 0.0
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
threshold = 0.1		# threshold starting at 0.1m needs further tuning
car_tolerance = 0.1 # extra tolerance for gap
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
car_width = 0.25

max_vel = 15.0
command_pub = rospy.Publisher('/car_5/offboard/command', AckermannDrive, queue_size = 1)

def findDisparity(data):
    # data: single message from topic /scan

    angle_increment = data.angle_increment  # angle between each value in ranges

    ranges = []
    for i,v  in enumerate(data.ranges):
        angle = data.angle_min + i * angle_increment
        if angle < 90 and angle > -90: ranges.append(v)

    disparities = []
    # step 1 find the disparities in range
    for i in range(1,len(ranges)):
        if abs(ranges[i] - ranges[i-1]) > threshold:
            #append the index of points to represent the disparity
            disparities.append((i-1, i))

    for i in range(len(disparities)):
        if ranges[disparities[i][0]] > ranges[disparities[i][1]]:
            extend_right = False  # tells us whether to extend right or left
            close_dist = ranges[disparities[i][0]]
        else:
            extend_right = True
            close_dist = ranges[disparities[i][1]]

        numbers_scan = int(math.ceil(math.radians((car_tolerance+car_width/2)/(2*math.pi*close_dist))/angle_increment))

        # TODO: extend disparities by changing ranges
        if extend_right:
            for j in range(1,numbers_scan):
                if i+j >= len(ranges): 
                    break
                if ranges[i+j] > close_dist:
                    ranges[i+j] = close_dist
        else:
            for j in range(1,numbers_scan):
                if i-j < 0: 
                    break
                if ranges[i-j] > close_dist:
                    ranges[i-j] = close_dist



    # step 3 find the farthest reachable distance
    dis = -1
    index = -1  #refer to the index of point in ranges
    for i, distance in enumerate(ranges):
        if distance>dis:
            dis = distance
            index = i

    return data.angle_min + index * angle_increment

def callback(data):#####
	#with FoV of 240 degrees 0 degrees actually 30 degrees
    # TODO: implement
    best_angle = findDisparity(data)
    command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
    steering_angle = best_angle + servo_offset
    clip_steering_angle = min(max(steering_angle, -100), 100)

    rospy.loginfo("Steering Angle = %.2f | Clipped = %.2f" , steering_angle , clip_steering_angle)
    command.steering_angle = clip_steering_angle
    
    dynamic_vel = max_vel
    # TODO: implement based on gap??

    command.speed = dynamic_vel
    command_pub.publish(command)




if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('disparities_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_5/scan",LaserScan,callback)
	rospy.spin()