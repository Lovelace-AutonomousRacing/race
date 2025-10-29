#!/usr/bin/env python
import rospy
import math

from sensor_msgs.msg import LaserScan

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
threshold = 0.1		# threshold starting at 0.1m needs further tuning
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
car_width = 0.25
def findDisparity(data):
    # data: single message from topic /scan

    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    angle_increment = 0.00436  # angle between each value in ranges

    ranges = data.ranges
    disparities = []
    # step 1 find the disparities in range
    for i in range(1,len(ranges)):
        if abs(ranges[i] - ranges[i-1]) > threshold:
            #append the index of points to represent the disparity
            disparities.append(i)
    # step 2 mask out the disparity that is unsafe for car to pass
    # TODO: find a good tolerance
    tolerance = 0.0

    # (index of right most in current disparity, distance to this disparity)
    filtered_disparities = None
    for i in disparities:
        closest_dis = min(ranges[disparities[i]], ranges[disparities[i-1]])
        numbers_scan= math.ceil(math.radians((tolerance+car_width)*360/2/math.pi/closest_dis)/angle_increment)
        for index in range(1,numbers_scan):
            # mark BAD disparity[index]= -1 means the gap is unsafe to pass
            if ranges[index+i] < closest_dis or i + index >= len(ranges):
                disparities[i] = -1
                break
            if index == numbers_scan-1:
                filtered_disparities.append((i+index,closest_dis))

    # step 3 find the farthest reachable distance
    dis = -1
    index = -1  #refer to the index of point in ranges
    for i, distance in filtered_disparities:
        if distance>dis:
            dis = distance
            index = i

    return index * angle_increment

def callback(data):#####

	#with FoV of 240 degrees 0 degrees actually 30 degrees
    # TODO: implement
    disparities = findDisparity(data)




if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('disparities_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_5/scan",LaserScan,callback)
	rospy.spin()