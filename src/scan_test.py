#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from dist_finder import find_disparities

def scan_callback(data):
    disparities = find_disparities(data.ranges)
    rospy.loginfo("Found disparities at: {}".format(disparities))

rospy.init_node("scan_test", anonymous=False)
sub = rospy.Subscriber("/car_5/scan", LaserScan, scan_callback)
rospy.spin()

