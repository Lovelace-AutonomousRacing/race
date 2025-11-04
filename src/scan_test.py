#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from find_disparities import find_disparities   # adjust import path if needed

def scan_callback(data):
    disparities, dis_closet, dis_samples = find_disparities(data.ranges, data=data)
    rospy.loginfo("Found disparities at: %s", disparities)
    rospy.loginfo("Found closet (closest distances) at: %s", dis_closet)
    rospy.loginfo("Found samples needed at: %s", dis_samples)

if __name__ == "__main__":
    rospy.init_node("scan_test", anonymous=False)
    sub = rospy.Subscriber("/car_5/scan", LaserScan, scan_callback)
    rospy.spin()

