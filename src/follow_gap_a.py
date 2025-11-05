#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

servo_offset = 0.0
angle_range = 240
threshold = 0.1
car_width = 0.25
tolerance = 0.05
max_vel = 35.0
min_vel = 20.0

command_pub = rospy.Publisher('/car_5/offboard/command', AckermannDrive, queue_size=1)

def preprocess_lidar(data):
	ranges = np.array(data.ranges)
	ranges = np.nan_to_num(ranges, nan=data.range_max, posinf=data.range_max)
	return np.clip(ranges, 0, data.range_max)

def find_disparities(ranges):
	disparities = []
	for i in range(1, len(ranges)):
		if abs(ranges[i] - ranges[i - 1]) > threshold:
			disparities.append((i - 1, i))
	return disparities

def mask_disparities(ranges, disparities, angle_increment):
	for left, right in disparities:
		if ranges[left] < ranges[right]:
			close_index = left
		else:
			close_index = right

		close_dist = ranges[close_index]
		angle_width = math.asin(min(1.0, (car_width / 2.0 + tolerance) / close_dist))
		num_samples = int(math.ceil(angle_width / angle_increment))

		if close_index == left:
			start = max(0, right - num_samples)
			end = right
		else:
			start = left
			end = min(len(ranges) - 1, left + num_samples)

		for j in range(start, end):
			if ranges[j] > close_dist:
				ranges[j] = close_dist

	return ranges

def find_best_direction(data, ranges):
	angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
	mask = np.logical_and(angles > -math.radians(90), angles < math.radians(90))
	front_ranges = ranges[mask]
	front_angles = angles[mask]
	max_index = np.argmax(front_ranges)
	best_angle = front_angles[max_index]
	return best_angle, front_ranges[max_index]

def callback(data):
	ranges = preprocess_lidar(data)
	disparities = find_disparities(ranges)
	filtered = mask_disparities(ranges.copy(), disparities, data.angle_increment)
	best_angle, farthest_dist = find_best_direction(data, filtered)
	steering = (best_angle / (math.radians(90))) * 100.0
	steering = max(min(steering + servo_offset, 100), -100)
	vel = max_vel - (abs(steering) / 100.0) * (max_vel - min_vel)
	command = AckermannDrive()
	command.steering_angle = steering
	command.speed = vel
	rospy.loginfo("Best Angle: %.2f deg | Steering: %.2f | Speed: %.2f", 
				  math.degrees(best_angle), steering, vel)
	command_pub.publish(command)

if __name__ == '__main__':
	rospy.init_node('follow_gap', anonymous=True)
	rospy.Subscriber("/car_5/scan", LaserScan, callback)
	rospy.spin()
