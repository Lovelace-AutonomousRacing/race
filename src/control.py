#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
kp = 6.0 #TODO
kd = 7.0 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 15.0	#TODO

# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_5/offboard/command', AckermannDrive, queue_size = 1)
def getPrevInfo(msg):
	prev_angle = msg.steering.angle
def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle = 0.0
	global prev_angle

	print("PID Control Node is Listening to error")

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
	rospy.Subscriber('/car_5/offboard/command', AckermannDrive, getPrevInfo)
	# 1. Scale the error
	error = error * 1
	# 2. Apply the PID equation on error to compute steering
	v_theta = kp * data + kd(prev_error - data)
	angle = prev_angle - v_theta

	command.steering_angle_velocity= v_theta

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	if (angle > -100) and (angle < 100):
		command.steering_angle = angle
	else
		rospy.loginfo('error angle')
		command.angle = 0

	# TODO: Make sure the velocity is within bounds [0,100]
	if (vel_input > 0) and (vel_input < 100):
		command.speed = vel_input
	else
		rospy.loginfo('error velocity')
		command.speed = 0
	
	
	# Move the car autonomously
	rospy.loginfo(command.steering_angle)
	rospy.loginfo(command.speed)
	rospy.loginfo(command.steering_angle_velocity)
	rospy.loginfo(command.acceleration)
	command.acceleration = 0

	command_pub.publish(command)

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_input
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	ki = input("Enter Ki Value: ")
	vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
