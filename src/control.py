#!/usr/bin/env python
import math
import rospy
import numpy as np
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params
# begin with kp tuning have ki and kd to 0
kp = 10.0 #TODO
#once the system starts to oscillate around the set point add kd
kd = 0.0 #TODO
# is a steady state error remains gradually increase Ki
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
# TODO: Use the correct topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_5/offboard/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle 
	global mag
	global speed
	angle= 0.0

	print("PID Control Node is Listening to error")

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
    #Define setpoint
	# 1. Scale the error
    error = data.pid_error
	# 2. Apply the PID equation on error to compute steering
    steering_output = kp * error
	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()
	# TODO: Make sure the steering value is within bounds [-100,100]
	command.steering_angle = np.clip(angle, -100, 100)

	# TODO: Make sure the velocity is within bounds [0,100]
	mag = vel_input - vel_scale * abs(data.pid_error)# reduces speed when error increses
	if (data.pid_error >= 1):
		speed = 0.7 * data.pid_vel + 0.3 * mag
	else:
		speed = mag
	if (speed > 0) and (speed < 100):
		command.speed = speed 
	else:
		rospy.loginfo('error velocity')
		command.speed = 0

	# Move the car autonomously
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