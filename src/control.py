#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

#PID Control Params
kp = 6.0 #TODO
kd = 0.8 #TODO
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



# Publisher for moving the car.
command_pub = rospy.Publisher('/car_5/offboard/command', AckermannDrive, queue_size = 1)

#function to clip
def bounds(value,min_val,max_val):
    return max(min_val, min(value,max_val))

#function to control steering and velocity
def control(data):
	global prev_error
	global vel_max
	global kp
	global kd


	rospy.loginfo('PID Control Node is Listening to error')

	## Your PID code goes here
	#TODO: Use kp, ki & kd to implement a PID controller
	# 1. Scale the error
    scaled_error = kp * data.pid_error *10

	# 2. Apply the PID equation on error to compute steering, missing ki helps vehicle get back on track if it runs over something which causes it to get off course

	v_theta = scaled_error + (kd * ( data.pid_error - prev_error)) #formula for the PID equation

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

    #define angle

    angle =  v_theta

	# TODO: Make sure the steering value is within bounds [-100,100]
	steering_angle = -angle + servo_offset
	clip_steering_angle = bounds(steering_angle,-100,100)
    rospy.loginfo("Steering Angle = %.2f | Clipped = %.2f" , steering_angle , clip_steering_angle)
	if clip_steering_angle == steering_angle:
	   command.steering_angle = clip_steering_angle
    else:
        rospy.loginfo('Warning: Error in Angle')
        command.steering_angle = clip_steering_angle
	# TODO: Make sure the dynamic velocity is within bounds [0,100]
    vel_max = 45.0                 #max speed
    vel_min = 25.0               # minimum speed for tight turns
    k_vel   = 25.0               #how aggresive to slow down

    turn_ratio = abs(clip_steering_angle)/100.0
    speed = vel_max - k_vel * turn_ratio

    if(data.pid_error >= 1):
        dynamic_vel = 0.7 * data.pid_vel + 0.3 * speed
    else:
        dynamic_vel = speed

    # limit to user range
    target_vel = bounds(dynamic_vel, vel_min, vel_max)
    # clip to command range
    clip_dynamic_vel = bounds(target_vel, 0, 100)

    rospy.loginfo("Dynamic Velocity = %.2f | Clipped = %.2f" , dynamic_vel , clip_dynamic_vel)

	if clip_dynamic_vel == target_vel:
		command.speed= clip_dynamic_vel
	else:
		rospy.loginfo('Warning: Error in Speed')
		command.speed = clip_dynamic_vel

    prev_error = data.pid_error

	# Move the car autonomously
	command_pub.publish(command)


if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	global kp
	global kd
	global ki
	global vel_max
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	ki = input("Enter Ki Value: ")
	vel_max = input("Enter max desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)

	rospy.spin()