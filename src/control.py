#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

#PID Control Params
kp = 0.0 #TODO
kd = 0.0 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0     # previous PID error
prev_angle = 0.0     # accumulated steering angle

# Velocity parameters
vel_input = 15.0	# desired base velocity
vel_scale = 0.0      # scale factor for reducing speed with error

# Publisher for moving the car.
command_pub = rospy.Publisher('/car_5/offboard/command', AckermannDrive, queue_size = 1)

#function to clip
def bounds(value,min_val,max_val):
    return max(min_val, min(value,max_val))

#function to control steering and velocity
def control(data):
	global prev_error
	global prev_angle
	global vel_input
	global kp
	global kd
	global vel_scale

	rospy.loginfo('PID Control Node is Listening to error')

	# 1. Scale the error
	scaled_error = kp * data.pid_error

	# 2. Apply the PID equation (PD form)
	v_theta = scaled_error + (kd * (data.pid_error - prev_error))

	# 3. Update steering angle using accumulated effect
	angle = prev_angle + v_theta
	prev_angle = angle

	# 4. Store current error for derivative use
	prev_error = data.pid_error

	# 5. Clamp steering
	command = AckermannDrive()
	steering_angle = -angle + servo_offset
	clip_steering_angle = bounds(steering_angle, -100, 100)
	rospy.loginfo("Steering Angle = %.2f | Clipped = %.2f", steering_angle, clip_steering_angle)
	command.steering_angle = clip_steering_angle

	# 6. Adjust speed dynamically based on wall-following error
	mag = vel_input - vel_scale * abs(data.pid_error)

	if data.pid_error >= 1:
		speed = 0.7 * getattr(data, 'pid_vel', 0) + 0.3 * mag  # safe if pid_vel doesn't exist
	else:
		speed = mag

	clip_speed = bounds(speed, 0, 100)
	if clip_speed != speed:
		rospy.loginfo('Warning: Error in Speed')
	command.speed = clip_speed

	# 7. Publish command
	command_pub.publish(command)

if __name__ == '__main__':
	global kp, kd, ki, vel_input, vel_scale
	kp = float(input("Enter Kp Value: "))
	kd = float(input("Enter Kd Value: "))
	ki = float(input("Enter Ki Value: "))
	vel_input = float(input("Enter desired velocity: "))
	vel_scale = float(input("Enter Velocity Scaling Value: "))
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
