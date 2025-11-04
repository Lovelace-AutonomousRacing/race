#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FTG CONTROLLER NODE (Python 2.7)
--------------------------------
This node:
 - Subscribes to 'error' topic receiving:
       msg.pid_error = target_angle (radians)
       msg.pid_vel   = distance to target (not required here)
 - Converts target angle → steering command in range [-100, 100]
 - Applies dynamic speed control based on steering magnitude
 - Publishes AckermannDrive to command topic
"""

from __future__ import print_function
import rospy
import math
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# ====================== PARAMETERS ============================
CMD_TOPIC = "/car_5/offboard/command"
SERVO_OFFSET = 0.0          # Adjust if wheels point slightly left/right when 0
MAX_SPEED = 35.0            # Forward speed when steering straight
MIN_SPEED = 15.0            # Slowest allowed speed when steering hard
# =============================================================

def bounds(value, lo, hi):
    """Clamp value into [lo, hi]."""
    return max(lo, min(hi, value))

def angle_to_steering(angle_rad):
    """
    Convert target angle (radians) -> steering percent [-100 to 100].
    - Lidar 0 rad = forward
    - Clip angle to [-90°, 90°]
    - Map linearly to [-100, 100]
    """
    deg = math.degrees(angle_rad)
    deg = max(-90.0, min(90.0, deg))
    steering_cmd = (deg / 90.0) * 100.0
    return steering_cmd

def dynamic_speed(steering_cmd):
    """
    Reduce speed as steering angle increases.
    Straight = MAX_SPEED
    Hard turns = MIN_SPEED
    """

    dynamic_vel = ((MAX_SPEED-MIN_SPEED)/2)*(math.sin((math.pi*abs(steering_cmd))/100.0 + math.pi/2.0)+1) + MIN_SPEED

    
    return dynamic_vel

def callback(msg):
    """
    On receiving target data:
    1. Extract angle
    2. Convert to steering command
    3. Compute speed
    4. Publish AckermannDrive
    """
    target_angle = msg.pid_error  # radians from FTG
    steering_cmd = angle_to_steering(target_angle)

    # Optional: invert sign depending on steering orientation
    steering_cmd = -steering_cmd + SERVO_OFFSET
    steering_cmd = bounds(steering_cmd, -100.0, 100.0)

    speed_cmd = dynamic_speed(steering_cmd)
    speed_cmd = bounds(speed_cmd, 0.0, 100.0)

    command = AckermannDrive()
    command.steering_angle = steering_cmd
    command.speed = speed_cmd

    pub.publish(command)

if __name__ == "__main__":
    rospy.init_node("ftg_controller")
    pub = rospy.Publisher(CMD_TOPIC, AckermannDrive, queue_size=1)
    rospy.Subscriber("error", pid_input, callback, queue_size=1)
    rospy.loginfo("FTG controller running (Python 2.7)")
    rospy.spin()
