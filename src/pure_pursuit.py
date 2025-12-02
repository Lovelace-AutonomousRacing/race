#!/usr/bin/env python

# Import necessary libraries
import rospy
import os
import sys
import csv
import math
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf

# Global variables for storing the path, path resolution, frame ID, and car details
plan                = []
path_resolution     = []
frame_id            = 'map'
car_name            = str(sys.argv[1])
trajectory_name     = str(sys.argv[2])

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 1)
polygon_pub         = rospy.Publisher('/{}/purepursuit_control/visualize'.format(car_name), PolygonStamped, queue_size = 1)
raceline_pub        = rospy.Publisher("/raceline", Path, queue_size=1, latch=True)

# Global variables for waypoint sequence and current polygon
global wp_seq
global curr_polygon

wp_seq          = 0
control_polygon = PolygonStamped()

def construct_path():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    file_path = os.path.expanduser('/home/nvidia/catkin_ws/src/f1tenth_purepursuit/path/{}.csv'.format(trajectory_name))
    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            plan.append(waypoint)

    # Convert string coordinates to floats and calculate path resolution
    for index in range(0, len(plan)):
        for point in range(0, len(plan[index])):
            plan[index][point] = float(plan[index][point])

    for index in range(1, len(plan)):
         dx = plan[index][0] - plan[index-1][0]
         dy = plan[index][1] - plan[index-1][1]
         path_resolution.append(math.sqrt(dx*dx + dy*dy)) # path_resolution[0] is dist from plan[0] to plan[1]
    
    dx = plan[-1][0] - plan[0][0]
    dy = plan[-1][1] - plan[0][1]
    path_resolution.append(math.sqrt(dx*dx + dy*dy)) # make last one loop back to beginning, idk if this is intended

    raceline_path = Path()
    raceline_path.header.frame_id = "map"

    for pt in plan:
        px, py = pt[0], pt[1]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = px
        pose.pose.position.y = py
        raceline_path.poses.append(pose)

    raceline_pub.publish(raceline_path)
    rospy.loginfo("Published latched raceline to /raceline")


# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN       = 0.325

def purepursuit_control_node(data):
    # Main control function for pure pursuit algorithm

    # Create an empty ackermann drive message that we will populate later with the desired steering angle and speed.
    command = AckermannDrive()

    global wp_seq
    global curr_polygon

    # Obtain the current position of the race car from the inferred_pose message
    odom_x = data.pose.position.x
    odom_y = data.pose.position.y


    # TODO 1: The reference path is stored in the 'plan' array.
    # Your task is to find the base projection of the car on this reference path.
    # The base projection is defined as the closest point on the reference path to the car's current position.
    # Calculate the index and position of this base projection on the reference path.
    
    # Your code here

    # closest_idx = 0
    # min_dist = 1000
    # for i in range(len(plan)):
    #     x = plan[i][0]
    #     y = plan[i][1]
    #     squared_dist = (odom_x-x)**2 + (odom_y-y)**2

    #     if squared_dist < min_dist:
    #         closest_idx = i
    #         min_dist = squared_dist
    # min_dist = math.sqrt(min_dist)

    closest_point = [0, 0] # closest point on the plan line
    left_point_idx = 0
    min_dist = 1000


    for i in range(len(plan)):
        x1 = plan[i][0]
        y1 = plan[i][1]
        x2 = plan[(i+1)%len(plan)][0]
        y2 = plan[(i+1)%len(plan)][1]

        dx = x2 - x1 # getting minimum distance to the line segment
        dy = y2 - y1

        # Projection scalar
        den = dx*dx + dy*dy
        if den < 1e-6:
            continue
        t = ((odom_x - x1)*dx + (odom_y - y1)*dy) / den # added avoidance for division by zero
        t = max(0, min(1, t))

        # Closest point
        qx = x1 + t * dx
        qy = y1 + t * dy
        dist = math.hypot(odom_x - qx, odom_y - qy)

        if dist < min_dist:
            min_dist = dist
            closest_point = [qx, qy]
            left_point_idx = i

    
    # Calculate heading angle of the car (in radians)
    # roll pitch yaw euler
    heading = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                        data.pose.orientation.y,
                                                        data.pose.orientation.z,
                                                        data.pose.orientation.w))[2]
    

    # TODO 2: You need to tune the value of the lookahead_distance
    lookahead_distance = 1.0


    # TODO 3: Utilizing the base projection found in TODO 1, your next task is to identify the goal or target point for the car.
    # This target point should be determined based on the path and the base projection you have already calculated.
    # The target point is a specific point on the reference path that the car should aim towards - lookahead distance ahead of the base projection on the reference path.
    # Calculate the position of this goal/target point along the path.

    # Your code here

    # idk how to find the lookahead point from the intersections of the lookahead radius circle and the polyline (plan segments)
    # so this code just follows the polyline for lookahead_distance units (meters)
    target_point = [i for i in closest_point]
    current_idx = left_point_idx

    dist_from_prev = math.sqrt((plan[current_idx][0]-closest_point[0])**2 + (plan[current_idx][1]-closest_point[1])**2)
    lookahead_distance += dist_from_prev # some cheese because first point isn't on a point
    
    while lookahead_distance > 0.0:
        dist_to_next = path_resolution[current_idx]
        next_idx = (current_idx+1) % len(plan) # wraps around
        if dist_to_next > lookahead_distance:
            factor = lookahead_distance/dist_to_next
            target_point[0] = factor*(plan[next_idx][0]-plan[current_idx][0]) # parameterize the line and find based on distance ratio
            target_point[1] = factor*(plan[next_idx][1]-plan[current_idx][1])
        lookahead_distance -= dist_to_next
        current_idx = next_idx

    # TODO 4: Implement the pure pursuit algorithm to compute the steering angle given the pose of the car, target point, and lookahead distance.
    # Your code here
    target_x, target_y = target_point
    alpha = math.atan2(target_y - odom_y, target_x - odom_x)
    if alpha < 0.0:
        alpha += 2*math.pi # normalize to [0, 2*pi]
    rotation_radius = lookahead_distance/(2.0*math.sin(alpha))
    delta = math.atan((2.0*WHEELBASE_LEN*math.sin(alpha))/rotation_radius)

    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # Your code here    
    clipped_angle = max(-100.0, min(100.0, delta))
    command.steering_angle = clipped_angle

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed
    MAX_SPEED = 4.0
    MIN_SPEED = 1.0

    steer_fraction = abs(delta)/STEERING_RANGE

    speed = MAX_SPEED * (1 - steer_fraction)
    speed = max(speed, MIN_SPEED)

    command.speed = speed

    # Visualization code
    # Make sure the following variables are properly defined in your TODOs above:
    # - odom_x, odom_y: Current position of the car
    # - pose_x, pose_y: Position of the base projection on the reference path
    # - target_x, target_y: Position of the goal/target point

    # These are set to zero only so that the template code builds. 
    pose_x, pose_y = closest_point
    target_x, target_y = target_point


    base_link    = Point32()
    nearest_pose = Point32()
    nearest_goal = Point32()
    base_link.x    = odom_x
    base_link.y    = odom_y
    nearest_pose.x = pose_x
    nearest_pose.y = pose_y
    nearest_goal.x = target_x
    nearest_goal.y = target_y
    control_polygon.header.frame_id = frame_id
    control_polygon.polygon.points  = [nearest_pose, base_link, nearest_goal]
    control_polygon.header.seq      = wp_seq
    control_polygon.header.stamp    = rospy.Time.now()
    wp_seq = wp_seq + 1
    polygon_pub.publish(control_polygon)

if __name__ == '__main__':

    try:

        rospy.init_node('pure_pursuit', anonymous = True)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, purepursuit_control_node)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass