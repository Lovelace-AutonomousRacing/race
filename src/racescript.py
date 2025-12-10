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

from sensor_msgs.msg import LaserScan
# want to combine follow gap with pure pursuit in order to overtake

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
global last_scan

wp_seq          = 0
control_polygon = PolygonStamped()
last_scan = LaserScan()

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

def update_laserscan(scan):
    global last_scan
    last_scan = scan

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
    
    lookahead_distance = 2.0

    # so this code just follows the polyline for lookahead_distance units (meters)
    target_point = [i for i in closest_point]
    current_idx = left_point_idx

    dist_from_prev = math.sqrt((plan[current_idx][0]-closest_point[0])**2 + (plan[current_idx][1]-closest_point[1])**2)
    lookahead_distance += dist_from_prev # some cheese because first point isn't on a point
    
    lookahead_distance_cpy = lookahead_distance
    while lookahead_distance_cpy > 0.0:
        dist_to_next = path_resolution[current_idx]
        next_idx = (current_idx+1) % len(plan) # wraps around

        if dist_to_next > lookahead_distance_cpy:
            factor = lookahead_distance_cpy/dist_to_next
            target_point[0] = plan[current_idx][0] + factor*(plan[next_idx][0]-plan[current_idx][0]) # parameterize the line and find based on distance ratio
            target_point[1] = plan[current_idx][1] + factor*(plan[next_idx][1]-plan[current_idx][1])
        lookahead_distance_cpy -= dist_to_next
        current_idx = next_idx

    # calculate desired angle based on target point
    target_x, target_y = target_point
    alpha = math.atan2(target_y - odom_y, target_x - odom_x) - heading
    rotation_radius = lookahead_distance/(2.0*math.sin(alpha))
    delta = math.atan(WHEELBASE_LEN/rotation_radius)

    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # Your code here
    delta_deg = 180.0 * delta / math.pi    
    clipped_angle = max(-100.0, min(100.0, 5*delta_deg))
    command.steering_angle = clipped_angle

    # TODO 6: Implement Dynamic Velocity Scaling instead of a constant speed
    MAX_SPEED = 50.0
    MIN_SPEED = 35.0

    dynamic_speed = MIN_SPEED + ((MAX_SPEED-MIN_SPEED)/2)*(math.sin(alpha + math.pi/2.0)+1) #fn of alpha where f(backwards) = min_speed

    command.speed = dynamic_speed
    command_pub.publish(command)

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
        rospy.Subscriber("/car_5/scan",LaserScan,update_laserscan)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass