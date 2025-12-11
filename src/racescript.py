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

# Global variables for waypoint sequence and current polygon
global LAST_SCAN
LAST_SCAN = LaserScan()

# Tunable parameters
MAX_SPEED = 50
MIN_SPEED = 35
LOOKAHEAD = 1.5
THRESHOLD = 0.15		# updated threshold
CAR_TOLERANCE = 0.22 # increased safety margin
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
CAR_WIDTH = 0.30  # increased car width

# Smoothing and rate-limit parameters to reduce oscillation
STEER_SMOOTH_ALPHA = 0.35  # EMA alpha for steering (0..1). Higher = less smoothing
SPEED_SMOOTH_ALPHA = 0.20  # EMA alpha for speed
MAX_STEER_STEP = 8.0       # max degrees change per control update
MAX_SPEED_STEP = 5.0       # max speed change per control update

# previous command state (for smoothing)
prev_steering = 0.0
prev_speed = MIN_SPEED

# Overtake conservatism parameters (can be overridden via ROS params)
OVERTAKE_GAP_MIN = 1.5      # minimum extra free distance that makes overtaking attractive (meters)
OVERTAKE_PP_DIST_MAX = 0.6  # require planned-path distance below this to even consider overtaking
OVERTAKE_STEER_SCALE = 2.5  # scale applied to disparity steering during overtakes (was 5.0)
OVERTAKE_SPEED_SCALE = 0.8  # fraction of MAX_SPEED to use during overtakes (slower for safety)

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


# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN       = 0.325

def update_laserscan(scan):
    global LAST_SCAN
    LAST_SCAN = scan

def get_dist(angle):
	angle_rad = math.radians(angle)
	index = int ((angle_rad - LAST_SCAN.angle_min)/LAST_SCAN.angle_increment)
	index = max(0,min(index,len(LAST_SCAN.ranges)-1))
	distance = LAST_SCAN.ranges[index]
	if math.isinf(distance) or math.isnan(distance):
		distance = LAST_SCAN.range_max
	return distance

def disparity_extender():
    if not LAST_SCAN.ranges:
        return 0.0, 0.0

    angle_increment = LAST_SCAN.angle_increment  # angle between each value in ranges
    angle_min = LAST_SCAN.angle_min # updated later to match our new ranges
    ranges = []
    last_value = LAST_SCAN.range_max
    for i,v  in enumerate(LAST_SCAN.ranges): 
        is_bad = math.isnan(v) or v > LAST_SCAN.range_max or v < LAST_SCAN.range_min
        if not is_bad:
            last_value = v
        angle = LAST_SCAN.angle_min + i * angle_increment
        if angle < math.pi/2 and angle > -math.pi/2: 
            if not ranges:
                angle_min = angle
            ranges.append(last_value)
            angle_max = angle

    disparities = []
    # step 1 find the disparities in range
    for i in range(1,len(ranges)):
        if abs(ranges[i] - ranges[i-1]) > THRESHOLD:
            #append the index of points to represent the disparity
            disparities.append((i-1, i))

    for i in range(len(disparities)):
        left, right = disparities[i]
        if ranges[left] < ranges[right]:
            extend_right = True  # tells us whether to extend right or left
            close_idx = left
        else:
            extend_right = False
            close_idx = right
        close_dist = ranges[close_idx]

        theta = math.atan2(CAR_TOLERANCE + CAR_WIDTH / 2.0, ranges[close_idx])
        numbers_scan = int(math.ceil(theta/angle_increment))

        # TODO: extend disparities by changing ranges
        if extend_right:
            for j in range(1, numbers_scan):
                if close_idx+j >= len(ranges): 
                    break
                ranges[close_idx+j] = min(ranges[close_idx+j], close_dist)
        else:
            for j in range(1, numbers_scan):
                if close_idx-j < 0: 
                    break
                ranges[close_idx-j] = min(ranges[close_idx-j], close_dist)

    # step 3 find the farthest reachable distance
    dis = -1
    index = -1  #refer to the index of point in ranges
    for i, distance in enumerate(ranges):
        if distance>dis:
            dis = distance
            index = i
    
    distance_margin = 0.3 #go for the middle of the gap
    left = index
    right = index
    while(left >= 0 and ranges[left] > dis-distance_margin):
        left -= 1
    while(right < len(ranges) and ranges[right] > dis-distance_margin):
        right += 1
    
    mid = (left+right)//2
    best_angle = angle_min + mid * angle_increment
    best_dist = ranges[mid]
    angle_deg = 180.0*best_angle / math.pi

    return angle_deg, best_dist  # return farthest distance

def pure_pursuit(odom):
    # Obtain the current position of the race car from the inferred_pose message
    odom_x = odom.pose.position.x
    odom_y = odom.pose.position.y


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
    heading = tf.transformations.euler_from_quaternion((odom.pose.orientation.x,
                                                        odom.pose.orientation.y,
                                                        odom.pose.orientation.z,
                                                        odom.pose.orientation.w))[2]

    # so this code just follows the polyline for lookahead_distance units (meters)
    target_point = [i for i in closest_point]
    current_idx = left_point_idx

    dist_from_prev = math.sqrt((plan[current_idx][0]-closest_point[0])**2 + (plan[current_idx][1]-closest_point[1])**2)
    # some cheese because first point isn't on a point
    lookahead_distance_cpy = LOOKAHEAD + dist_from_prev

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

    # protect against division by zero for very small alpha
    sin_alpha = math.sin(alpha)
    if abs(sin_alpha) < 1e-6:
        rotation_radius = float('inf')
    else:
        rotation_radius = LOOKAHEAD/(2.0*sin_alpha)

    if rotation_radius == float('inf'):
        delta = 0.0
    else:
        delta = math.atan(WHEELBASE_LEN/rotation_radius)

    # steering in degrees
    delta_deg = 180.0 * delta / math.pi
    # lookahead angle in degrees (useful for querying lidar)
    alpha_deg = 180.0 * alpha / math.pi
    dynamic_speed = MIN_SPEED + ((MAX_SPEED-MIN_SPEED)/2)*(math.sin(alpha + math.pi/2.0)+1)

    # return steering (deg), speed, and lookahead angle (deg)
    return delta_deg, dynamic_speed, alpha_deg

def control_node(data):
    global prev_steering, prev_speed

    pp_delta_deg, pp_speed, pp_alpha_deg = pure_pursuit(data)
    disparity_angle_deg, best_dist = disparity_extender()

    # Query LIDAR at the lookahead direction (pp_alpha_deg)
    pp_dist = get_dist(pp_alpha_deg)

    # Decide whether to overtake based on scan distances
    # Conservative overtaking: require BOTH a close obstacle on the planned path
    # and a sufficiently large nearby gap before committing to an overtake.
    if (pp_dist < OVERTAKE_PP_DIST_MAX) and ((best_dist - pp_dist) > OVERTAKE_GAP_MIN):
        # use follow-the-gap steering (disparity_angle is already degrees)
        a_deg = disparity_angle_deg
        # scale down steering to be less aggressive during overtakes
        clipped_steering_angle = max(-100.0, min(100.0, OVERTAKE_STEER_SCALE * a_deg))
        # apply a more cautious target speed during overtakes
        s = min(((MAX_SPEED-MIN_SPEED)/2)*(math.sin((math.pi*clipped_steering_angle)/100.0 + math.pi/2.0)+1) + MIN_SPEED,
                MAX_SPEED * OVERTAKE_SPEED_SCALE)
        rospy.loginfo('Control: Overtake branch selected pp_dist=%.2f best_dist=%.2f disp_deg=%.2f', pp_dist, best_dist, a_deg)
    else:
        # follow pure pursuit steering
        s = pp_speed
        a_deg = pp_delta_deg
        clipped_steering_angle = max(-100.0, min(100.0, 5.0 * a_deg))
        rospy.loginfo('Control: PurePursuit branch pp_alpha=%.2f deg delta_deg=%.2f', pp_alpha_deg, a_deg)

    # Apply exponential smoothing (EMA) to reduce high-frequency oscillations
    desired_steer = clipped_steering_angle
    desired_speed = s

    smoothed_steer = prev_steering + STEER_SMOOTH_ALPHA * (desired_steer - prev_steering)
    # limit absolute step per update to avoid large quick changes
    steer_delta = smoothed_steer - prev_steering
    if steer_delta > MAX_STEER_STEP:
        steer_delta = MAX_STEER_STEP
    elif steer_delta < -MAX_STEER_STEP:
        steer_delta = -MAX_STEER_STEP
    final_steer = prev_steering + steer_delta

    smoothed_speed = prev_speed + SPEED_SMOOTH_ALPHA * (desired_speed - prev_speed)
    speed_delta = smoothed_speed - prev_speed
    if speed_delta > MAX_SPEED_STEP:
        speed_delta = MAX_SPEED_STEP
    elif speed_delta < -MAX_SPEED_STEP:
        speed_delta = -MAX_SPEED_STEP
    final_speed = prev_speed + speed_delta

    command = AckermannDrive()
    command.speed = final_speed
    command.steering_angle = final_steer

    # update previous state for next smoothing step
    prev_steering = final_steer
    prev_speed = final_speed

    command_pub.publish(command)

if __name__ == '__main__':

    try:
        rospy.init_node('pure_pursuit', anonymous = True)
        # allow tuning of overtaking behavior via ROS params
        global OVERTAKE_GAP_MIN, OVERTAKE_PP_DIST_MAX, OVERTAKE_STEER_SCALE, OVERTAKE_SPEED_SCALE
        OVERTAKE_GAP_MIN = rospy.get_param('~overtake_gap_min', OVERTAKE_GAP_MIN)
        OVERTAKE_PP_DIST_MAX = rospy.get_param('~overtake_pp_dist_max', OVERTAKE_PP_DIST_MAX)
        OVERTAKE_STEER_SCALE = rospy.get_param('~overtake_steer_scale', OVERTAKE_STEER_SCALE)
        OVERTAKE_SPEED_SCALE = rospy.get_param('~overtake_speed_scale', OVERTAKE_SPEED_SCALE)
        if not plan:
            rospy.loginfo('obtaining trajectory')
            construct_path()

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, control_node)
        rospy.Subscriber("/{}/scan".format(car_name), LaserScan, update_laserscan)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass