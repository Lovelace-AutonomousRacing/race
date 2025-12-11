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
TOTAL_LINES = 5
plans                = []
path_resolutions     = []
frame_id            = 'map'
car_name            = str(sys.argv[1])
trajectory_name     = str(sys.argv[2])

# Publishers for sending driving commands and visualizing the control polygon
command_pub         = rospy.Publisher('/{}/offboard/command'.format(car_name), AckermannDrive, queue_size = 1)
raceline_pubs = []
for i in range(TOTAL_LINES):
    raceline_pubs.append(rospy.Publisher('/raceline' + str(i), Path, queue_size=1, latch=True))

# Global variables for waypoint sequence and current polygon
global LAST_SCAN
LAST_SCAN = LaserScan()

# Tunable parameters
MAX_SPEED = 60
MIN_SPEED = 20
LOOKAHEAD = 2.0
THRESHOLD = 0.15		# updated threshold
CAR_TOLERANCE = 0.20 # increased safety margin
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
CAR_WIDTH = 0.30  # increased car width

def construct_paths():
    # Function to construct the path from a CSV file
    # TODO: Modify this path to match the folder where the csv file containing the path is located.
    for i in range(5):
        plan = []
        path_resolution = []
        new_name = trajectory_name + str(i)
        file_path = os.path.expanduser('/home/nvidia/catkin_ws/src/f1tenth_purepursuit/path/{}.csv'.format(new_name))
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

        raceline_pubs[i].publish(raceline_path)

        plans.append(plan)
        path_resolutions.append(path_resolution)


# Steering Range from -100.0 to 100.0
STEERING_RANGE = 100.0

# vehicle physical parameters
WHEELBASE_LEN       = 0.325

def update_laserscan(scan):
    global LAST_SCAN
    LAST_SCAN = scan

def get_dist(angle, ranges, angle_min):
	angle_rad = math.radians(angle)
	index = int ((angle_rad - angle_min)/LAST_SCAN.angle_increment)
	index = max(0,min(index,len(ranges)-1))
	distance = ranges[index]
	if math.isinf(distance) or math.isnan(distance):
		distance = LAST_SCAN.range_max
	return distance


def min_dist_ahead(window_deg=20.0):
    """Return the minimum valid range within +/- window_deg/2 around 0 degrees (vehicle forward).
    If scan is not available, returns a large value (LAST_SCAN.range_max).
    """
    if not LAST_SCAN.ranges:
        return float('inf')

    half = math.radians(window_deg) / 2.0
    angle_min = LAST_SCAN.angle_min
    angle_inc = LAST_SCAN.angle_increment
    start_idx = int(( -half - angle_min) / angle_inc)
    end_idx = int(( half - angle_min) / angle_inc)
    start_idx = max(0, start_idx)
    end_idx = min(len(LAST_SCAN.ranges)-1, end_idx)
    minv = float('inf')
    for i in range(start_idx, end_idx+1):
        d = LAST_SCAN.ranges[i]
        if math.isinf(d) or math.isnan(d):
            continue
        if d < minv:
            minv = d
    if minv == float('inf'):
        return LAST_SCAN.range_max
    return minv

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
    return ranges, angle_min

def pure_pursuit(odom, plan, path_resolution):
    # Obtain the current position of the race car from the inferred_pose message
    odom_x = odom.pose.position.x
    odom_y = odom.pose.position.y
    # Calculate heading angle of the car (in radians)
    # roll pitch yaw euler
    heading = tf.transformations.euler_from_quaternion((odom.pose.orientation.x,
                                                        odom.pose.orientation.y,
                                                        odom.pose.orientation.z,
                                                        odom.pose.orientation.w))[2]
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

    return alpha

def control_node(data):
    modified_ranges, new_min = disparity_extender()
    pp_alphas = []
    for i in range(len(plans)):
        pp_alphas.append(pure_pursuit(data, plans[i], path_resolutions[i]))

    mx_dist = 0
    best_line = 0
    steering_angle = 0.0
    
    for i, alpha in enumerate(pp_alphas):
        rotation_radius = LOOKAHEAD/(2.0*math.sin(alpha))
        delta = math.atan(WHEELBASE_LEN/rotation_radius)
        delta = max(-0.4, min(0.4, delta))
        dist = get_dist(delta, modified_ranges, new_min)
        if dist > mx_dist:
            mx_dist = dist
            best_line = i
            steering_angle = (delta/0.4)*100.0


    # TODO 5: Ensure that the calculated steering angle is within the STEERING_RANGE and assign it to command.steering_angle
    # Your code here
    command = AckermannDrive()
    if mx_dist < 0.5: # if a line provides overtake, take it
        command.speed = MIN_SPEED
    else:
        dynamic_speed = MIN_SPEED + ((MAX_SPEED-MIN_SPEED)/2)*(math.sin(1.6*pp_alphas[best_line] + math.pi/2.0)+1) #fn of alpha where f(backwards) = min_speed
        command.speed = dynamic_speed
    command.steering_angle = steering_angle

    command_pub.publish(command)

if __name__ == '__main__':

    try:
        rospy.init_node('pure_pursuit', anonymous = True)
        if not plans:
            rospy.loginfo('obtaining trajectory')
            construct_paths()

        # This node subsribes to the pose estimate provided by the Particle Filter. 
        # The message type of that pose message is PoseStamped which belongs to the geometry_msgs ROS package.
        rospy.Subscriber('/{}/particle_filter/viz/inferred_pose'.format(car_name), PoseStamped, control_node)
        rospy.Subscriber("/{}/scan".format(car_name), LaserScan, update_laserscan)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass