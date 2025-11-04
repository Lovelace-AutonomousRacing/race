#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FTG DIST FINDER NODE (Python 2.7)
---------------------------------
This ROS node:
 - Subscribes to /car_5/scan (LaserScan)
 - Applies Follow-The-Gap algorithm:
      1. Lidar preprocessing (disparity detection & extension)
      2. Obstacle "bubble" clearing around the closest object
      3. Finding the largest navigable gap (-90° to +90°)
      4. Selecting a target point inside that gap
 - Publishes ONLY the target angle (steering direction) and distance
   using race.msg.pid_input message:
       msg.pid_error = target_angle (radians)
       msg.pid_vel   = target_distance (meters)
Control logic (steering + speed) happens in ftg_controller.py
"""

from __future__ import print_function  # Python 2/3 print compatibility
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# ====================== PARAMETERS (TUNE THESE) ============================
DISPARITY_THRESHOLD = 0.2       # Difference (m) to declare a disparity between consecutive rays
CAR_WIDTH = 0.35                # Approximate vehicle width in meters
CAR_HALF_WIDTH = CAR_WIDTH / 2.0
CAR_TOLERANCE = 0.05            # Extra buffer (m) when "extending" disparities
BUBBLE_RADIUS = 0.35            # Safety radius (m) to clear around closest obstacle
MAX_LOOK_ANGLE = math.pi / 2.0  # Restrict target search to ±90 degrees
CHOICE = "center"               # Choose "center" of gap or "deepest" point

# Publishes target info on topic 'error'
pub = rospy.Publisher('error', pid_input, queue_size=1)
# Publisher for the virtual (processed) scan visualization in RViz
virtual_scan_pub = rospy.Publisher('/car_5/virtual_scan', LaserScan, queue_size=1)


# ====================== HELPER FUNCTIONS (UNCHANGED LOGIC) ============================

def angle_of_index(scan, idx):
    """Convert array index -> actual lidar angle (radians)."""
    return scan.angle_min + idx * scan.angle_increment

def index_of_angle(scan, angle_rad):
    """Convert angle in radians -> nearest index in lidar array."""
    return int(round((angle_rad - scan.angle_min) / scan.angle_increment))

def safe_asin_arg(x):
    """Clamp x to [-1,1] to avoid invalid asin calls."""
    return max(-1.0, min(1.0, x))

def extend_disparities(scan_ranges, scan):
    """
    Detect and 'extend' disparities:
    - If difference between adjacent points > DISPARITY_THRESHOLD,
      treat closer point as obstacle edge and extend it sideways
      for CAR_HALF_WIDTH + tolerance distance.
    """
    virtual = list(scan_ranges)  # copy
    n = len(virtual)

    # Replace any invalid readings with range_max
    for i in range(n):
        r = virtual[i]
        if r is None or math.isinf(r) or math.isnan(r):
            virtual[i] = scan.range_max

    for i in range(n - 1):
        d1, d2 = virtual[i], virtual[i + 1]

        # A disparity occurs
        if abs(d1 - d2) > DISPARITY_THRESHOLD:
            # Identify which index is closer
            if d1 < d2:
                closer_idx, farther_idx = i, i + 1
            else:
                closer_idx, farther_idx = i + 1, i

            d_closer = virtual[closer_idx]
            if d_closer <= 0:
                continue#skips invalid values, continues for loop

            # How many lidar samples correspond to half car width at d_closer?/////////////////////// double check
            arg = safe_asin_arg((CAR_HALF_WIDTH + CAR_TOLERANCE) / d_closer)
            ang_half = math.asin(arg)
            samples = int(math.ceil(ang_half / scan.angle_increment))

            # Extend from the 'farther_idx' outward
            step = 1 if (farther_idx > closer_idx) else -1
            idx2 = farther_idx
            for _ in range(samples):
                if 0 <= idx2 < n:
                    if virtual[idx2] > d_closer:
                        virtual[idx2] = d_closer
                idx2 += step

    return virtual

def set_nearest_bubble(virtual, scan):
    """
    Create a 'safety bubble' around the closest obstacle.
    All lidar points within BUBBLE_RADIUS of that obstacle are zeroed out
    to discourage driving too close.
    """
    virtual = list(virtual)
    nearest_idx = int(np.argmin(virtual))
    nearest_dist = virtual[nearest_idx]

    # Ignore infinite or invalid distance
    if (nearest_dist <= 0) or math.isinf(nearest_dist) or math.isnan(nearest_dist):
        return virtual

    # Determine how many indices correspond to BUBBLE_RADIUS at that distance
    arg = safe_asin_arg(BUBBLE_RADIUS / nearest_dist)
    ang_half = math.asin(arg)
    samples = int(math.ceil(ang_half / scan.angle_increment))

    for i in range(nearest_idx - samples, nearest_idx + samples + 1):
        if 0 <= i < len(virtual):
            virtual[i] = 0.0  # mark as non-drivable

    return virtual

def find_max_gap(virtual, scan):
    """
    Find largest contiguous region of non-zero values (safe space)
    within forward ±90 degrees.
    Returns (start_index, end_index) or None if no gap.
    """
    n = len(virtual)

    # Only search within front field of view
    i_min = max(0, index_of_angle(scan, -MAX_LOOK_ANGLE))
    i_max = min(n - 1, index_of_angle(scan, MAX_LOOK_ANGLE))

    best = None
    max_len = 0
    start = None

    for i in range(i_min, i_max + 1):
        if virtual[i] > 0:
            if start is None:
                start = i
        else:
            if start is not None:
                length = i - start
                if length > max_len:
                    max_len = length
                    best = (start, i - 1)
                start = None

    # Check if gap continues to array end
    if start is not None:
        length = (i_max + 1) - start
        if length > max_len:
            best = (start, i_max)

    return best

def choose_target_in_gap(virtual, scan, start_idx, end_idx):
    """
    Select actual target point inside the found gap:
     - Either the center index, or
     - The greatest depth (farthest distance) inside the gap
    Returns (index, angle, distance).
    """
    if start_idx is None:
        return None

    if CHOICE == "center":
        target_idx = (start_idx + end_idx) // 2
    else:
        sub = virtual[start_idx:end_idx + 1]
        rel = int(np.argmax(sub))
        target_idx = start_idx + rel

    return target_idx, angle_of_index(scan, target_idx), virtual[target_idx]

# ============================= CALLBACK =============================

def callback(scan):
    """
    1. Read lidar
    2. Process to find virtual scan after disparity and bubble
    3. Find max gap, choose target
    4. Publish target angle + distance
    """
    # Clean data (replace NaN/inf with max range)
    ranges = []
    for r in scan.ranges:
        if r is None or math.isinf(r) or math.isnan(r):
            ranges.append(scan.range_max)
        else:
            ranges.append(r)

    virtual = extend_disparities(ranges, scan)
    virtual = set_nearest_bubble(virtual, scan)

    # --- PUBLISH VIRTUAL SCAN FOR RVIZ VISUALIZATION ---
    virtual_msg = LaserScan()
    virtual_msg.header = scan.header
    virtual_msg.angle_min = scan.angle_min
    virtual_msg.angle_max = scan.angle_max
    virtual_msg.angle_increment = scan.angle_increment
    virtual_msg.time_increment = scan.time_increment
    virtual_msg.scan_time = scan.scan_time
    virtual_msg.range_min = scan.range_min
    virtual_msg.range_max = scan.range_max
    virtual_msg.ranges = virtual  # use your modified list of distances
    virtual_scan_pub.publish(virtual_msg)


    gap = find_max_gap(virtual, scan)
    if gap is None:
        # No safe gap: stop or send zeros
        msg = pid_input()
        msg.pid_error = 0
        msg.pid_vel = 0
        pub.publish(msg)
        return

    start_idx, end_idx = gap
    target = choose_target_in_gap(virtual, scan, start_idx, end_idx)

    if target is None:
        return

    _, target_angle, target_dist = target

    # Send as pid_input message (repurposed):
    # pid_error = angle (radians), pid_vel = distance
    msg = pid_input()
    msg.pid_error = target_angle
    msg.pid_vel = target_dist
    pub.publish(msg)

# ============================= MAIN =============================

if __name__ == "__main__":
    rospy.init_node("ftg_dist_finder")
    rospy.Subscriber("/car_5/scan", LaserScan, callback, queue_size=1)
    rospy.loginfo("FTG dist_finder running (Python 2.7)")
    rospy.spin()
