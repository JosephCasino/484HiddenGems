#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
import numpy as np

# Globals
current_speed = 0.0
current_angle = 0.0
logged_this_stop = False
drive_pub = None
mode = "FORWARD"
backup_start_time = None
backup_distance_limit = 0.0
odometry_history = []
MAX_ODOM_HISTORY = 50

# Tunable Parameters
BASE_SPEED = 0.4
TURN_SPEED = 0.2
BACKUP_SPEED = -0.3
backup_duration = 0.4
emergency_distance = 0.3
safe_forward_distance = 2.0
backup_safe_distance = 1.0
side_clearance_threshold = 1.2
small_correction = 0.12

def command_callback(msg):
    global current_speed, current_angle
    current_speed = msg.speed
    current_angle = msg.steering_angle

def get_sector_min(ranges, center_idx, width):
    start = max(0, center_idx - width)
    end = min(len(ranges), center_idx + width)
    sector = ranges[start:end]
    return np.min(sector)

def odom_callback(msg):
    global odometry_history
    pos = msg.pose.pose.position
    current = np.array([pos.x, pos.y])
    odometry_history.append(current)
    if len(odometry_history) > MAX_ODOM_HISTORY:
        odometry_history.pop(0)

def get_recent_distance_moved():
    if len(odometry_history) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(odometry_history)):
        total += np.linalg.norm(odometry_history[i] - odometry_history[i - 1])
    return total

def scan_callback(msg):
    global drive_pub, mode, backup_start_time, backup_distance_limit

    ranges = np.array(msg.ranges)
    ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0)
    ranges = np.clip(ranges, 0, 10)

    num_ranges = len(ranges)
    front_idx = num_ranges // 2
    left_idx = int(num_ranges * 3 / 4)
    right_idx = int(num_ranges * 1 / 4)

    front_distance = get_sector_min(ranges, front_idx, 10)
    left_distance = get_sector_min(ranges, left_idx, 10)
    right_distance = get_sector_min(ranges, right_idx, 10)

    rear_indices = list(range(0, 10)) + list(range(num_ranges - 10, num_ranges))
    rear_distance = np.mean([ranges[i] for i in rear_indices])

    drive_msg = AckermannDrive()

    # Trap Check
    if front_distance < safe_forward_distance and rear_distance < backup_safe_distance:
        rospy.logerr("TRAPPED! Front and Rear blocked. Emergency STOP.")
        drive_msg.speed = 0.0
        drive_msg.steering_angle = 0.0
        drive_pub.publish(drive_msg)
        return

    # Backup Mode
    if mode == "BACKUP":
        elapsed = rospy.get_time() - backup_start_time
        moved = get_recent_distance_moved()

        rospy.loginfo("Backup status: moved=%.2f, limit=%.2f, rear=%.2f", moved, backup_distance_limit, rear_distance)

        if rear_distance < backup_safe_distance or moved >= backup_distance_limit:
            if front_distance > safe_forward_distance:
                mode = "FORWARD"
                rospy.loginfo("Backup done. Switching to FORWARD.")
            else:
                mode = "BACKUP"
                backup_start_time = rospy.get_time()
                rospy.loginfo("Retrying backup, still blocked ahead.")

            drive_msg.speed = 0.0
            drive_msg.steering_angle = 0.0
            drive_pub.publish(drive_msg)
            return
        else:
            drive_msg.speed = BACKUP_SPEED
            drive_msg.steering_angle = -0.3 if left_distance > right_distance else 0.3
            rospy.loginfo("Backing up (%.2fm rear clearance)", rear_distance)
            drive_pub.publish(drive_msg)
            return

    # Forward Driving
    if front_distance < emergency_distance:
        mode = "BACKUP"
        backup_start_time = rospy.get_time()
        backup_distance_limit = max(get_recent_distance_moved(), 0.2)  # <- FIXED HERE
        rospy.logwarn("Too close! Starting backup. Will reverse %.2f m", backup_distance_limit)
        drive_msg.speed = 0.0
        drive_msg.steering_angle = 0.0
        drive_pub.publish(drive_msg)
        return

    elif front_distance < safe_forward_distance:
        drive_msg.speed = TURN_SPEED
        drive_msg.steering_angle = 0.5 if left_distance > right_distance else -0.5
        rospy.loginfo("Turning %s (front %.2fm)", "LEFT" if drive_msg.steering_angle > 0 else "RIGHT", front_distance)

    else:
        drive_msg.speed = BASE_SPEED
        correction = 0.0
        if left_distance < side_clearance_threshold:
            correction -= small_correction
            rospy.loginfo("Adjusting RIGHT: Left wall %.2fm", left_distance)
        if right_distance < side_clearance_threshold:
            correction += small_correction
            rospy.loginfo("Adjusting LEFT: Right wall %.2fm", right_distance)
        drive_msg.steering_angle = correction
        if correction == 0.0:
            rospy.loginfo("Driving straight (front %.2fm)", front_distance)

    drive_pub.publish(drive_msg)

def main():
    global drive_pub
    rospy.init_node('my_laser_listener')
    drive_pub = rospy.Publisher('/car_1/multiplexer/command', AckermannDrive, queue_size=10)
    rospy.Subscriber('/car_1/scan', LaserScan, scan_callback)
    rospy.Subscriber('/car_1/multiplexer/command', AckermannDrive, command_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
