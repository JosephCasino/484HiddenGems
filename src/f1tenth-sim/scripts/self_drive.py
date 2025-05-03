#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import numpy as np

# Globals
current_speed = 0.0
current_angle = 0.0
logged_this_stop = False
drive_pub = None
mode = "FORWARD" #Initialize car to move forward (Also a BACKUP mode)
backup_start_time = None
backup_duration = 0.4 #secons to reverse before reevaluationg

# Tunable Speeds
BASE_SPEED = 0.4    #default forward speed
TURN_SPEED = 0.2    #speed when turning
BACKUP_SPEED = -0.3 #speed when reversing

def command_callback(msg):
    global current_speed, current_angle
    current_speed = msg.speed
    current_angle = msg.steering_angle


#gets closest thing in the LiDAR range (looks at a slice of readings)
def get_sector_min(ranges, center_idx, width):
    start = max(0, center_idx - width)
    end = min(len(ranges), center_idx + width)
    sector = ranges[start:end]
    return np.min(sector)

def scan_callback(msg):
    global drive_pub, mode, backup_start_time

    # Convert LiDAR ranges to clean array
    ranges = np.array(msg.ranges)
    ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0)
    ranges = np.clip(ranges, 0, 10)

    # Thresholds
    emergency_distance = 0.3
    safe_forward_distance = 2.0
    backup_safe_distance = 1.0
    side_clearance_threshold = 1.2
    small_correction = 0.12


    # Calculates front, left, right angles 
    num_ranges = len(ranges)
    front_idx = num_ranges // 2
    left_idx = int(num_ranges * 3 / 4)
    right_idx = int(num_ranges * 1 / 4)

    # Use range of readings for each direction (change width variable for bigger or smaller range)
    front_distance = get_sector_min(ranges, front_idx, 10)
    left_distance = get_sector_min(ranges, left_idx, 10)
    right_distance = get_sector_min(ranges, right_idx, 10)

    # Rear: average of beginning + end slices
    rear_indices = list(range(0, 10)) + list(range(num_ranges-10, num_ranges))
    rear_distances = [ranges[i] for i in rear_indices]
    rear_distance = np.mean(rear_distances)

    drive_msg = AckermannDrive()

    # Emergency TRAP Check
    if front_distance < safe_forward_distance and rear_distance < backup_safe_distance:
        rospy.logerr("TRAPPED! Front and Rear both blocked. Emergency STOP.")
        drive_msg.speed = 0.0
        drive_msg.steering_angle = 0.0
        drive_pub.publish(drive_msg)
        return

    # Backup Mode
    if mode == "BACKUP":
        elapsed = rospy.get_time() - backup_start_time
        if rear_distance < backup_safe_distance or elapsed > backup_duration:
            if front_distance > safe_forward_distance:
                mode = "FORWARD"
                rospy.loginfo("Backup done, front clear. Switching to FORWARD mode.")
            else:
                mode = "BACKUP"
                backup_start_time = rospy.get_time()
                rospy.loginfo("Backup done, but front still blocked. Backing up again!")

            drive_msg.speed = 0.0
            drive_msg.steering_angle = 0.0
            drive_pub.publish(drive_msg)
            return
        else:
            drive_msg.speed = BACKUP_SPEED
            drive_msg.steering_angle = -0.3 if left_distance > right_distance else 0.3
            rospy.loginfo("Backing up... rear clearance: %.2f m", rear_distance)
            drive_pub.publish(drive_msg)
            return

    # Normal FORWARD driving
    if front_distance < emergency_distance:
        mode = "BACKUP"
        backup_start_time = rospy.get_time()
        rospy.logwarn("Emergency! Too close ahead (%.2fm). Starting backup.", front_distance)
        drive_msg.speed = 0.0
        drive_msg.steering_angle = 0.0
        drive_pub.publish(drive_msg)
        return

    elif front_distance < safe_forward_distance:
        drive_msg.speed = TURN_SPEED
        if left_distance > right_distance:
            drive_msg.steering_angle = 0.5
            rospy.loginfo("Turning LEFT (front %.2fm)", front_distance)
        else:
            drive_msg.steering_angle = -0.5
            rospy.loginfo("Turning RIGHT (front %.2fm)", front_distance)

    else:
        # Wall-following adjustment
        drive_msg.speed = BASE_SPEED
        steering_correction = 0.0

        if left_distance < side_clearance_threshold:
            steering_correction -= small_correction
            rospy.loginfo("Adjusting RIGHT: Left wall too close (%.2fm)", left_distance)

        if right_distance < side_clearance_threshold:
            steering_correction += small_correction
            rospy.loginfo("Adjusting LEFT: Right wall too close (%.2fm)", right_distance)

        drive_msg.steering_angle = steering_correction
        if steering_correction == 0.0:
            rospy.loginfo("Driving straight (front %.2fm)", front_distance)

    drive_pub.publish(drive_msg)

def main():
    global drive_pub
    rospy.init_node('my_laser_listener')
    drive_pub = rospy.Publisher('/car_1/multiplexer/command', AckermannDrive, queue_size=10)
    rospy.Subscriber('/car_1/scan', LaserScan, scan_callback)
    rospy.Subscriber('/car_1/multiplexer/command', AckermannDrive, command_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
