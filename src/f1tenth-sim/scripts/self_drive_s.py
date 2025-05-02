#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import numpy as np
import random

# Globals
current_speed = 0.0
current_angle = 0.0
drive_pub = None
mode = "FORWARD"  # Modes: FORWARD, BACKUP
backup_start_time = None
backup_duration = 0.4  # seconds to back up

# Exploration variables
straight_drive_count = 0
min_straight_count = 20
exploration_turn_active = False
exploration_turn_counter = 0
exploration_turn_duration = 15  # Number of iterations to maintain an exploration turn

# Tunable Speeds
BASE_SPEED = 0.4   # normal forward speed
TURN_SPEED = 0.2   # slower while turning
BACKUP_SPEED = -0.3  # backing up speed

# Debug flag
DEBUG = True

def log_debug(msg):
    if DEBUG:
        rospy.loginfo(msg)

def command_callback(msg):
    global current_speed, current_angle
    current_speed = msg.speed
    current_angle = msg.steering_angle

def scan_callback(msg):
    global drive_pub, mode, backup_start_time, straight_drive_count
    global exploration_turn_active, exploration_turn_counter

    # Handle 270-degree LiDAR data correctly
    ranges = np.array(msg.ranges)
    ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0)
    ranges = np.clip(ranges, 0, 10)
    
    # Calculate angular range of the LiDAR - handle both 270 and 360 degree configurations
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_range = angle_max - angle_min
    
    # Detect LiDAR configuration
    is_270_lidar = (angle_range < 6.0)  # Less than ~345 degrees
    
    if is_270_lidar:
        log_debug("270-degree LiDAR detected")
    else:
        log_debug("Full 360-degree LiDAR detected")
    
    # Get number of points in scan
    num_ranges = len(ranges)
    
    # For 270-degree LiDAR, the rear indices may not be available
    # We'll use what we have, but be aware of limitations
    
    # Thresholds
    emergency_distance = 0.3
    safe_forward_distance = 2.0
    backup_safe_distance = 1.0
    side_clearance_threshold = 1.2  # Wall side clearance distance
    small_correction = 0.12  # radians (gentle steering)

    # Key indices - adjusted for 270-degree LiDAR
    front_idx = num_ranges // 2
    left_idx = int(num_ranges * 3 / 4)
    right_idx = int(num_ranges * 1 / 4)
    
    # For rear readings in 270-degree LiDAR, we might not have direct measurements
    # Use the available data at the edges, but be cautious
    if is_270_lidar:
        # For 270-degree LiDAR, the rear is partially or completely blind
        # We'll use the endpoints (which are close to but not exactly at the rear)
        rear_indices = list(range(0, min(5, num_ranges//10))) + list(range(max(num_ranges - num_ranges//10, num_ranges-5), num_ranges))
    else:
        # For 360-degree LiDAR, we can use the true rear
        rear_indices = list(range(0, 10)) + list(range(num_ranges-10, num_ranges))
    
    # Get average distances for key sectors
    rear_distances = [ranges[i] for i in rear_indices]
    rear_distance = np.mean(rear_distances) if rear_distances else 1.0  # Default if no data
    
    front_distance = ranges[front_idx]
    left_distance = ranges[left_idx]
    right_distance = ranges[right_idx]
    
    # Forward arc - 60 degree span in front
    front_arc_start = max(0, front_idx - num_ranges // 12)
    front_arc_end = min(num_ranges - 1, front_idx + num_ranges // 12)
    front_arc = ranges[front_arc_start:front_arc_end]
    front_min_distance = np.min(front_arc) if len(front_arc) > 0 else front_distance
    
    # Log key distances
    log_debug(f"Distances - Front: {front_distance:.2f}, Left: {left_distance:.2f}, "
             f"Right: {right_distance:.2f}, Rear: {rear_distance:.2f}")

    drive_msg = AckermannDrive()

    # Emergency TRAP Check: Only check if we have reliable rear data
    if not is_270_lidar and front_distance < safe_forward_distance and rear_distance < backup_safe_distance:
        rospy.logerr("TRAPPED! Front and Rear both blocked. Emergency STOP.")
        drive_msg.speed = 0.0
        drive_msg.steering_angle = 0.0
        drive_pub.publish(drive_msg)
        return

    # Backup Mode
    if mode == "BACKUP":
        elapsed = rospy.get_time() - backup_start_time
        
        # In 270-degree LiDAR, we might not have accurate rear data,
        # so rely more on the timer for backup completion
        backup_complete = False
        
        if is_270_lidar:
            # For 270-degree, rely primarily on timer
            backup_complete = elapsed > backup_duration
        else:
            # For 360-degree, use both time and sensor data
            backup_complete = rear_distance < backup_safe_distance or elapsed > backup_duration
        
        if backup_complete:
            if front_distance > safe_forward_distance:
                mode = "FORWARD"
                straight_drive_count = 0
                exploration_turn_active = False
                log_debug("Backup done, front clear. Switching to FORWARD mode.")
            else:
                mode = "BACKUP"
                backup_start_time = rospy.get_time()
                log_debug("Backup done, but front still blocked. Backing up again!")

            drive_msg.speed = 0.0
            drive_msg.steering_angle = 0.0
            drive_pub.publish(drive_msg)
            return
        else:
            drive_msg.speed = BACKUP_SPEED
            # Alternate backup directions slightly to help escape tight spots
            if backup_start_time % 2 == 0:
                drive_msg.steering_angle = 0.3
            else:
                drive_msg.steering_angle = -0.3
                
            log_debug(f"Backing up... elapsed: {elapsed:.1f}s")
            drive_pub.publish(drive_msg)
            return

    # Normal FORWARD driving with exploration
    if front_min_distance < emergency_distance:
        mode = "BACKUP"
        backup_start_time = rospy.get_time()
        straight_drive_count = 0
        exploration_turn_active = False
        rospy.logwarn(f"Emergency! Too close ahead ({front_min_distance:.2f}m). Starting backup.")
        drive_msg.speed = 0.0
        drive_msg.steering_angle = 0.0
        drive_pub.publish(drive_msg)
        return

    elif front_min_distance < safe_forward_distance:
        drive_msg.speed = TURN_SPEED
        straight_drive_count = 0
        exploration_turn_active = False
        
        # Choose the direction with more space
        if left_distance > right_distance:
            drive_msg.steering_angle = 0.5
            log_debug(f"Turning LEFT (front {front_min_distance:.2f}m)")
        else:
            drive_msg.steering_angle = -0.5
            log_debug(f"Turning RIGHT (front {front_min_distance:.2f}m)")
            
    else:
        # Normal driving with exploration capabilities
        drive_msg.speed = BASE_SPEED
        steering_correction = 0.0
        
        # Handle active exploration turns
        if exploration_turn_active:
            exploration_turn_counter += 1
            if exploration_turn_counter >= exploration_turn_duration:
                # End the exploration turn
                exploration_turn_active = False
                exploration_turn_counter = 0
                log_debug("Exploration turn complete")
            else:
                # Continue the turn
                if exploration_turn_counter % 2 == 0:  # Left turn
                    steering_correction = 0.3
                    log_debug(f"Continuing LEFT exploration turn ({exploration_turn_counter}/{exploration_turn_duration})")
                else:  # Right turn
                    steering_correction = -0.3
                    log_debug(f"Continuing RIGHT exploration turn ({exploration_turn_counter}/{exploration_turn_duration})")
        
        # Not in exploration turn - check if we should start one
        elif straight_drive_count >= min_straight_count:
            # 10% chance to start exploration once eligible
            if random.random() < 0.1:
                exploration_turn_active = True
                exploration_turn_counter = 0
                
                # Choose direction based on space - prefer side with more room
                if left_distance > right_distance * 1.5:
                    # Much more space left
                    steering_correction = 0.3
                    log_debug("Starting LEFT exploration turn (more space left)")
                elif right_distance > left_distance * 1.5:
                    # Much more space right
                    steering_correction = -0.3
                    log_debug("Starting RIGHT exploration turn (more space right)")
                else:
                    # Similar space - random choice
                    if random.choice([True, False]):
                        steering_correction = 0.3
                        log_debug("Starting LEFT exploration turn (random choice)")
                    else:
                        steering_correction = -0.3
                        log_debug("Starting RIGHT exploration turn (random choice)")
                
                straight_drive_count = 0
        
        # Standard wall avoidance (even during exploration)
        if left_distance < side_clearance_threshold:
            steering_correction -= small_correction  # steer slightly right
            log_debug(f"Adjusting RIGHT: Left wall too close ({left_distance:.2f}m)")
            straight_drive_count = 0  # Reset when avoiding walls
            
        if right_distance < side_clearance_threshold:
            steering_correction += small_correction  # steer slightly left
            log_debug(f"Adjusting LEFT: Right wall too close ({right_distance:.2f}m)")
            straight_drive_count = 0  # Reset when avoiding walls
            
        # If no steering correction is applied, we're going straight
        if abs(steering_correction) < 0.01:
            straight_drive_count += 1
            log_debug(f"Driving straight (front {front_distance:.2f}m, count: {straight_drive_count})")
        
        # Final steering angle
        drive_msg.steering_angle = steering_correction

    drive_pub.publish(drive_msg)

def main():
    global drive_pub

    rospy.init_node('lidar270_explorer')

    drive_pub = rospy.Publisher('/car_1/multiplexer/command', AckermannDrive, queue_size=10)

    rospy.Subscriber('/car_1/scan', LaserScan, scan_callback)
    rospy.Subscriber('/car_1/multiplexer/command', AckermannDrive, command_callback)
    
    rospy.loginfo("270-degree LiDAR Explorer initialized!")

    rospy.spin()

if __name__ == '__main__':
    main()