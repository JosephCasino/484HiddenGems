#!/usr/bin/env python3

#This is the code used in the f1-Tenth Simulator to read the closest obstacle sensed by the LiDAR. 

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import math

current_speed = 0.0
current_angle = 0.0
logged_this_stop = False

def command_callback(msg):
    global current_speed, current_angle
    current_speed = msg.speed
    current_angle = msg.steering_angle

def scan_callback(msg):
    global current_speed, current_angle, logged_this_stop

    if current_speed == 0.0 and current_angle == 0.0 and not logged_this_stop:
        # Find the closest range and its index
        min_range = min(msg.ranges)
        min_index = msg.ranges.index(min_range)

        # Calculate angle of that index
        angle = msg.angle_min + min_index * msg.angle_increment
        angle_deg = math.degrees(angle)

        rospy.loginfo("Closest obstacle at: %.2f meters, Angle: %.2f radians (%.1f°)", min_range, angle, angle_deg)
        logged_this_stop = True
    elif current_speed != 0.0 or current_angle != 0.0:
        logged_this_stop = False

def main():
    rospy.init_node('my_laser_listener')

    rospy.Subscriber('/car_1/scan', LaserScan, scan_callback)
    rospy.Subscriber('/car_1/multiplexer/command', AckermannDrive, command_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
