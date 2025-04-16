#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import math

import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint

count = 0
global_x = []
global_y = []

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
        pprint(msg)
        min_range = min(msg.ranges)
        min_index = msg.ranges.index(min_range)

        # Calculate angle of that index
        angle = msg.angle_min + min_index * msg.angle_increment
        angle_deg = math.degrees(angle)

        rospy.loginfo("Closest obstacle at: %.2f meters, Angle: %.2f radians (%.1f°)", min_range, angle, angle_deg)
        logged_this_stop = True
    elif current_speed != 0.0 or current_angle != 0.0:
        logged_this_stop = False

def scan_callback2(msg):
    global count
    angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

    x = msg.ranges*np.cos(angles)
    y = msg.ranges*np.sin(angles)

    fig1, ax1 = plt.subplots()
    ax1.scatter(x,y,s=2)
    ax1.arrow(0,0,0.5,0, head_width=0.1, head_length=.1, fc ='red',ec='red')
    ax1.set_aspect('equal')
    ax1.set_xlim(-10,10)
    ax1.set_ylim(-10,10)
    ax1.set_title("scan")
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")

    count +=1
    if count % 10 == 0:
        fig1.savefig('lidar.png')
    plt.close(fig1)


def main():
    rospy.init_node('my_laser_listener')

    rospy.Subscriber('/car_1/scan', LaserScan, scan_callback2)
    rospy.Subscriber('/car_1/multiplexer/command', AckermannDrive, command_callback)

    rospy.spin()

if __name__ == '__main__':
    main()






# UNCOMMENT TO PRINT LIDAR DATA WHEN CAR COMES TO A STOP
# logged_this_stop = False  # global flag

# def lidar_callback(data):
#     global speed, angle, logged_this_stop

#     if speed == 0.0 and angle == 0.0 and not logged_this_stop:
#         rospy.loginfo("LiDAR Raw Sensor Data:")
#         rospy.loginfo("angle_min: %.4f rad", data.angle_min)
#         rospy.loginfo("angle_max: %.4f rad", data.angle_max)
#         rospy.loginfo("angle_increment: %.4f rad", data.angle_increment)
#         rospy.loginfo("time_increment: %.6f sec", data.time_increment)
#         rospy.loginfo("scan_time: %.6f sec", data.scan_time)
#         rospy.loginfo("range_min: %.2f m", data.range_min)
#         rospy.loginfo("range_max: %.2f m", data.range_max)
#         rospy.loginfo("ranges[] length: %d", len(data.ranges))
#         rospy.loginfo("intensities[] length: %d", len(data.intensities))

#         # Optionally print first few raw range readings (unaltered)
#         rospy.loginfo("  First 5 ranges  : %s", [data.ranges[i] for i in range(min(5, len(data.ranges)))])

#         logged_this_stop = True
#     elif speed != 0.0 or angle != 0.0:
#         logged_this_stop = False