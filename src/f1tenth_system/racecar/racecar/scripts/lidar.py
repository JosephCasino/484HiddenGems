#!/usr/bin/env python3

import importlib

import rospy
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
import actionlib
import rostopic
import rosservice
from threading import Thread
from rosservice import ROSServiceException

import numpy as np

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
import math

import matplotlib
matplotlib.use('Agg')
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
    current_speed = msg.data
    current_angle = msg.steering_angle

def scan_callback(msg):
    global current_speed, current_angle, logged_this_stop

    # if current_speed == 0.0 and current_angle == 0.0 and not logged_this_stop:
    # Find the closest range and its index
    # pprint(msg)
    min_range = min(msg.ranges)
    min_index = msg.ranges.index(min_range)

    # Calculate angle of that index
    angle = msg.angle_min + min_index * msg.angle_increment
    angle_deg = math.degrees(angle)

    rospy.loginfo("Closest obstacle at: %.2f meters, Angle: %.2f radians (%.1f°)", min_range, angle, angle_deg)
    logged_this_stop = True
    # elif current_speed != 0.0 or current_angle != 0.0:
        # logged_this_stop = False

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

    rospy.Subscriber('/scan', LaserScan, scan_callback2, queue_size=1)
    rospy.Subscriber('/vesc/commands/motor/speed', AckermannDrive, command_callback)

    rospy.spin()

# if __name__ == '__main__':
#     main()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
