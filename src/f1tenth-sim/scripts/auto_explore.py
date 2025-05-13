#!/usr/bin/env python3

import rospy
from math import atan2, sqrt, cos, sin
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from pynput import keyboard
import threading

# Configurable parameters
LOOKAHEAD_DISTANCE = 1
MAX_LINEAR_SPEED = 0.1
MAX_ANGULAR_SPEED = 0.6
WHEELBASE = 0.5

# Globals
waypoints = []
current_pose = None
manual_mode = False
manual_speed = 0.0
manual_angle = 0.0

# Key mappings
key_bindings = {
    'w': (1.0, 0.0),
    'a': (1.0, 1.0),
    'd': (1.0, -1.0),
    's': (-1.0, 0.0),
    'q': (0.0, 0.0),
}

def odom_callback(msg):
    global current_pose
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    yaw = 2 * atan2(ori.z, ori.w)
    current_pose = (pos.x, pos.y, yaw)

def path_callback(msg):
    global waypoints
    waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

def find_target_waypoint(position, waypoints, lookahead):
    farthest_wp = None
    max_dist = -1.0
    for wp in waypoints:
        dx = wp[0] - position[0]
        dy = wp[1] - position[1]
        dist = sqrt(dx**2 + dy**2)
        if dist > lookahead:
            return wp
        if dist > max_dist:
            farthest_wp = wp
            max_dist = dist
    return farthest_wp if farthest_wp else position

def pure_pursuit_control():
    if current_pose is None or len(waypoints) < 1:
        return 0.0, 0.0

    x, y, yaw = current_pose
    target = find_target_waypoint((x, y), waypoints, LOOKAHEAD_DISTANCE)
    dx = target[0] - x
    dy = target[1] - y

    # Transform to local frame
    local_x = cos(-yaw) * dx - sin(-yaw) * dy
    local_y = sin(-yaw) * dx + cos(-yaw) * dy

    # Clamp to avoid division by zero
    clamped_x = max(local_x, 0.1)

    steering_angle = atan2(2 * WHEELBASE * local_y, LOOKAHEAD_DISTANCE**2)
    angle = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, steering_angle))
    speed = MAX_LINEAR_SPEED

    return speed, angle

def keyboard_listener():
    global manual_mode, manual_speed, manual_angle
    def on_press(key):
        global manual_mode, manual_speed, manual_angle
        try:
            k = key.char
            if k == 'm':
                manual_mode = not manual_mode
                mode = "MANUAL" if manual_mode else "AUTONOMOUS"
                rospy.loginfo(f"[Keyboard] Toggled to {mode} mode.")
                manual_speed = 0.0
                manual_angle = 0.0
            elif manual_mode and k in key_bindings:
                speed, angle = key_bindings[k]
                manual_speed = speed * 0.25
                manual_angle = angle * 0.325
        except AttributeError:
            pass
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

if __name__ == '__main__':
    rospy.init_node('auto_explore_with_keyboard')
    rospy.Subscriber('/car_1/base/odom', Odometry, odom_callback)
    rospy.Subscriber('/planned_path', Path, path_callback)
    command_pub = rospy.Publisher('/car_1/multiplexer/command', AckermannDrive, queue_size=1)

    # Launch keyboard listener in background
    threading.Thread(target=keyboard_listener, daemon=True).start()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        command = AckermannDrive()

        if manual_mode:
            command.speed = manual_speed
            command.steering_angle = manual_angle
            rospy.loginfo_throttle(1, f"[Manual] speed={manual_speed:.2f}, angle={manual_angle:.2f}")
        else:
            speed, angle = pure_pursuit_control()
            command.speed = speed
            command.steering_angle = angle
            rospy.loginfo_throttle(1, f"[Auto] speed={speed:.2f}, angle={angle:.2f}")

        command_pub.publish(command)
        rate.sleep()