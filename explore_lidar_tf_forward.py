#!/usr/bin/env python3

import rospy
import math
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

class LidarExplorerTF:
    def __init__(self):
        rospy.init_node('lidar_explorer_tf')
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber('/car_1/scan', LaserScan, self.scan_callback)

        rospy.loginfo("Lidar Explorer TF Node with forward preference started")

    def get_robot_pose(self):
        try:
            # Wait for the transform to be available
            self.tf_listener.waitForTransform('map', 'car_1/base_link', rospy.Time(0), rospy.Duration(1.0))
            # Get the transform from 'map' to 'car_1/base_link'
            # This will give us the robot's position and orientation in the map frame
            (trans, rot) = self.tf_listener.lookupTransform('map', 'car_1/base_link', rospy.Time(0))
            # Convert the quaternion to Euler angles
            # We only need the yaw angle for 2D navigation
            euler = tf.transformations.euler_from_quaternion(rot)
            return (trans[0], trans[1], euler[2])  # x, y, yaw
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed.")
            return None

    def scan_callback(self, scan):
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return

        max_val = max(scan.ranges)
        threshold = 0.1
        max_indices = [i for i, r in enumerate(scan.ranges) if abs(r - max_val) < threshold]
        angles = [scan.angle_min + i * scan.angle_increment for i in max_indices]

        forward_range = math.radians(30)
        forward_angles = [a for a in angles if abs(a) < forward_range]

        if forward_angles:
            best_angle = min(forward_angles, key=lambda a: abs(a))
        else:
            best_angle = min(angles, key=lambda a: abs(a))

        robot_x, robot_y, robot_yaw = robot_pose
        goal_x = robot_x + 1.5 * math.cos(robot_yaw + best_angle)
        goal_y = robot_y + 1.5 * math.sin(robot_yaw + best_angle)

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

        rospy.loginfo("Published goal with forward preference: (%.2f, %.2f), angle: %.2f°", goal_x, goal_y, math.degrees(best_angle))

if __name__ == '__main__':
    try:
        explorer = LidarExplorerTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
