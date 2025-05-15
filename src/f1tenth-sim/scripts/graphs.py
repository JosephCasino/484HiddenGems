#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

# Data storage
odom_poses = []
gt_poses = []
lidar_stds = []

def odom_callback(msg):
    pos = msg.pose.pose.position
    timestamp = rospy.get_time()
    odom_poses.append((timestamp, pos.x, pos.y))
    rospy.logdebug(f"[ODOM] Time: {timestamp:.2f} | Pos: ({pos.x:.2f}, {pos.y:.2f})")

def scan_callback(msg):
    ranges = np.array(msg.ranges)
    ranges = np.nan_to_num(ranges, nan=0.0, posinf=10.0)
    ranges = np.clip(ranges, 0, 10)

    std = np.std(ranges)
    lidar_stds.append((rospy.get_time(), std))
    rospy.logdebug(f"[SCAN] Time: {rospy.get_time():.2f} | STD: {std:.3f}")

def model_states_callback(msg):
    try:
        idx = msg.name.index("car_1")
        pos = msg.pose[idx].position
        gt_poses.append((rospy.get_time(), pos.x, pos.y))
        rospy.logdebug(f"[GT] Time: {rospy.get_time():.2f} | GT Pos: ({pos.x:.2f}, {pos.y:.2f})")
    except ValueError:
        rospy.logwarn("Model 'car_1' not found in /gazebo/model_states")

def plot_and_save(x, y, title, xlabel, ylabel, filename):
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, color='tab:blue', linewidth=2)
    plt.title(title, fontsize=14)
    plt.xlabel(xlabel, fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.tight_layout()
    plt.savefig(filename)
    plt.close()
    rospy.loginfo(f"Saved plot: {filename}")

def save_graphs():
    if not odom_poses or not gt_poses:
        rospy.logwarn("Insufficient data to generate graphs.")
        return

    odom_times, x_odom, y_odom = zip(*odom_poses)
    gt_times, x_gt, y_gt = zip(*gt_poses)

    gt_tree = cKDTree(np.array(gt_times).reshape(-1, 1))
    errors = []
    matched_times = []

    for t, xo, yo in zip(odom_times, x_odom, y_odom):
        dist, idx = gt_tree.query([t])
        xg, yg = x_gt[idx], y_gt[idx]
        error = np.linalg.norm([xo - xg, yo - yg])
        errors.append(error)
        matched_times.append(t)

    plot_and_save(matched_times, errors, 
                  "Localization Error Over Time", 
                  "Time (s)", "Position Error (m)", 
                  "localization_error.png")

    if lidar_stds:
        t_lidar, stds = zip(*lidar_stds)
        plot_and_save(t_lidar, stds, 
                      "LiDAR Scan Noise", 
                      "Time (s)", "Std Dev of Ranges (m)", 
                      "lidar_noise_std.png")
    else:
        rospy.logwarn("No LiDAR data collected.")

def main():
    rospy.init_node("error_and_noise_analyzer", anonymous=True)
    rospy.Subscriber("/car_1/base/odom", Odometry, odom_callback)
    rospy.Subscriber("/car_1/scan", LaserScan, scan_callback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

    rospy.on_shutdown(save_graphs)
    rospy.loginfo("Error + Noise Analyzer running. Press Ctrl+C to stop and save graphs.")
    rospy.spin()

if __name__ == "__main__":
    main()
