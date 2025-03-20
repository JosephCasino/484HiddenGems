import rospy
import numpy as np
import argparse

from gazebo_msgs.msg import  ModelState
from controller import vehicleController
import time
from waypoint_list import WayPoints
from util import euler_to_quaternion, quaternion_to_euler
import matplotlib.pyplot as plt

def run_model():
    acceleartion_list = []
    x_log_list = []
    y_log_list = []
    rospy.init_node("model_dynamics")
    controller = vehicleController()

    waypoints = WayPoints()
    pos_list = waypoints.getWayPoints()
    pos_idx = 1

    target_x, target_y = pos_list[pos_idx]
    way_point_list = np.array(pos_list)
    x_way_list, y_way_list = way_point_list[:,0], way_point_list[:,1]

    def shutdown():
        """Stop the car when this ROS node shuts down"""
        controller.stop()
        rospy.loginfo("Stop the car")
    

    
    rospy.on_shutdown(shutdown)

    rate = rospy.Rate(100)  # 100 Hz
    rospy.sleep(0.0)
    start_time = rospy.Time.now()
    prev_wp_time = start_time

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state

        # Get the current position and orientation of the vehicle
        currState =  controller.getModelState()
        
        if not currState.success:
            continue

        # Compute relative position between vehicle and waypoints
        distToTargetX = abs(target_x - currState.pose.position.x)
        distToTargetY = abs(target_y - currState.pose.position.y)

        cur_time = rospy.Time.now()
        if (cur_time - prev_wp_time).to_sec() > 4:
            print(f"failure to reach {pos_idx}-th waypoint in time")

            plt.plot(np.linspace(0, len(acceleartion_list)*0.01, len(acceleartion_list)), acceleartion_list,  label = "Acceleration")
            plt.xlabel("Time (s)")
            plt.ylabel('Acceleartion (cm/s^2)')
            plt.title("Acceleration-Time plot")
            plt.legend()
            plt.grid(True)
            plt.savefig("accel-time", dpi = 300, bbox_inches = 'tight')
            plt.close()

            plt.plot(x_log_list,y_log_list, linestyle = "-", marker = 'o', color = 'black', label = "Vehicle trajectory", zorder = 1)
            plt.scatter(x_way_list, y_way_list, color = 'red', marker = 'x', label = "Waypoints", zorder = 2)
            plt.scatter(x_log_list[0], y_log_list[0], color = 'blue', marker = 'o', label = "Initial position", zorder = 3)
            plt.xlabel("x")
            plt.ylabel('y')
            plt.title("x-y plot")
            plt.legend()
            plt.grid(True, zorder = 0)
            plt.savefig("x-y", dpi = 300, bbox_inches = 'tight')
            plt.close()

            return False, pos_idx, (cur_time - start_time).to_sec()

        if (distToTargetX < 2 and distToTargetY < 2):
            # If the vehicle is close to the waypoint, move to the next waypoint
            prev_pos_idx = pos_idx
            pos_idx = pos_idx+1

            if pos_idx == len(pos_list): #Reached all the waypoints
                print("Reached all the waypoints")
                total_time = (cur_time - start_time).to_sec()
                print("Total Time: ", total_time)

                plt.plot(np.linspace(0, len(acceleartion_list)*0.01, len(acceleartion_list)), acceleartion_list,  label = "Acceleration")
                plt.xlabel("Time (s)")
                plt.ylabel('Acceleartion (cm/s^2)')
                plt.title("Acceleration-Time plot")
                plt.legend()
                plt.grid(True)
                plt.savefig("accel-time", dpi = 300, bbox_inches = 'tight')
                plt.close()

                plt.plot(x_log_list,y_log_list, linestyle = "-", marker = 'o', color = 'black', label = "Vehicle trajectory", zorder = 1)
                plt.scatter(x_way_list, y_way_list, color = 'red', marker = 'x', label = "Waypoints", zorder = 2)
                plt.scatter(x_log_list[0], y_log_list[0], color = 'blue', marker = 'o', label = "Initial position", zorder = 3)
                plt.xlabel("x")
                plt.ylabel('y')
                plt.title("x-y plot")
                plt.legend()
                plt.grid(True, zorder = 0)
                plt.savefig("x-y", dpi = 300, bbox_inches = 'tight')
                plt.close()
                return True, pos_idx, total_time

            target_x, target_y = pos_list[pos_idx]

            time_taken = (cur_time- prev_wp_time).to_sec()
            prev_wp_time = cur_time
            print(f"Time Taken: {round(time_taken, 2)}", "reached",pos_list[prev_pos_idx][0],pos_list[prev_pos_idx][1],"next",pos_list[pos_idx][0],pos_list[pos_idx][1])

        acel, x_log, y_log = controller.execute(currState, [target_x, target_y], pos_list[pos_idx:])
        acceleartion_list.append(acel)
        x_log_list.append(x_log)
        y_log_list.append(y_log)

if __name__ == "__main__":
    try:
        status, num_waypoints, time_taken = run_model()
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("Shutting down")
