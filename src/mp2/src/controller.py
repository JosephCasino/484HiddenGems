import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
import math
from util import euler_to_quaternion, quaternion_to_euler
import time

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)
        self.prev_vel = 0
        self.L = 1.75 # Wheelbase, can be get from gem_control.py
        self.log_acceleration = True

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp


    # Tasks 1: Read the documentation https://docs.ros.org/en/fuerte/api/gazebo/html/msg/ModelState.html
    #       and extract yaw, velocity, vehicle_position_x, vehicle_position_y
    # Hint: you may use the the helper function(quaternion_to_euler()) we provide to convert from quaternion to euler
    def extract_vehicle_info(self, currentPose):

        ####################### TODO: Your TASK 1 code starts Here #######################
        pos_x, pos_y, vel, yaw = 0, 0, 0, 0

        pos_x = currentPose.pose.position.x
        pos_y = currentPose.pose.position.y
        vel = currentPose.twist.linear.x

        roll, pitch, yaw = quaternion_to_euler(currentPose.pose.orientation.x,currentPose.pose.orientation.y,
                                               currentPose.pose.orientation.z,currentPose.pose.orientation.w)

        ####################### TODO: Your Task 1 code ends Here #######################

        return pos_x, pos_y, vel, yaw # note that yaw is in radian

    # Task 2: Longtitudal Controller
    # Based on all unreached waypoints, and your current vehicle state, decide your velocity
    def longititudal_controller(self, curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints):

        ####################### TODO: Your TASK 2 code starts Here #######################     
        # Target Velocity = 10 m/s
    
        x1, y1 = curr_x, curr_y  # Current vehicle position
        x2, y2 = x1, y1
        x3, y3 = x1, y1
        if len(future_unreached_waypoints)>1:
            x2, y2 = future_unreached_waypoints[0]  # Next waypoint
        if len(future_unreached_waypoints)>2:
            x3, y3 = future_unreached_waypoints[1]  # Second future waypoint
        
        # Compute angles
        angle1 = np.arctan2(y2 - y1, x2 - x1)  # Angle from current position to next waypoint
        angle2 = np.arctan2(y3 - y2, x3 - x2)  # Angle from next waypoint to second waypoint
        angle_diff = abs(angle2 - angle1)  # Curvature estimation
        
        # Adjust speed based on curvature
        if angle_diff > 0.3 and angle_diff <=0.5:  # If turn is sharp
            target_velocity = 10 # Slow down
        elif angle_diff > 0.5:  # If turn is sharp
            target_velocity = 8  # Slow down
        else:
            target_velocity = 15.0  # Maintain high speed

        ####################### TODO: Your TASK 2 code ends Here #######################
        return target_velocity


    # Task 3: Lateral Controller (Pure Pursuit)
    def pure_pursuit_lateral_controller(self, curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints,curr_vel):

        ####################### TODO: Your TASK 3 code starts Here #######################
        L = self.L  # Wheelbase
        # ld = min(max(2.0, 0.5 * future_unreached_waypoints[0][0]), 5.0)  # Dynamic lookahead distance
        
        # Adaptive lookahead distance: increases with speed, limited between 2m and 6m
        ld = max(4, min(0.5 * abs(curr_vel), 8))

        # Find the best lookahead point from future waypoints
        for waypoint in future_unreached_waypoints:
            dist = np.sqrt((waypoint[0] - curr_x)**2 + (waypoint[1] - curr_y)**2)
            if dist >= ld:
                target_point = waypoint
                break

        tx, ty = target_point
        angle_to_target = np.arctan2(ty - curr_y, tx - curr_x)
        alpha = angle_to_target - curr_yaw
        target_steering = np.arctan2(2 * L * np.sin(alpha), ld)


        ####################### TODO: Your TASK 3 code starts Here #######################
        return target_steering


    def execute(self, currentPose, target_point, future_unreached_waypoints):
        # Compute the control input to the vehicle according to the
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   target_point: [target_x, target_y]
        #   future_unreached_waypoints: a list of future waypoints[[target_x, target_y]]
        # Output: None

        curr_x, curr_y, curr_vel, curr_yaw = self.extract_vehicle_info(currentPose)

        # Acceleration Profile
        if self.log_acceleration:
            acceleration = (curr_vel- self.prev_vel) * 100 # Since we are running in 100Hz
            x_log = curr_x
            y_log = curr_y



        target_velocity = self.longititudal_controller(curr_x, curr_y, curr_vel, curr_yaw, future_unreached_waypoints)
        target_steering = self.pure_pursuit_lateral_controller(curr_x, curr_y, curr_yaw, target_point, future_unreached_waypoints,curr_vel)


        #Pack computed velocity and steering angle into Ackermann command
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = target_velocity
        newAckermannCmd.steering_angle = target_steering

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)

        return acceleration, x_log, y_log

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
