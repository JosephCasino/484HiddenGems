#!/usr/bin/env python3


#This is the edited keyboard teleops code that prints the LiDAR data to the terminal only when the car comes to a complete stop. 

import rospy
import sys
import select
import termios
import tty

from ackermann_msgs.msg import AckermannDrive

from sensor_msgs.msg import LaserScan


keyBindings = {'w':(1.0,  0.0), # move forward
               'd':(1.0, -1.0), # move foward and right
               'a':(1.0 , 1.0), # move forward and left
               's':(-1.0, 0.0), # move reverse
               'q':(0.0,  0.0)} # all stop

speed_limit = 0.250
angle_limit = 0.325

def getKey():
   tty.setraw(sys.stdin.fileno())
   select.select([sys.stdin], [], [], 0)
   key = sys.stdin.read(1)
   termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
   return key

def vels(speed, turn):
  return 'currently:\tspeed {}\tturn {}'.format(speed, turn)




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




if __name__== '__main__':
  settings    = termios.tcgetattr(sys.stdin)
  command_pub = rospy.Publisher('/car_1/multiplexer/command', AckermannDrive, queue_size = 1)
  rospy.init_node('keyboard_teleop', anonymous = True)

  #subscribe to Lidar data
  #rospy.Subscriber('/car_1/scan', LaserScan, lidar_callback)


  speed  = 0.0
  angle  = 0.0
  status = 0.0

  try:
    while True:
       key = getKey()
       if key in keyBindings.keys():
          speed = keyBindings[key][0]
          angle = keyBindings[key][1]
       else:
          speed = 0.0
          angle = 0.0
          if (key == '\x03'):
             break
       command                = AckermannDrive();
       command.speed          = speed * speed_limit
       command.steering_angle = angle * angle_limit
       command_pub.publish(command)

  except:
    print('raise exception: key binding error')

  finally:
    command = AckermannDrive();
    command.speed = speed * speed_limit
    command.steering_angle = angle * angle_limit
    command_pub.publish(command)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
