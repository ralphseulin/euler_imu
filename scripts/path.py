#!/usr/bin/env python

## Initiate the IMU microstrain_3dm_gx3_35

## Starts and stops automatically 
## the "microstrain_3dm_gx3_35_init/launch/imu.launch" launch file

## To be performed only one time after plugging the IMU device
## to a USB port

## Do not forget to setup uDev rule to enable rw permissions on the port
## /dev/ttyACM0

## initialisation is performing well until you unplug the device 
## or shutdown the computer

import rospy
import roslaunch
import rospkg

class Init2_Imu():
    def __init__(self):
        # Init node for conversion called "quaternion2euler"
        rospy.init_node('path_find', anonymous=False)
        
        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)
        
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the file path for rospy_tutorials
        package_path = rospack.get_path('microstrain_comm_35')
        
        rospy.loginfo("path for microstrain_3dm_gx3_35_init = " + package_path)    
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        
        # display "Stopping the node ...." 
        rospy.loginfo("Stopping the path_find node ....")

if __name__ == '__main__':
    #standard Python block for running the script. 
    #we simply instantiate the NewJoy2Twist class and manage exception
    try:
        Init2_Imu()
        # simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Init_Imu node terminated")
