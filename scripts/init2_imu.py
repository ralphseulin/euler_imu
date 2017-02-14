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
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the file path for rospy_tutorials
        package_path = rospack.get_path('microstrain_3dm_gx3_35_init')
                
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        #launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/ralphseulin/ros/indigo/catkin_ws/src/microstrain_3dm_gx3_35_init/launch/imu.launch"])
        launch = roslaunch.parent.ROSLaunchParent(uuid, [package_path + "/launch/imu.launch"])
        
        launch.start()
        
        rospy.sleep(3)
        
        launch.shutdown()

if __name__ == '__main__':
    #standard Python block for running the script. 
    #we simply instantiate the NewJoy2Twist class and manage exception
    try:
        Init2_Imu()
        # simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Init_Imu node terminated")
