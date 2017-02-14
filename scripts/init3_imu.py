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

## Python API for rospkg - http://docs.ros.org/independent/api/rospkg/html/python_api.html
## Python API for roslaunch - http://wiki.ros.org/roslaunch/API%20Usage#Simple_usage_example-1 
##                          - refer to 2-simple example

import rospy
import roslaunch
import rospkg

class Init2_Imu():
    def __init__(self):
   
        # get an instance of RosPack with the default search paths
        # mandatory
        rospack = rospkg.RosPack()
  
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # catkin_ws/src/microstrain_3dm_gx3_35_init/launch/imu.launch
        launch = roslaunch.parent.ROSLaunchParent(uuid, [rospack.get_path('microstrain_comm_35') + "/launch/imu.launch"])
        
        launch.start()
        
        rospy.sleep(3)
        
        launch.shutdown()

if __name__ == '__main__':
    # standard Python block for running the script. 
    # we simply instantiate the Init2_Imu class and manage exception
    try:
        Init2_Imu()
        # simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Init_Imu node terminated")
