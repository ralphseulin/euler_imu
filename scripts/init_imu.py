#!/usr/bin/env python

## Reads the IMU orientation and performs
## conversion from Quaternion to Euler Angles

## Display the Euler angles 
## in degrees for a human friendly understanding

## Subscribes to the topic /imu_3dm_node/imu/pose
## of type geometry_msgs/PoseStamped message 

import rospy

class Init_Imu():
    def __init__(self):
          
        child = subprocess.Popen(["roslaunch","microstrain_comm_35","imu.launch"])
        #child.wait() #You can use this line to block the parent process untill the child process finished.
        print("parent process: roslaunch microstrain_comm_35 imu.launch")
        print(child.poll())

        rospy.loginfo('The PID of child: %d', child.pid)
        print ("The PID of child:", child.pid)

        rospy.sleep(2)

        child.send_signal(signal.SIGINT) #You may also use .terminate() method
        child.terminate()
        child.kill()
        #for more: https://docs.python.org/2/library/subprocess.html

if __name__ == '__main__':
    #standard Python block for running the script. 
    #we simply instantiate the NewJoy2Twist class and manage exception
    try:
        Init_Imu()
        # simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Init_Imu node terminated")
