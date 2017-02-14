#!/usr/bin/env python

## Reads the IMU orientation and performs
## conversion from Quaternion to Euler Angles

## Display the Euler angles 
## in degrees for a human friendly understanding

## Subscribes to the topic /imu_3dm_node/imu/pose
## of type geometry_msgs/PoseStamped message 


import rospy

from geometry_msgs.msg import PoseStamped

## Not used for the moment - for future implementations
# from sensor_msgs.msg import Imu

## tf.transformations 
## https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/transformations.py
from tf.transformations import euler_from_quaternion

## To transform radians to degrees
from math import degrees

## Custom message to publish Euler angles
from euler_imu.msg import Eulers

class Quaternion2Euler():
    def __init__(self):
        
        self.euler_msg = Eulers()

        # Init node for conversion called "quaternion2euler"
        rospy.init_node('quaternion2euler', anonymous=False)
        
        # Set rospy to execute a shutdown function when exiting
        rospy.on_shutdown(self.shutdown)

        ## Create subscribers and publishers.

        ## subscribes to the topic /imu_3dm_node/imu/pose 
        ## and calls the callback function for every received message
        rospy.Subscriber("/imu_3dm_node/imu/pose", PoseStamped, self.callback)
        
        #rospy.Subscriber("/imu_3dm_node/imu/data", Imu, self.callback)
        
        ## publisher to /euler topic
        self.pub_euler = rospy.Publisher("/euler", Eulers, queue_size=10)


    def callback(self,data):
        # display the fully-qualified name of the node 
        # + data axes values for every received message
        #rospy.loginfo("Pose orientation x = " + '{}'.format(data.pose.orientation.x))
        #rospy.loginfo("Pose orientation x = ")

        # Convert quaternions to Euler angles.
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        
        ## Display euler angles in the terminal
        rospy.loginfo('\n roll:{0:10.2f} \n pitch:{1:9.2f} \n yaw:{2:11.2f}'.format(degrees(roll), degrees(pitch), degrees(yaw)))
        
        self.fill_euler_msg(data, roll, pitch, yaw)
        
        ## publish the message
        self.pub_euler.publish(self.euler_msg)
    
    ## Fill in Euler angle message.
    def fill_euler_msg(self, msg, r, p, y):
        self.euler_msg.header.stamp = msg.header.stamp
        self.euler_msg.roll  = r
        self.euler_msg.pitch = p
        self.euler_msg.yaw   = y

    def shutdown(self):
        # Always stop the robot when shutting down the node
        
        # display "Stopping the node ...." 
        rospy.loginfo("Stopping the Quaternion2Euler node ....")


if __name__ == '__main__':
    #standard Python block for running the script. 
    #we simply instantiate the NewJoy2Twist class and manage exception
    try:
        Quaternion2Euler()
        # simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Quaternion2Euler node terminated")
