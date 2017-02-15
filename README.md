# euler_imu package
# IMU_MicroStrain-3DM-GX3-35 management
***v2017-02-15***

## Concepts

The 3DM-GX3 ® -35 is a high-performance and miniature Attitude Heading Reference System (AHRS) with GPS. It combines MEMS sensor technology and a highly sensitive embedded GPS receiver. It incorporates a triaxial accelerometer, triaxial gyro, triaxial magnetometer, temperature sensors, and an on-board processor running a sophisticated fusion algorithm to provide orientation, inertial, and GPS measurements.

The 3DM-GX3-35 is capable of producing two types of data: “AHRS” (Attitude and Heading Reference System) data and “GPS” (Global Positioning Sensor) data:

**AHRS data** - Acceleration Vector, Gyro Vector, Euler Angles, etc.
**GPS data** - Latitude, Longitude, UTC, Satellites in view, etc.

### References

- 3DM-GX3 ® -35 Data Communications Protocol Manual - DCP Manual - 8500-0014 Revision 008 - "*3DM-GX3-35-Data-Communications-Protocol.pdf*"

- LORD PRODUCT DATASHEET - 3DM-GX3 -35® - Miniature Attitude Heading Reference System with GPS - "*3DM-GX3-35-Attitude-Heading-Reference-System-GPS-Data-Sheet.pdf*"

- MicroStrain Inc. - Orientation Quantity Conversion Formulas - 7-30-2003Orientation Conversion formulas - MicroStrain.pdf

About Euler angles

- http://www.exelisvis.fr/Ressources/Blog/TabId/1707/ArtMID/6442/ArticleID/14649/Yaw-Pitch-Roll-et-Euler--le-cauchemar-du-d233veloppeur-.aspx

- https://en.wikipedia.org/wiki/Davenport_chained_rotations

- https://www.geometrictools.com/Documentation/EulerAngles.pdf

- https://fr.wikipedia.org/wiki/R%C3%A8gle_de_la_main_droite

## Install ROS packages for 3DM-GX3-35

**Dirty but working fix**
[ClearPath robotics package](https://github.com/clearpathrobotics/microstrain_3dm_gx3_35) is performing correctly IMU data streaming **ONLY AFTER** intialisation from [Robot Perception & Navigation Group package](https://github.com/rpng/microstrain_3dm_gx3_35) 

### Repositories
https://github.com/clearpathrobotics/microstrain_3dm_gx3_35 >> forked to https://github.com/ralphseulin/microstrain_3dm_gx3_35

https://github.com/rpng/microstrain_3dm_gx3_35 >> forked to https://github.com/ralphseulin/microstrain_comm_35.git

https://github.com/ralphseulin/euler_imu.git

Move to your ROS workspace / src folder and clone the 3 repos:
`git clone https://github.com/ralphseulin/microstrain_3dm_gx3_35`
`git clone https://github.com/ralphseulin/microstrain_comm_35.git`
`git clone https://github.com/ralphseulin/euler_imu.git`

move to .../src/microstrain_comm_35 folder and switch to 'init' branch
`git checkout init`

compile:
`catkin_make`

## Install Device

Plug on USB2.0 or USB 3.0 port

**!!! Wait at least 30 seconds !!!**
    
Check the device features on USB port
    
`lsusb | grep Micro`

        Bus 004 Device 003: ID 199b:3d65 MicroStrain, Inc.
    
Check the port setting 

`ls -l /dev/ttyACM0`

Change permissions

`sudo chmod a+rw /dev/ttyACM0`

To avoid this use uDev rule to permanently set permissions:

`$ lsusb`

	Bus 002 Device 018: ID 199b:3d65 MicroStrain, Inc. 

IDs :

	Vendor = 199b
	Product = 3d65

edit or create the /etc/udev/rules.d/50-microstrain.rules file and add the following line:

	ATTRS{idVendor}=="199b", ATTRS{idProduct}=="3d65", MODE:="0666", GROUP:="dialout"

by doing the following:

`$ sudo gedit /etc/udev/rules.d/gedit 50-microstrain.rules`

restart udev with the following:

`$ sudo service udev reload`

`$ sudo service udev restart`

unplug and plug again your peripheral and check that the permissions are OK:

`$ ls -l /dev/input/ttyACM0`

## Test the ROS driver

### Initiate the IMU sensor

Plug on USB2.0 or USB 3.0 port

**!!! Wait at least 30 seconds !!!**

Start the initialization script:

`roslaunch euler_imu imu_gx3-35_init.launch --screen`

The launch file will **stop automatically after initialization** and a few seconds of data streaming.
The sensor LED will then blink @0.5Hz frequency.

### Start the IMU driver

`roslaunch euler_imu imu_gx3-35_full_start.launch --screen`

The terminal displays the Euler angles of the IMU in degrees - this is convenient and human-friendly:

	[INFO] [WallTime: 1487149331.136133] 
	 roll:      0.22 
	 pitch:    -2.01 
	 yaw:     -30.35

### Streamed data from the IMU

#### Pose

Message type: 

	geometry_msgs/PoseStamped

**!!! this is not a Pose message but a PoseStamped message !!!**

This is a message for representation of pose in free space, composed of position and orientation:

- Header
- Point position
	- geometry_msgs/Point position
- Quaternion orientation
	- geometry_msgs/Quaternion orientation

`rostopic echo /imu_3dm_node/imu/pose`

	---
	header: 
	  seq: 12599
	  stamp: 
	    secs: 1486724190
	    nsecs: 738229705
	  frame_id: /imu_link
	pose: 
	  position: 
	    x: 0.0
	    y: 0.0
	    z: 0.0
	  orientation: 
	    x: -0.0869959378347
	    y: 0.0824829230882
	    z: 0.329228593007
	    w: 0.936609207592
	---

http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html

#### Angular & Linear Accelerations

Message type:
	
	sensor_msgs/Imu

This is a message to hold data from an IMU (Inertial Measurement Unit) composed of orientation, angular velocity and linear acceleration:

- Header
	- std_msgs/Header header

- Orientation
	- geometry_msgs/Quaternion orientation
	- float64[9] orientation_covariance # Row major about x, y, z axes

- Angular velocity
	- geometry_msgs/Vector3 angular_velocity
	- float64[9] angular_velocity_covariance # Row major about x, y, z axes

- Linear Acceleration
	- geometry_msgs/Vector3 linear_acceleration
	- float64[9] linear_acceleration_covariance # Row major x, y z 

`rostopic echo /imu_3dm_node/imu/data`

	---
	header: 
	  seq: 1957
	  stamp: 
	    secs: 1486724191
	    nsecs: 438148933
	  frame_id: /imu_link
	orientation: 
	  x: -0.0870586089896
	  y: 0.0820396328395
	  z: 0.328820194391
	  w: 0.936785769002
	orientation_covariance: [0.0012250000000000002, 0.0, 0.0, 0.0, 0.0012250000000000002, 0.0, 0.0, 0.0, 0.0012250000000000002]
	angular_velocity: 
	  x: 0.000475883483887
	  y: -0.0134788062423
	  z: 0.0178283154964
	angular_velocity_covariance: [0.000144, 0.0, 0.0, 0.0, 0.000144, 0.0, 0.0, 0.0, 0.000144]
	linear_acceleration: 
	  x: -0.246200695634
	  y: -0.09714846313
	  z: 0.963926553726
	linear_acceleration_covariance: [0.009604000000000001, 0.0, 0.0, 0.0, 0.009604000000000001, 0.0, 0.0, 0.0, 0.009604000000000001]
	---

http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html

### Display the IMU data in RViz

Settings (saved in ~/.rviz/IMU.rviz):

- Set "fixed frame" to */imu_link*
- Add new display "rviz -> Axes"
- Add new display "rviz -> Pose", set topic to */imu_3dm_node/imu/pose* and set "Shape" to *Axes*
- Add new display "rviz_plugin_tutorials -> Imu" and set topic to */imu_3dm_node/imu/data*

Rviz axes color:

- red - x
- green - y
- blue -z

http://wiki.ros.org/rviz/DisplayTypes/Axes

[ImuDisplay documentation]( http://docs.ros.org/jade/api/rviz_plugin_tutorials/html/display_plugin_tutorial.html#trying-it-out) 

## Euler Angles

### tf.transformations

Homogeneous Transformation Matrices and Quaternions.

A library for calculating 4x4 matrices for translating, rotating, reflecting, scaling, shearing, projecting, orthogonalizing, and superimposing arrays of
3D homogeneous coordinates as well as for converting between rotation matrices, Euler angles, and quaternions. Also includes an Arcball control object and functions to decompose transformation matrices.

https://github.com/ros/geometry/blob/indigo-devel/tf/src/tf/transformations.py

We need to import the euler_from_quaternion function from the tf library since we will be converting some quaternions to Euler angles.

	from tf.transformations import euler_from_quaternion

And then we can perform the conversion - example of implementation with data = msg output from 
	(roll, pitch, yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

euler_from_quaternion

	def euler_from_quaternion(quaternion, axes='sxyz'):
	    """Return Euler angles from quaternion for specified axis sequence.
	    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
	    >>> numpy.allclose(angles, [0.123, 0, 0])
	    True
	    """
	    #Return Euler angles from rotation matrix for specified axis sequence.
	    return euler_from_matrix(quaternion_matrix(quaternion), axes)
	    
###Ressources

- Example of quaternion_from_euler use from RBX1 book
https://github.com/pirobot/rbx1/blob/indigo-devel/rbx1_nav/nodes/move_base_square.py

- http://answers.ros.org/question/69754/quaternion-transformations-in-python/

### euler_imu package

A dedicated package has been created to perform live conversion from Quaternions and display of the Euler angles in degrees

Create and compile the package:
 `catkin_create_pkg euler_imu rospy std_msgs sensor_msgs geometry_msgs tf`
 
 `catkin_make`
 
 Run the conversion (the IMU streaming has to be started before !)
 
 `rosrun euler_imu quaternion2euler.py`
 
	 [INFO] [WallTime: 1486744754.120629] 
	 roll:     44.53 
	 pitch:   -88.63 
	 yaw:    -139.41

Publish to dedicated topic and message type.

To do so, we created a euler_imu/msg/Eulers.msg file with the following content:

	Header header
	float64 roll
	float64 pitch
	float64 yaw
	
**!!! Angles are in radians !!!**

Modify the CMakeLists.txt

	find_package(catkin REQUIRED COMPONENTS
	  ....
	  # for Eulers message generation
	  message_generation
	)
	
	## Generate messages in the 'msg' folder
	# for Eulers message generation
	 add_message_files(
	   FILES
	   Eulers.msg
	 )

	## Generate added messages and services with any dependencies listed here
	 generate_messages(
	   DEPENDENCIES
	   geometry_msgs#   sensor_msgs
	 )

	catkin_package(
	....
	  # for message generation
	  CATKIN_DEPENDS message_runtime
	)

Modify the package.xml:

	  <build_depend>message_generation</build_depend>
	  <run_depend>message_runtime</run_depend>

Recompile the package

Check that the custom message is available:
`$ rosmsg show euler_imu/Eulers`

	std_msgs/Header header
	  uint32 seq
	  time stamp
	  string frame_id
	float64 roll
	float64 pitch
	float64 yaw
 
A /euler topic is now avalaible with Eulers messages published to it:

`$ rostopic echo /euler`

	---
	header: 
	  seq: 104
	  stamp: 
	    secs: 1487150302
	    nsecs: 588772941
	  frame_id: ''
	roll: 0.00345661584288
	pitch: -0.035785254091
	yaw: -0.524042546749
	---

http://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/#17106

http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

http://wiki.ros.org/action/show/msg?action=show&redirect=ROS%2FMessage_Description_Language

## Additional References
http://answers.ros.org/question/220799/not-seeing-topic-data-using-microstrain_3dm_gx3_45-with-a-3dm-gx3-35/

>I found out paulbovbel had created a branch of the code just for the 35 model.
>
>https://github.com/clearpathrobotics/...
>
>After installing that these were the steps to get it to run:
>
>If you haven started roscore start it by typing roscore in a terminal
>    in another terminal (may be specific to my case) go into the devel folder of the workspace where you installed the package and source the setup.bash file there. Command: . setup.bash
>    Make sure your sensor is plugged in and the LED is lit, then check the permissions by typing ls -l /dev/ttyACM0 your device may not be ttyACM0,
>    in which case type as far as the tty then tab twice to see the list. 
>    If there are no USB or ACM devices it may be missing. You can plug and unplug to see if any change to determine if your port is showing up and what it is.
>    If your permissions read "crw-rw----" you need to add RW by typing sudo chmod a+rw /dev/ttyACM0 it should now read "crw-rw-rw-" if you check again
>    start the driver by typing rosrun microstrain_3dm_gx3_35 imu_driver
>    If that works, to see the data in another terminal type: rostopic echo /imu3dmgx3/imu/data
>
>Hope this helps if anyone is stuck as I was.

