Prerequisite pkg : dualarm_mobile_description

Launch procedure : 

$ sudo ifconfig eth1 192.168.3.70

$ sudo route add 192.168.1.202 eth1

$ roslaunch velodyne_pointcloud VLP16_points.launch

$ roslaunch dualarm_mobile_bringup bringup.launch

If you want to use camera and velodyne without wheel odometry, then follow the instruction.

Go to the directory 

/catkin_ws/src/dualarm_mobile/dualarm_mobile_bringup/launch/

and then, in the sensor.launch file,

modify like the following in the launch tag:

    <include file="$(find realsense2_camera)/launch/rs_t265.launch" />
	<arg name="unite_imu_method" value="copy" />
    </include>

This modification would allow you to use only camera when you initiate the bringup.launch file.
