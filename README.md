Prerequisite pkg : dualarm_mobile_description

If you want to use only camera, then follow the instruction.

Go to the directory 

/catkin_ws/src/dualarm_mobile/dualarm_mobile_bringup/launch/

and then, in the sensor.launch file,

modify like the following in the launch tag:

    <include file="$(find realsense2_camera)/launch/rs_t265.launch" />
	<arg name="unite_imu_method" value="copy" />
    </include>

This modification would allow you to use only camera when you initiate the bringup.launch file.
