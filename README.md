Launch procedure : 

1. EtherCAT - motor controller launch

NUC1 $ roscore

NUC2 $ sudo -s

(Go into Root)

NUC2 $ rosrun ethercat_test ethercat_mobile_control

2. Sensor Launch

Xavier1 $ velodyne_eth

(Same as : Xavier1 $ sudo ifconfig eth1 192.168.3.70

Xavier1 $ sudo route add 192.168.1.202 eth1)

Xavier1 $ roslaunch velodyne_pointcloud VLP16_points.launch

Check if the scan topic works well

RPB $ mobile_sensor1

(Magnet encoder and IMU)

NUC3 $ rosrun mobile_control wheel_odom

Xavier2 $ roslaunch dualarm_mobile_bringup bringup_v2.launch

Xavier3 $ roslaunch dualarm_mobile_localization gmapping.launch

(Onboard SLAM)

RemotePC1 $ roslaunch dualarm_mobile_localization gmapping_monitor.launch

(For the purpose of SLAM display)

3. Controller and Planner

NUC4 $ rosrun mobile_control pd_control_trans_only

NUC5 $ rosrun path_planner velocity_plan

4. Input Goal position

RemotePC2 $ rostopic pub /move_base_simple (tab --> Position and Orientation-quaternion) 

If you want to use camera and velodyne without wheel odometry, then follow the instruction.

Go to the directory 

/catkin_ws/src/dualarm_mobile/dualarm_mobile_bringup/launch/

and then, in the sensor.launch file,

modify like the following in the launch tag:

    <include file="$(find realsense2_camera)/launch/rs_t265.launch" />
	<arg name="unite_imu_method" value="copy" />
    </include>

This modification would allow you to use only camera when you initiate the bringup.launch file.
