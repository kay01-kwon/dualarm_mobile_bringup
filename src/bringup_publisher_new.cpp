#include <dualarm_mobile_bringup/publisher.h>
#include <dualarm_mobile_bringup/attitude_estimator.hpp>

using Eigen::Vector3d;
using Eigen::VectorXd;

using std::cout;
using std::endl;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bring_publisher");
  attitude_estimator estimator;
  estimator.PublisherSetting();
  estimator.EncoderSubscriberSetting();

  estimator.InitiateVariables();

  ros::NodeHandle nh;

  tf::TransformBroadcaster broadcaster;

  nh.param<int>("rate", rate, 40);

  ros::Rate loop_rate(rate);

  while(ros::ok())
  {

    Vector3d b_p_base_arm;
    Vector3d b_p_camera_sensor;
    Vector3d b_p_lidar_sensor;
    VectorXd roll_pitch = VectorXd(2);

    estimator.RelativeInfo();
    if(estimator.is_init == true)
    {
      b_p_base_arm = estimator.b_p_base_arm;
      cout<<b_p_base_arm<<endl;
      cout<<endl;
      //b_p_camera_sensor = estimator.b_p_camera_sensor;
      //b_p_lidar_sensor = estimator.b_p_lidar_sensor;
      roll_pitch = estimator.roll_pitch;

      ros::Time currentTime = ros::Time::now();

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.305, 0.0, 0.485)),
    	currentTime,"odom", "camera_odom_frame"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(roll_pitch(0),roll_pitch(1),0)),
      tf::Vector3(-0.305, 0.0, -0.485)),
    	currentTime,"camera_pose_frame", "base_footprint"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
    	currentTime,"base_footprint", "base_link"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.281, -0.215, -0.020)),
    	currentTime,"base_link", "br_wheel_link"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.281, 0.215, -0.020)),
    	currentTime,"base_link", "bl_wheel_link"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.281, -0.215, -0.020)),
    	currentTime,"base_link", "fr_wheel_link"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.281, 0.215, -0.020)),
    	currentTime,"base_link", "fl_wheel_link"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
    	currentTime,"base_link", "imu_link"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0,PI/4)), tf::Vector3(0.340, 0.235, 0.57)),
    	currentTime,"base_link", "velodyne"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, 0.0)), tf::Vector3(0.42, 0.0, 0.09)),
    	currentTime,"base_link", "base_sonar_front"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, PI)), tf::Vector3(-0.41, 0.0, 0.09)),
    	currentTime,"base_link", "base_sonar_rear"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, PI/2)), tf::Vector3(0.0, 0.3, 0.09)),
    	currentTime,"base_link", "base_sonar_left"));

      broadcaster.sendTransform(
      tf::StampedTransform(
    	tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, -PI/2)), tf::Vector3(0.0, -0.3, 0.09)),
    	currentTime,"base_link", "base_sonar_right"));
    }
      ros::spinOnce();
      loop_rate.sleep();

  } //End while (ros::ok())

  return 0;

}
