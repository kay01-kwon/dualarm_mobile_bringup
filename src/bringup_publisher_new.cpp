#include <dualarm_mobile_bringup/attitude_estimator.hpp>

using Eigen::Vector3d;
using Eigen::VectorXd;

using std::cout;
using std::endl;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "bring_publisher");
  
  attitude_estimator estimator;

  bool odom_init = false;

  // Relative translation info
  Vector3d odom_to_camera_odom_;
  Vector3d camera_pose_to_base_footprint_;
  Vector3d base_footprint_to_base_link_;
  Vector3d base_link_to_Velodyne1_;
  Vector3d base_link_to_Velodyne2_;
  Vector3d base_link_to_base_arm_;

  VectorXd p_q_b_ = VectorXd(4);
  VectorXd p_q_b0_ = VectorXd(4);
  VectorXd b_q_p_ = VectorXd(4);

  estimator.PublisherSetting();
  estimator.EncoderSubscriberSetting();
  estimator.InitiateVariables();


  tf::TransformBroadcaster broadcaster;

  rate = 40;

  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    estimator.RelativeInfo();
    if(estimator.is_subscription == true) // If subscription occurs --> true!
    {
      camera_pose_to_base_footprint_ = estimator.camera_pose_to_base_footprint;
      base_footprint_to_base_link_ = estimator.base_footprint_to_base_link;
      base_link_to_Velodyne1_ = estimator.base_link_to_Velodyne1;
      base_link_to_Velodyne2_ = estimator.base_link_to_Velodyne2;
      base_link_to_base_arm_ = estimator.base_link_to_base_arm;
      p_q_b_ = estimator.p_q_b;
      b_q_p_ = estimator.b_q_p;

      if(odom_init == false) // Reference frame (Odom-camera_odom_frame) initialization
      {
        odom_to_camera_odom_ = estimator.odom_to_camera_odom;
        p_q_b0_ = estimator.p_q_b;

        odom_init = true;
      }

      ros::Time currentTime = ros::Time::now();

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(p_q_b0_(0),p_q_b0_(1),p_q_b0_(2),p_q_b0_(3)), 
          tf::Vector3(odom_to_camera_odom_(0),odom_to_camera_odom_(1),odom_to_camera_odom_(2))),
    	    currentTime,"odom", "camera_odom_frame"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(b_q_p_(0),b_q_p_(1),b_q_p_(2),b_q_p_(3)),
          tf::Vector3(camera_pose_to_base_footprint_(0),camera_pose_to_base_footprint_(1),camera_pose_to_base_footprint_(2))),
    	    currentTime,"camera_pose_frame", "base_footprint"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(p_q_b_(0),p_q_b_(1),p_q_b_(2),p_q_b_(3)), 
          tf::Vector3(base_footprint_to_base_link_(0),base_footprint_to_base_link_(1),base_footprint_to_base_link_(2))),
    	    currentTime,"base_footprint", "base_link"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(0, 0, 0, 1), 
          tf::Vector3(-0.281, -0.215, -0.020)),
    	    currentTime,"base_link", "br_wheel_link"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(0, 0, 0, 1), 
          tf::Vector3(-0.281, 0.215, -0.020)),
    	    currentTime,"base_link", "bl_wheel_link"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(0, 0, 0, 1), 
          tf::Vector3(0.281, -0.215, -0.020)),
    	    currentTime,"base_link", "fr_wheel_link"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(0, 0, 0, 1), 
          tf::Vector3(0.281, 0.215, -0.020)),
    	    currentTime,"base_link", "fl_wheel_link"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(0, 0, 0, 1), 
          tf::Vector3(0.0, 0.0, 0.0)),
    	    currentTime,"base_link", "imu_link"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0,0.0,M_PI_4)), 
          tf::Vector3(base_link_to_Velodyne1_(0),base_link_to_Velodyne1_(1),base_link_to_Velodyne1_(2))),
    	    currentTime,"base_link", "velodyne"));

      broadcaster.sendTransform(
        tf::StampedTransform(
          tf::Transform(tf::Quaternion(0.0,0.0,0.0,1.0),
          tf::Vector3(base_link_to_base_arm_(0),base_link_to_base_arm_(1),base_link_to_base_arm_(2))),
          currentTime,"base_link","base_arm"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, 0.0)), 
          tf::Vector3(0.42, 0.0, 0.09)),
    	    currentTime,"base_link", "base_sonar_front"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, M_PI)), 
          tf::Vector3(-0.41, 0.0, 0.09)),
    	    currentTime,"base_link", "base_sonar_rear"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, M_PI_2)), 
          tf::Vector3(0.0, 0.3, 0.09)),
    	    currentTime,"base_link", "base_sonar_left"
        )
      );

      broadcaster.sendTransform(
        tf::StampedTransform(
    	    tf::Transform(tf::Quaternion(tf::createQuaternionFromRPY(0.0, 0.0, -M_PI_2)), 
          tf::Vector3(0.0, -0.3, 0.09)),
    	    currentTime,"base_link", "base_sonar_right"
        )
      );

    }
      ros::spinOnce();
      loop_rate.sleep();

  } //End while (ros::ok())

  return 0;

}
