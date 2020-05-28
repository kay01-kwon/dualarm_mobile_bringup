#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <vehicle_control/as5047Msg.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define _USE_MATH_DEFINES

using vehicle_control::as5047Msg;
using nav_msgs::Odometry;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using Eigen::Vector3d;

using std::cout;
using std::endl;

int rate;

std::string topic_name;

tf::StampedTransform transform;

nav_msgs::Odometry odom;

ros::Time currentTime;

class attitude_estimator{

    public:

    // Member functions
    void InitiateVariables();
    void EncoderSubscriberSetting();
    void PublisherSetting();
    void EncoderCallbackFunc(const as5047Msg & mag_enc);
    void WheelPosCalculation();
    void NormalVectorCalculation();
    void RollPitchCalculation();
    void PlatformCenterCalculation();
    void RelativeInfo();

    MatrixXd Rotation_x(double &phi);
    MatrixXd Rotation_y(double &theta);

    // Variables to use in the cpp file

    // Encoder data recieved --> is_subscription is switched to true
    bool is_subscription = false;
    
    // Translation info.
    Vector3d odom_to_camera_odom;
    Vector3d camera_pose_to_base_footprint;
    Vector3d base_footprint_to_base_link;
    Vector3d base_link_to_Velodyne1;
    Vector3d base_link_to_Velodyne2;
    Vector3d base_link_to_base_arm;

    // roll_pitch(0) : roll, roll_pitch(1) : pitch
    VectorXd roll_pitch = VectorXd(2);

    // p_q_b : quaternion from base(ground) to platform center ref frame
    // b_q_p : quaternion from platform center ref frame to base
    VectorXd p_q_b = VectorXd(4);
    VectorXd b_q_p = VectorXd(4);

    private:

    const int num_joint = 4;

    // Link info
    const double l_1 = 0.010;
    const double l_2 = 0.191;

    // Wheel info
    const double l_sep_x = 0.140;
    const double l_sep_y = 0.310825;
    const double wheel_radious = 0.076;

    // data conversion info
    const double bit_to_deg = (double) 360.0/16363.0;
    const double deg_to_rad = (double) M_PI/180.0;

    // Joints position
    MatrixXd joint_pos = MatrixXd(3,num_joint);

    // Joint angle info
    MatrixXd direction_sign = MatrixXd(num_joint,num_joint);
    VectorXd joint_angle_init = VectorXd(num_joint);
    VectorXd joint_angle_curr = VectorXd(num_joint);
    VectorXd joint_angle_rotation = VectorXd(num_joint);

    MatrixXd link_wheel_center_pos = MatrixXd(3,num_joint);
    MatrixXd p_wheel_center_pos = MatrixXd(3,num_joint);

    // wheel gnd position w.r.t platform center frame aligned to the ground
    MatrixXd b_p_wheel_gnd_pos = MatrixXd(3,num_joint);

    // p_z_b : z unit vector of base_footprint w.r.t platform center
    // b_z_p : z unit vector of platform center w.r.t base footprint    
    Vector3d p_z_b;
    Vector3d b_z_p;
    Vector3d p_z_p;

    // Rotation transformation matrix
    // p_R_b : base(ground) frame to platform center ref frame
    // b_R_p : platform center ref frame to base frame 
    MatrixXd p_R_b = MatrixXd(3,3);
    MatrixXd b_R_p = MatrixXd(3,3);

    // b_p_p : base_footprint location w.r.t platform center aligned to the ground
    Vector3d b_p_p;

    // p_p_target : target location w.r.t platform center ref frame
    Vector3d p_p_camera_sensor;
    Vector3d p_p_Velodyne1;
    Vector3d p_p_Velodyne2;

    ros::NodeHandle nh;
    ros::Publisher BaseLinkPosePublisher;
    ros::Subscriber Encoder_subscriber;

};

void attitude_estimator::InitiateVariables()
{
    joint_pos << l_sep_x/2, l_sep_x/2, -l_sep_x/2, -l_sep_x/2,
                l_sep_y/2, -l_sep_y/2, -l_sep_y/2, l_sep_y/2,
                0, 0, 0, 0;

    direction_sign << 1,0,0,0,
                    0,-1,0,0,
                    0,0,-1,0,
                    0,0,0,1;

    joint_angle_init <<306.2234, 80.5273,
                    258.8940, 107.3574;
    
    link_wheel_center_pos << -l_1, -l_1, l_1, l_1,
                            0, 0, 0, 0,
                            -l_2, -l_2, -l_2, -l_2;

    p_wheel_center_pos.fill(0.0);

    p_z_p << 0,0,1;

    base_link_to_base_arm<< 0, 0, 0.520;

    p_p_camera_sensor<< 0.290 + 0.00595, 0, 0.08290 + 0.39840;

    p_p_Velodyne1<< 0.240, 0.235, 0.28540 + 0.03603;

    p_p_Velodyne2 << -0.240, -0.235, 0.28540 + 0.03603;
    
}

void attitude_estimator::EncoderSubscriberSetting()
{
    Encoder_subscriber = nh.subscribe("/magEnc",1,&attitude_estimator::EncoderCallbackFunc,this);
}

void attitude_estimator::PublisherSetting()
{
    BaseLinkPosePublisher = nh.advertise<Odometry>("/base_arm_pose",1);
}

void attitude_estimator::EncoderCallbackFunc(const as5047Msg &mag_enc)
{    
    if(is_subscription == false)
        is_subscription = true;
    // Encoder Data Acquisition
    for(int i = 0; i < num_joint;++i)
    {
        joint_angle_curr(i) = (double) mag_enc.mag_enc[i] * bit_to_deg;
    }

    joint_angle_rotation = deg_to_rad*direction_sign*(joint_angle_curr - joint_angle_init);
}

void attitude_estimator::WheelPosCalculation()
{
    for(int i = 0; i < num_joint; ++i)
    {
        p_wheel_center_pos.col(i) = Rotation_y(joint_angle_rotation(i))*link_wheel_center_pos.col(i);
    }

    p_wheel_center_pos += joint_pos;
}

void attitude_estimator::NormalVectorCalculation()
{
    MatrixXd w = MatrixXd(3,num_joint);

    Vector3d u_temp;
    Vector3d v_temp;
    Vector3d w_temp;

    u_temp.fill(0.0);
    v_temp.fill(0.0);
    w_temp.fill(0.0);

    p_z_b.fill(0.0);

    for(int i = 0; i < num_joint; ++i)
    {
        int j,k;
        j = (i+2)%num_joint;
        k = (i+1)%num_joint;

        u_temp = p_wheel_center_pos.col(j) - p_wheel_center_pos.col(i);
        v_temp = p_wheel_center_pos.col(k) - p_wheel_center_pos.col(i);

        w_temp = u_temp.cross(v_temp);
        w.col(i) = w_temp/sqrt(w_temp.dot(w_temp));

        p_z_b += w.col(i);

        u_temp.fill(0.0);
        v_temp.fill(0.0);
        w_temp.fill(0.0);
    }

    // Average unit vector
    p_z_b /= num_joint;
    p_z_b.normalize();

}

void attitude_estimator::RollPitchCalculation()
{
    // roll_pitch(0) : roll
    // roll_pitch(1) : pitch
    roll_pitch(1) = asin(p_z_b(0));

    double Y = -p_z_b(1)/cos(roll_pitch(1));
    double Z = p_z_b(2)/cos(roll_pitch(1));

    roll_pitch(0) = atan2(Y,Z);
}

void attitude_estimator::PlatformCenterCalculation()
{
    // Grounded wheel position w.r.t platform center
    MatrixXd p_wheel_ground = MatrixXd(3,num_joint);
    Vector3d temp_vec = Vector3d(3);
    MatrixXd shift = MatrixXd(3,num_joint);

    shift<<0, 0, 0, 0,
            0, 0, 0, 0,
            -wheel_radious, -wheel_radious, -wheel_radious, -wheel_radious;

    p_R_b = Rotation_x(roll_pitch(0))*Rotation_y(roll_pitch(1));
    b_R_p = p_R_b.transpose();

    b_p_wheel_gnd_pos = b_R_p * p_wheel_center_pos + shift;

    // Initialize the current 
    b_p_p.fill(0.0);

    for(int i = 0; i < num_joint; ++i)
    {
        temp_vec = b_p_wheel_gnd_pos.col(i);
        b_p_p -= temp_vec;
    }

    b_p_p /= num_joint;
}

void attitude_estimator::RelativeInfo()
{
    WheelPosCalculation();
    NormalVectorCalculation();
    RollPitchCalculation();
    PlatformCenterCalculation();

    odom_to_camera_odom = b_p_p + b_R_p * p_p_camera_sensor;
    camera_pose_to_base_footprint = -p_p_camera_sensor + p_R_b * b_p_p;
    base_footprint_to_base_link = b_p_p;
    base_link_to_Velodyne1 = p_p_Velodyne1;
    base_link_to_Velodyne2 = p_p_Velodyne2;

    double phi;
    double theta;

    double p_qx_b;
    double p_qy_b;
    double p_qz_b;
    double p_qw_b;
    
    double b_qx_p;
    double b_qy_p;
    double b_qz_p;
    double b_qw_p;

    phi = roll_pitch(0);
    theta = roll_pitch(1);

    p_qx_b = cos(-theta/2)*sin(-phi/2);
    p_qy_b = sin(-theta/2)*cos(-phi/2);
    p_qz_b = -sin(-theta/2)*sin(-phi/2);
    p_qw_b = cos(-theta/2)*cos(-phi/2);

    b_qx_p = cos(theta/2)*sin(phi/2);
    b_qy_p = sin(theta/2)*cos(phi/2);
    b_qz_p = sin(theta/2)*sin(phi/2);
    b_qw_p = cos(theta/2)*cos(phi/2);

    p_q_b.fill(0.0);
    b_q_p.fill(0.0);

    p_q_b << p_qx_b, p_qy_b, p_qz_b, p_qw_b;
    b_q_p << b_qx_p, b_qy_p, b_qz_p, b_qw_p;
    
    cout<<b_q_p<<endl;
    cout<<endl;
}

MatrixXd attitude_estimator::Rotation_x(double &phi)
{
    MatrixXd R_phi = MatrixXd(3,3);

    R_phi << 1, 0, 0,
            0, cos(phi), -sin(phi),
            0, sin(phi), cos(phi);
    return R_phi;
}

MatrixXd attitude_estimator::Rotation_y(double &theta)
{
    MatrixXd R_theta = MatrixXd(3,3);

    R_theta << cos(theta), 0 ,sin(theta),
            0, 1, 0,
            -sin(theta), 0, cos(theta);

    return R_theta;
}