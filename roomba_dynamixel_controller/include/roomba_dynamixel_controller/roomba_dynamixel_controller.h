#ifndef __ROOMBA_DYNAMIXEL_CONTROLLER_H
#define __ROOMBA_DYNAMIXEL_CONTROLLER_H

#include "ros/ros.h"
#include "ros/time.h" 
#include "trajectory_msgs/JointTrajectory.h"
#include "dynamixel_angle_msgs/DynamixelAngle.h"
#include <math.h>

class Dynamixel{
public:
    Dynamixel();
    void process();

private:
    void angle_callback(const dynamixel_angle_msgs::DynamixelAngle::ConstPtr& msg);
    void set_parameter(double angle);
    void normalize(double& angle);
    void offset_process(double& angle);

    float target_angle;     // 目標の角度
    double offset_angle;    // オフセットの角度
    double execution_time;  // 実行時間
    std::string dynamixel_name;

    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    ros::Publisher joint_pub;
    ros::Subscriber angle_sub;

    trajectory_msgs::JointTrajectory jt;
};

#endif // __ROOMBA_DYNAMIXEL_CONTROLLER_H