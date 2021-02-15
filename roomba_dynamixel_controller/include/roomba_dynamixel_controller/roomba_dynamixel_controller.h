#ifndef __ROOMBA_DYNAMIXEL_CONTROLLER_H
#define __ROOMBA_DYNAMIXEL_CONTROLLER_H

#include "ros/ros.h"
#include "ros/time.h" 
#include "trajectory_msgs/JointTrajectory.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "dynamixel_angle_msgs/DynamixelAngle.h"
#include <math.h>

class Dynamixel{
public:
    Dynamixel();
    void process();

private:
    void angle_callback(const dynamixel_angle_msgs::DynamixelAngle::ConstPtr& msg);
    void set_parameter(double angle);
    void rotation(double angle);

    float target_angle;
    float record_angle;
    double wait_time;
    double Hz;
    bool angle_received = false;

    ros::NodeHandle nh;
    ros::Publisher joint_pub;
    ros::Subscriber angle_sub;

    //tf::TransformBroadcaster broadcaster;
    //tf::TransformListener listener;

    trajectory_msgs::JointTrajectory jt;
};

#endif // __ROOMBA_DYNAMIXEL_CONTROLLER_H