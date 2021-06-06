#ifndef __ROOMBA_DYNAMIXEL_CONTROLLER_H
#define __ROOMBA_DYNAMIXEL_CONTROLLER_H

#include <ros/ros.h>
#include <ros/time.h> 
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>

#include "dynamixel_angle_msgs/DynamixelAngle.h"

class RoombaDynamixelController{
public:
    RoombaDynamixelController();
    void process();

private:
    void angle_callback(const dynamixel_angle_msgs::DynamixelAngle::ConstPtr& msg);
    void jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void set_parameter(double angle);
    void normalize(double& angle);
    void offset_process(double& angle);

    // parameter
    float target_angle;     // target angle
    double offset_angle;    // offset angle(no use)
    double execution_time;  // execution time
    
    // dynamixel parameter
    std::string dynamixel_name;
    std::string base_link_frame_id;
    std::string dynamixel_frame_id;

    // tf
    tf2_ros::TransformBroadcaster broadcaster;
    double dynamixel_x;
    double dynamixel_y;
    double dynamixel_z;
    double dynamixel_roll;
    double dynamixel_pitch;

    // ros
    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    ros::Publisher joint_pub;
    ros::Subscriber angle_sub;
    ros::Subscriber joint_sub;
    trajectory_msgs::JointTrajectory jt;
};

#endif // __ROOMBA_DYNAMIXEL_CONTROLLER_H