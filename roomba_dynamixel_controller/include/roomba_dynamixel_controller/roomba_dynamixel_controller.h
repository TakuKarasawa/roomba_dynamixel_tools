#ifndef ROOMBA_DYNAMIXEL_CONTROLLER_H_
#define ROOMBA_DYNAMIXEL_CONTROLLER_H_

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
    float target_angle_;     // target angle
    double offset_angle_;    // offset angle(no use)
    double execution_time_;  // execution time
    
    // dynamixel parameter
    std::string dynamixel_name_;
    std::string base_link_frame_id_;
    std::string dynamixel_frame_id_;

    // tf
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    double dynamixel_x_;
    double dynamixel_y_;
    double dynamixel_z_;
    double dynamixel_roll_;
    double dynamixel_pitch_;

    // ros
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Publisher joint_pub_;
    ros::Subscriber angle_sub_;
    ros::Subscriber joint_sub_;
    trajectory_msgs::JointTrajectory jt_;
};

#endif // ROOMBA_DYNAMIXEL_CONTROLLER_H_