#ifndef __TEST_DYNAMIXEL_H
#define __TEST_DYNAMIXEL_H

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

class Test_Dynamixel
{
public:
    Test_Dynamixel();
    void process();

private:
    void jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void set_angle(double angle);
    void rotation(double angle);
    void dynamixel_reader();
    void dynamixel_writer();

    // ros
    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    ros::Subscriber joint_sub;
    ros::Publisher joint_pub;
    trajectory_msgs::JointTrajectory jt;

    // parameter
    double hz;
    double target_angle;

    // hard parameter
    std::string dynamixel_name;
    std::string base_link_frame_id;     
    std::string dynamixel_frame_id;
    std::string mode;

    // tf
    tf2_ros::TransformBroadcaster broadcaster;
    double dynamixel_x;
    double dynamixel_y;
    double dynamixel_z;
    double dynamixel_roll;
    double dynamixel_pitch;

    // reader
    std_msgs::String joint_name; // name reader
    std_msgs::Float64 joint_pos; // position reader
    std_msgs::Float64 joint_vel; // velocity reader
    std_msgs::Float64 joint_eff; // effort reader
};

#endif  // __TEST_DYNAMIXEL_H