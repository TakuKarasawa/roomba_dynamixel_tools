#ifndef __TEST_DYNAMIXEL_H
#define __TEST_DYNAMIXEL_H

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
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

    ros::NodeHandle private_nh;
    ros::NodeHandle nh;
    ros::Subscriber joint_sub;
    ros::Publisher joint_pub;

    double hz;
    double target_angle;
    std::string dynamixel_name;
    std::string dynamixel_frame;
    std::string mode;

    trajectory_msgs::JointTrajectory jt;

    std_msgs::String joint_name; // 名前読み出し
    std_msgs::Float64 joint_pos; // ポジション読み出し
    std_msgs::Float64 joint_vel; // 速度読み出し
    std_msgs::Float64 joint_eff; // 負荷読み出し

};



#endif  // __TEST_DYNAMIXEL_H