#include <ros/ros.h>
#include <iostream>
#include <math.h>

#include "dynamixel_angle_msgs/DynamixelAngle.h"

bool is_number(const std::string& str)
{
    std::string sub_str = str;
    if(sub_str.substr(0,1) == "-") sub_str.erase(0,1);
    for(char const &c : sub_str){
        if(std::isdigit(c) == 0) return false;
        else return true;
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"teleop_angle");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<dynamixel_angle_msgs::DynamixelAngle>("/angle",1);
 
    while(1){
        std::cout << "Input an angle [rad] (-M_PI ~ M_PI)" << std::endl;
        
        std::string input;
        std::cin >> input;
        if(is_number(input)){
            dynamixel_angle_msgs::DynamixelAngle angle;
            angle.theta = std::stof(input);
            pub.publish(angle);
            ROS_INFO_STREAM("Success to publish angle");        
        }else{
            if(input == "q") break;
            ROS_ERROR("The input value is invalid");
        }
    }

    return 0;
}