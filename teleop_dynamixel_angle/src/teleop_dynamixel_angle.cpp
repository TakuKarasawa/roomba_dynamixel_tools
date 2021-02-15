#include "ros/ros.h"
#include "dynamixel_angle_msgs/DynamixelAngle.h"
#include <math.h>

int main(int argc,char **argv)
{
    ros::init(argc,argv,"teleop_angle");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<dynamixel_angle_msgs::DynamixelAngle>("/angle",1);
 
    while(true){
        std::cout << "Input an angle [rad] (-M_PI ~ M_PI)" << std::endl;
        float input_angle;
        std::cin >> input_angle;

        dynamixel_angle_msgs::DynamixelAngle angle;
        angle.theta = input_angle;

        pub.publish(angle);
        ROS_INFO_STREAM("Success to publish angle");
    }

    return 0;
}