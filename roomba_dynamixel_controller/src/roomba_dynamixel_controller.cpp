#include "roomba_dynamixel_controller/roomba_dynamixel_controller.h"

Dynamixel::Dynamixel() : private_nh("~")
{
    private_nh.param("wait_time",wait_time,{10.0});
    private_nh.param("offset_angle",offset_angle,{0.0});

    joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",1);
    angle_sub = nh.subscribe("/angle",1,&Dynamixel::angle_callback,this);

    jt.header.frame_id = "base_motor";
    jt.points.resize(1);
    
    jt.joint_names.resize(1);
    jt.joint_names[0] ="dynamixel";

    jt.points[0].positions.resize(2);

}

void Dynamixel::angle_callback(const dynamixel_angle_msgs::DynamixelAngle::ConstPtr& msg)
{   
    target_angle = msg->theta;
    angle_received = true;
    ROS_INFO("has received angle!");
}

void Dynamixel::set_parameter(float angle=0.0)
{
    normalize(angle);

    if(angle > M_PI - offset_angle) angle = -(2*M_PI - offset_angle - angle);
    else angle += offset_angle ;

    jt.points[0].positions[0] = angle;
    jt.points[0].time_from_start = ros::Duration(1.0);
    joint_pub.publish(jt);
}

void Dynamixel::rotation(float angle)
{
    static ros::Time begin = ros::Time::now();
    static int step = 0;

    ros::Duration diff(0,0);

    ros::Rate rate(10);
    while(diff < ros::Duration(1)){
        jt.header.stamp = ros::Time::now();
        set_parameter(angle);
        ros::spinOnce();

        if(step++ == 0) begin = ros::Time::now();
        diff = ros::Time::now() - begin;

        rate.sleep();
        ROS_INFO("Joint(%s)= %f", jt.joint_names[0].c_str(),jt.points[0].positions[0]);
    }
}

void Dynamixel::normalize(float& angle)
{
    while(angle <= M_PI && angle >= -M_PI){
        if(angle > M_PI) angle -= 2*M_PI;
        if(angle < -M_PI) angle += 2*M_PI; 
    }
}

void Dynamixel::process()
{
    ROS_INFO("start process!");

    while(ros::ok()){
        ros::spinOnce();
        if(angle_received){
            ros::Duration(1.0).sleep();
            rotation(0.0);
            ros::Duration(1.0).sleep();
            if(record_angle != target_angle){ 
                record_angle = target_angle;
                rotation(target_angle);
                ros::Duration(wait_time).sleep();
            }
            else ros::Duration(1.0).sleep();
        }
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"roomba_dynamixel_controller");
    Dynamixel dynamixel;
    dynamixel.process();

    return 0;
}