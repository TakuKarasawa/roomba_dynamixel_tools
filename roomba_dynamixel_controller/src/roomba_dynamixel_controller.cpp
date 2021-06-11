#include "roomba_dynamixel_controller/roomba_dynamixel_controller.h"

RoombaDynamixelController::RoombaDynamixelController() : private_nh("~")
{
    private_nh.param("offset_angle",offset_angle,{0.0});
    private_nh.param("execution_time",execution_time,{1.0});
    private_nh.param("dynamixel_name",dynamixel_name,{"dynamixel"});
    private_nh.param("dynamixel_frame_id",dynamixel_frame_id,{"dynamixel"});
    private_nh.param("base_link_frame_id",base_link_frame_id,{"base_link"});
    private_nh.param("dynamixel_x",dynamixel_x,{0.0});
    private_nh.param("dynamixel_y",dynamixel_y,{0.0});
    private_nh.param("dynamixel_z",dynamixel_z,{0.663});
    private_nh.param("dynamixel_roll",dynamixel_roll,{0.0});
    private_nh.param("dynamixel_pitch",dynamixel_pitch,{0.0});

    joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",1);
    angle_sub = nh.subscribe("/angle",10,&RoombaDynamixelController::angle_callback,this);
    joint_sub = nh.subscribe("/dynamixel_workbench/joint_states",10,&RoombaDynamixelController::jointstate_callback,this);

    jt.points.resize(1);
    jt.joint_names.resize(1);
    jt.joint_names[0] = dynamixel_name;
    jt.points[0].positions.resize(2);
}

void RoombaDynamixelController::angle_callback(const dynamixel_angle_msgs::DynamixelAngle::ConstPtr& msg)
{
    ROS_INFO("has received angle!");
    target_angle = msg->theta;
    set_parameter(target_angle);
}

void RoombaDynamixelController::jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // tf
    geometry_msgs::TransformStamped dynamixel_pose;
    dynamixel_pose.header.stamp = ros::Time::now();
    dynamixel_pose.header.frame_id = base_link_frame_id;
    dynamixel_pose.child_frame_id = dynamixel_frame_id;
    dynamixel_pose.transform.translation.x = dynamixel_x;
    dynamixel_pose.transform.translation.y = dynamixel_y;
    dynamixel_pose.transform.translation.z = dynamixel_z;

    tf2::Quaternion q;
    q.setRPY(dynamixel_roll,dynamixel_pitch,msg->position[0]);
    dynamixel_pose.transform.rotation.x = q.x();
    dynamixel_pose.transform.rotation.y = q.y();
    dynamixel_pose.transform.rotation.z = q.z();
    dynamixel_pose.transform.rotation.w = q.w();
    broadcaster.sendTransform(dynamixel_pose);
}

void RoombaDynamixelController::set_parameter(double angle=0.0)
{
    normalize(angle);
    //offset_process(angle);    // no use

    jt.points[0].positions[0] = angle;
    jt.points[0].time_from_start = ros::Duration(execution_time);
    joint_pub.publish(jt);
}

void RoombaDynamixelController::normalize(double& angle)
{
    while(angle > M_PI || angle <= -M_PI){
        if(angle > M_PI) angle -= 2*M_PI;
        if(angle < -M_PI) angle += 2*M_PI;
    }
}

void RoombaDynamixelController::offset_process(double& angle)
{
    if(offset_angle > 0){
        if(angle > M_PI - offset_angle) angle += offset_angle - 2*M_PI;
        else angle += offset_angle;
    }
    else if(offset_angle < 0){
        if(angle < -M_PI - offset_angle) angle = offset_angle + 2*M_PI;
        else angle += offset_angle;
    }
}

void RoombaDynamixelController::process(){ ros::spin();}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"roomba_dynamixel_controller");
    RoombaDynamixelController controller;
    controller.process();
    return 0;
}
