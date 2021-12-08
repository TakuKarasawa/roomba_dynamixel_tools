#include "roomba_dynamixel_controller/roomba_dynamixel_controller.h"

RoombaDynamixelController::RoombaDynamixelController() : private_nh_("~")
{
    private_nh_.param("offset_angle",offset_angle_,{0.0});
    private_nh_.param("execution_time",execution_time_,{1.0});
    private_nh_.param("dynamixel_name",dynamixel_name_,{"dynamixel"});
    private_nh_.param("dynamixel_frame_id",dynamixel_frame_id_,{"dynamixel"});
    private_nh_.param("base_link_frame_id",base_link_frame_id_,{"base_link"});
    private_nh_.param("dynamixel_x",dynamixel_x_,{0.0});
    private_nh_.param("dynamixel_y",dynamixel_y_,{0.0});
    private_nh_.param("dynamixel_z",dynamixel_z_,{0.663});
    private_nh_.param("dynamixel_roll",dynamixel_roll_,{0.0});
    private_nh_.param("dynamixel_pitch",dynamixel_pitch_,{0.0});

    joint_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",1);
    angle_sub_ = nh_.subscribe("/angle",10,&RoombaDynamixelController::angle_callback,this);
    joint_sub_ = nh_.subscribe("/dynamixel_workbench/joint_states",10,&RoombaDynamixelController::jointstate_callback,this);

    broadcaster_.reset(new tf2_ros::TransformBroadcaster);

    jt_.points.resize(1);
    jt_.joint_names.resize(1);
    jt_.joint_names[0] = dynamixel_name_;
    jt_.points[0].positions.resize(2);
}

void RoombaDynamixelController::angle_callback(const dynamixel_angle_msgs::DynamixelAngle::ConstPtr& msg)
{
    ROS_INFO("has received angle!");
    target_angle_ = msg->theta;
    set_parameter(target_angle_);
}

void RoombaDynamixelController::jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // tf
    geometry_msgs::TransformStamped dynamixel_pose;
    dynamixel_pose.header.stamp = ros::Time::now();
    dynamixel_pose.header.frame_id = base_link_frame_id_;
    dynamixel_pose.child_frame_id = dynamixel_frame_id_;
    dynamixel_pose.transform.translation.x = dynamixel_x_;
    dynamixel_pose.transform.translation.y = dynamixel_y_;
    dynamixel_pose.transform.translation.z = dynamixel_z_;

    tf2::Quaternion q;
    q.setRPY(dynamixel_roll_,dynamixel_pitch_,msg->position[0]);
    dynamixel_pose.transform.rotation.x = q.x();
    dynamixel_pose.transform.rotation.y = q.y();
    dynamixel_pose.transform.rotation.z = q.z();
    dynamixel_pose.transform.rotation.w = q.w();
    broadcaster_->sendTransform(dynamixel_pose);
}

void RoombaDynamixelController::set_parameter(double angle = 0.0)
{
    normalize(angle);

    jt_.points[0].positions[0] = angle;
    jt_.points[0].time_from_start = ros::Duration(execution_time_);
    joint_pub_.publish(jt_);
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
    if(offset_angle_ > 0){
        if(angle > M_PI - offset_angle_) angle += offset_angle_ - 2*M_PI;
        else angle += offset_angle_;
    }
    else if(offset_angle_ < 0){
        if(angle < -M_PI - offset_angle_) angle = offset_angle_ + 2*M_PI;
        else angle += offset_angle_;
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
