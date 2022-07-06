#include "roomba_dynamixel_controller/roomba_dynamixel_controller.h"

RoombaDynamixelController::RoombaDynamixelController() :
    private_nh_("~"),
    target_angle_(0.0)
{
    private_nh_.param("IS_TF_PUBLISH",IS_TF_PUBLISH_,{true});
    private_nh_.param("DYNAMIXEL_NAME",DYNAMIXEL_NAME_,{std::string("dynamixel")});
    private_nh_.param("DYNAMIXEL_FRAME_ID",DYNAMIXEL_FRAME_ID_,{std::string("dynamixel")});
    private_nh_.param("ROBOT_FRAME_ID",ROBOT_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("OFFSET_ANGLE",OFFSET_ANGLE_,{0.0});
    private_nh_.param("EXECUTION_TIME",EXECUTION_TIME_,{1.0});
    private_nh_.param("DYNAMIXEL_X",DYNAMIXEL_X_,{0.0});
    private_nh_.param("DYNAMIXEL_Y",DYNAMIXEL_Y_,{0.0});
    private_nh_.param("DYNAMIXEL_Z",DYNAMIXEL_Z_,{0.663});
    private_nh_.param("DYNAMIXEL_ROLL",DYNAMIXEL_ROLL_,{0.0});
    private_nh_.param("DYNAMIXEL_PITCH",DYNAMIXEL_PITCH_,{0.0});

    joint_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",1);
    angle_sub_ = nh_.subscribe("/angle",10,&RoombaDynamixelController::angle_callback,this);
    joint_sub_ = nh_.subscribe("/dynamixel_workbench/joint_states",10,&RoombaDynamixelController::jointstate_callback,this);

    broadcaster_.reset(new tf2_ros::TransformBroadcaster);

    init_jt_msg(jt_);
}

void RoombaDynamixelController::angle_callback(const dynamixel_angle_msgs::DynamixelAngle::ConstPtr& msg)
{
    ROS_INFO("has received angle!");
    target_angle_ = msg->theta;
    set_parameter(target_angle_);
}

void RoombaDynamixelController::init_jt_msg(trajectory_msgs::JointTrajectory& jt)
{
    jt.points.resize(1);
    jt.joint_names.resize(1);
    jt.joint_names[0] = DYNAMIXEL_NAME_;
    jt.points[0].positions.resize(2);
}

void RoombaDynamixelController::jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(IS_TF_PUBLISH_){
        // tf
        geometry_msgs::TransformStamped dynamixel_pose;
        dynamixel_pose.header.stamp = ros::Time::now();
        dynamixel_pose.header.frame_id = ROBOT_FRAME_ID_;
        dynamixel_pose.child_frame_id = DYNAMIXEL_FRAME_ID_;
        dynamixel_pose.transform.translation.x = DYNAMIXEL_X_;
        dynamixel_pose.transform.translation.y = DYNAMIXEL_Y_;
        dynamixel_pose.transform.translation.z = DYNAMIXEL_Z_;

        tf2::Quaternion q;
        q.setRPY(DYNAMIXEL_ROLL_,DYNAMIXEL_PITCH_,msg->position[0]);
        dynamixel_pose.transform.rotation.x = q.x();
        dynamixel_pose.transform.rotation.y = q.y();
        dynamixel_pose.transform.rotation.z = q.z();
        dynamixel_pose.transform.rotation.w = q.w();
        broadcaster_->sendTransform(dynamixel_pose);
    }
    else return;
}

void RoombaDynamixelController::set_parameter(double angle = 0.0)
{
    normalize(angle);

    jt_.points[0].positions[0] = angle;
    jt_.points[0].time_from_start = ros::Duration(EXECUTION_TIME_);
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
    if(OFFSET_ANGLE_ > 0){
        if(angle > M_PI - OFFSET_ANGLE_) angle += OFFSET_ANGLE_ - 2*M_PI;
        else angle += OFFSET_ANGLE_;
    }
    else if(OFFSET_ANGLE_ < 0){
        if(angle < -M_PI - OFFSET_ANGLE_) angle = OFFSET_ANGLE_ + 2*M_PI;
        else angle += OFFSET_ANGLE_;
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
