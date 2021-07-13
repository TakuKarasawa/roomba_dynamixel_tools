#include "test_dynamixel/test_dynamixel.h"

Test_Dynamixel::Test_Dynamixel() : private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("target_angle",target_angle,{M_PI/6});
    private_nh.param("dynamixel_x",dynamixel_x,{0.0});
    private_nh.param("dynamixel_y",dynamixel_y,{0.0});
    private_nh.param("dynamixel_z",dynamixel_z,{0.663});
    private_nh.param("dynamixel_roll",dynamixel_roll,{0.0});
    private_nh.param("dynamixel_pitch",dynamixel_pitch,{0.0});
    private_nh.param("dynamixel_name",dynamixel_name,{"dynamixel"});
    private_nh.param("dynamixel_frame_id",dynamixel_frame_id,{"dynamixel"});
    private_nh.param("base_link_frame_id",base_link_frame_id,{"base_link"});
    private_nh.param("mode",mode,{"reader"});

    joint_sub = nh.subscribe("/dynamixel_workbench/joint_states",10,&Test_Dynamixel::jointstate_callback,this);
    joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/dynamixel_workbench/joint_trajectory",1);

    jt.header.frame_id = dynamixel_frame_id;
    jt.points.resize(1);
    jt.joint_names.resize(1);
    jt.joint_names[0] = dynamixel_name;
    jt.points[0].positions.resize(2);
    jt.points[0].positions[0] = 0.0;

    std::cout << "----- Dynamixel Parameters -----" << std::endl;
    std::cout << "hz: " << hz << std::endl;
    std::cout << "target_angle: " << target_angle << std::endl;
    std::cout << "dynamixel_name: " << dynamixel_name << std::endl;
    std::cout << "mode: " << mode << std::endl;
    std::cout << std::endl;
}

void Test_Dynamixel::jointstate_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_name.data = msg->name[0];
    joint_pos.data = msg->position[0];
    joint_vel.data = msg->velocity[0];
    joint_eff.data = msg->effort[0];

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

void Test_Dynamixel::set_angle(double angle=0.0)
{
    while(angle > M_PI || angle < -M_PI){
        if(angle > M_PI) angle -= 2*M_PI;
        if(angle < -M_PI) angle += 2*M_PI;
    }

    jt.points[0].positions[0] = angle;
    jt.points[0].time_from_start = ros::Duration(1.0);
    joint_pub.publish(jt);
}

void Test_Dynamixel::rotation(double angle=0.0)
{
    static ros::Time begin = ros::Time::now();
    static int step = 0;

    ros::Duration diff(0,0);

    ros::Rate loop_rate(hz);
    while(diff < ros::Duration(1.0)){
        jt.header.stamp = ros::Time::now();
        set_angle(angle);
        ros::spinOnce();
        if(step++ == 0) begin = ros::Time::now();
        diff = ros::Time::now() - begin;
        loop_rate.sleep();
        ROS_INFO("Joint[%s]: %f", joint_name.data.c_str(), jt.points[0].positions[0]);
    }

}

void Test_Dynamixel::dynamixel_reader()
{
    ROS_INFO("start dynamixel reader");
    ROS_INFO("Joint[name]: (position,velocity,effort)");
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO("Joint[%s]: (%f,%f,%f)", joint_name.data.c_str(), joint_pos.data, joint_vel.data, joint_eff.data);
    }
}


void Test_Dynamixel::dynamixel_writer()
{
    ROS_INFO("start dynamixel writer");
    ROS_INFO("Joint[name]: position");
    rotation();
    ros::Duration(1).sleep();
    rotation(target_angle);
}

void Test_Dynamixel::process()
{
    ROS_INFO("start process!");
    if(mode == "reader") dynamixel_reader();
    else if(mode == "writer") dynamixel_writer();
    else ROS_ERROR("Invalid mode!");
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"test_dynamixel");
    Test_Dynamixel test_dynamixel;
    test_dynamixel.process();
    return 0;
}
