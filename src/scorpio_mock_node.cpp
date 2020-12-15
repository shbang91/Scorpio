//
// Created by mfawn on 7/6/20.
//

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
// #include "control_msgs/GripperCommandAction.h"
#include <gazebo_scorpio_plugin/GripperCommandSrv.h>
#include <gazebo_scorpio_plugin/MoveEndEffectorToSrv.h>
#include <actionlib/client/simple_action_client.h>

void OnRosMsg(geometry_msgs::Pose p) {
    //Just mock
}

bool OnRosSrv(gazebo_scorpio_plugin::MoveEndEffectorToSrv::Request &req,
              gazebo_scorpio_plugin::MoveEndEffectorToSrv::Response &res) {
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scorpio_control_node");
    ros::NodeHandle n("scorpio");
    ros::ServiceServer endeff_server = n.advertiseService("MoveEndEffectorToSrv", OnRosSrv);
    ros::Subscriber point_sub = n.subscribe< geometry_msgs::Pose>("MoveEndEffectorTo", 1000, OnRosMsg);
    ros::ServiceServer gripper_server = n.advertiseService("GripperCommandSrv", OnRosSrv);
    // actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client("gripper_control", true);
    // ros::Subscriber point_sub = n.subscribe<>("Grasp", 1000);
    ros::Publisher endeff_pub = n.advertise< geometry_msgs::Pose>("endeff_pos", 1000);
    ros::Publisher joint_pub = n.advertise< sensor_msgs::JointState>("joint_pos", 1000);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
