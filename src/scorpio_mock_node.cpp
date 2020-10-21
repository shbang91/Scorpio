//
// Created by mfawn on 7/6/20.
//

#include <ros/ros.h>
#include "geometry_msgs/Point.h"

void OnRosMsg(geometry_msgs::Point p) {
    //Just mock
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scorpio_control_node");
    ros::NodeHandle n("scorpio");
    ros::Subscriber point_sub = n.subscribe< geometry_msgs::Point>("moveAbsolute", 1000, OnRosMsg);
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}