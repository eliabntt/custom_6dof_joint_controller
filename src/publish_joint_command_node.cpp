//
// Created by ebonetto on 10.02.22.
//
#include <ros/ros.h>
#include "custom_joint_controller_ros/publish_joint_commands.h"


int main(int argc, char **argv) {
	ros::init(argc, argv, "custom_joint_controller_irotate_ros");

	auto *nh = new ros::NodeHandle();
	auto *nh_prv = new ros::NodeHandle("~");
	int pub_rate;
	nh_prv->getParam("publish_rate", pub_rate);

	publish_joint_commands::CustomJointController custom_joint_controller;
	custom_joint_controller.init(*nh, *nh_prv);


	const int loop_freq = pub_rate;
	ros::Rate loop_rate(static_cast<double>(loop_freq));

	while(ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}
}