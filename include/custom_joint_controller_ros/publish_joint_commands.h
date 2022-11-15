//
// Created by ebonetto on 10.02.22.
//

#ifndef SRC_PUBLISH_JOINT_COMMANDS_H
#define SRC_PUBLISH_JOINT_COMMANDS_H

#pragma once

#include <algorithm>
#include <memory>
#include <math.h>
#include <vector>
#include <string>
#include <angles/angles.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <ros/subscribe_options.h>

#include <controller_interface/controller.h>

#include <hardware_interface/joint_state_interface.h>

#include <joint_state_controller/joint_state_controller.h>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <control_toolbox/pid.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include "tf_conversions/tf_eigen.h"
#include <eigen_conversions/eigen_msg.h>

namespace publish_joint_commands {
		class CustomJointController {

		public:
				struct Commands
				{
						double position_; // Last commanded position
						double velocity_; // Last commanded velocity
						bool has_velocity_; // false if no velocity command has been specified
				};

			CustomJointController() : publish_rate_(100.0) {}
			~CustomJointController() {}

			bool init(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);
			void starting(const ros::Time &time,  const std::map<std::string, double>& position_setpoints);
			void stopping(const ros::Time &time);
			void update(const ros::Time &time, const ros::Duration &period, const std::map<std::string, std::vector<double>> &current_state);

			std::map<std::string, realtime_tools::RealtimeBuffer<Commands>> commands_;
			std::map<std::string, Commands> commands_struct_;

		private:
				std::map<std::string, double> max_position_;
				std::map<std::string, double> max_velocity_;

				std::vector<std::string> joint_names_;
				std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> joint_state_publisher_;

				ros::Time last_publish_time_;

				double publish_rate_;

				std::map<std::string, control_toolbox::Pid> joint_pid_controllers_;

				double p_gain_roll, i_gain_roll, d_gain_roll, i_max_roll, i_min_roll;
				double p_gain_pitch, i_gain_pitch, d_gain_pitch, i_max_pitch, i_min_pitch;
				double p_gain_yaw, i_gain_yaw, d_gain_yaw, i_max_yaw, i_min_yaw;
				double p_gain_x, i_gain_x, d_gain_x, i_max_x, i_min_x;
				double p_gain_y, i_gain_y, d_gain_y, i_max_y, i_min_y;
				double p_gain_z, i_gain_z, d_gain_z, i_max_z, i_min_z;
				bool anti_windup_roll, anti_windup_pitch, anti_windup_yaw, anti_windup_x, anti_windup_y, anti_windup_z;

				Eigen::Quaterniond quaternion_odom;

				unsigned int n_joints_;

				void enforceLimits(std::string name, double &position);
				void setCommand(const std::map<std::string, std::vector<double>> &setpoints);
				void limitVelocity(const std::string& name, double &velocity);

				void getJointStates(const sensor_msgs::JointState::ConstPtr& msg);

				void getOdom(const nav_msgs::Odometry::ConstPtr& msg);
				void getCurrentSetpoint(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
				void getDesiredSpeed(const geometry_msgs::Twist::ConstPtr &msg);
				bool first_commad_ = false;
				bool ignore_position = false;
				void pubDrone(ros::Time time);
				void pubIRotate(ros::Time time);
				enum robot_type{
						DRONE, IROTATE
				};
				int robot_id;
				ros::Subscriber sub_joint_states,sub_joint_states2, sub_nmpc_goal, sub_cmd_vel_goal;
		};
}


#endif //SRC_PUBLISH_JOINT_COMMANDS_H
