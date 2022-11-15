//
// Created by ebonetto on 10.02.22.
//

#include <tf/LinearMath/Matrix3x3.h>
#include "custom_joint_controller_ros/publish_joint_commands.h"
#define MATH_PI     3.1415926535897931
#define SIN_PI_3    sinf(MATH_PI/3)
#define COS_PI_3    cosf(MATH_PI/3)
#define R_WHEEL     4
#define R_BASE      17

using namespace publish_joint_commands;

bool CustomJointController::init(ros::NodeHandle &nh,
                                 ros::NodeHandle &nh_priv) {

	// Get p gains from parameter server for roll, pitch, yaw, x, y, z
	nh_priv.param("p_gain_roll", p_gain_roll, 0.0);
	nh_priv.param("p_gain_pitch", p_gain_pitch, 0.0);
	nh_priv.param("p_gain_yaw", p_gain_yaw, 0.0);
	nh_priv.param("p_gain_cameraholder", p_gain_cameraholder, 0.0);
	nh_priv.param("p_gain_x", p_gain_x, 0.0);
	nh_priv.param("p_gain_y", p_gain_y, 0.0);
	nh_priv.param("p_gain_z", p_gain_z, 0.0);

	// Get d gains from parameter server for roll, pitch, yaw, x, y, z
	nh_priv.param("d_gain_roll", d_gain_roll, 0.0);
	nh_priv.param("d_gain_pitch", d_gain_pitch, 0.0);
	nh_priv.param("d_gain_yaw", d_gain_yaw, 0.0);
	nh_priv.param("d_gain_cameraholder", d_gain_cameraholder, 0.0);
	nh_priv.param("d_gain_x", d_gain_x, 0.0);
	nh_priv.param("d_gain_y", d_gain_y, 0.0);
	nh_priv.param("d_gain_z", d_gain_z, 0.0);

	// Get i gains from parameter server for roll, pitch, yaw, x, y, z
	nh_priv.param("i_gain_roll", i_gain_roll, 0.0);
	nh_priv.param("i_gain_pitch", i_gain_pitch, 0.0);
	nh_priv.param("i_gain_yaw", i_gain_yaw, 0.0);
	nh_priv.param("i_gain_cameraholder", i_gain_cameraholder, 0.0);
	nh_priv.param("i_gain_x", i_gain_x, 0.0);
	nh_priv.param("i_gain_y", i_gain_y, 0.0);
	nh_priv.param("i_gain_z", i_gain_z, 0.0);

	// Get i_max from parameter server for roll, pitch, yaw, x, y, z
	nh_priv.param("i_max_roll", i_max_roll, 0.0);
	nh_priv.param("i_max_pitch", i_max_pitch, 0.0);
	nh_priv.param("i_max_yaw", i_max_yaw, 0.0);
	nh_priv.param("i_max_cameraholder", i_max_cameraholder, 0.0);
	nh_priv.param("i_max_x", i_max_x, 0.0);
	nh_priv.param("i_max_y", i_max_y, 0.0);
	nh_priv.param("i_max_z", i_max_z, 0.0);

	// Get i_min from parameter server for roll, pitch, yaw, x, y, z
	nh_priv.param("i_min_roll", i_min_roll, 0.0);
	nh_priv.param("i_min_pitch", i_min_pitch, 0.0);
	nh_priv.param("i_min_yaw", i_min_yaw, 0.0);
	nh_priv.param("i_min_cameraholder", i_min_cameraholder, 0.0);
	nh_priv.param("i_min_x", i_min_x, 0.0);
	nh_priv.param("i_min_y", i_min_y, 0.0);
	nh_priv.param("i_min_z", i_min_z, 0.0);

	// Get anti_windup from parameter server for roll, pitch, yaw, x, y, z
	nh_priv.param("anti_windup_roll", anti_windup_roll, false);
	nh_priv.param("anti_windup_pitch", anti_windup_pitch, false);
	nh_priv.param("anti_windup_yaw", anti_windup_yaw, false);
	nh_priv.param("anti_windup_cameraholder", anti_windup_cameraholder, false);
	nh_priv.param("anti_windup_x", anti_windup_x, false);
	nh_priv.param("anti_windup_y", anti_windup_y, false);
	nh_priv.param("anti_windup_z", anti_windup_z, false);

	// Get velocity_limit from parameter server for roll, pitch, yaw, x, y, z
	double velocity_limit_roll, velocity_limit_pitch, velocity_limit_yaw, velocity_limit_x, velocity_limit_y, velocity_limit_z, velocity_limit_cameraholder;
	nh_priv.param("velocity_limit_roll", velocity_limit_roll, 0.0);
	nh_priv.param("velocity_limit_pitch", velocity_limit_pitch, 0.0);
	nh_priv.param("velocity_limit_yaw", velocity_limit_yaw, 0.0);
	nh_priv.param("velocity_limit_cameraholder", velocity_limit_cameraholder, 0.0);
	nh_priv.param("velocity_limit_x", velocity_limit_x, 0.0);
	nh_priv.param("velocity_limit_y", velocity_limit_y, 0.0);
	nh_priv.param("velocity_limit_z", velocity_limit_z, 0.0);

	// Get position_limit from parameter server for roll, pitch, yaw, x, y, z
	double position_limit_roll, position_limit_pitch, position_limit_yaw, position_limit_cameraholder, position_limit_x, position_limit_y, position_limit_z;
	nh_priv.param("position_limit_roll", position_limit_roll, 0.0);
	nh_priv.param("position_limit_pitch", position_limit_pitch, 0.0);
	nh_priv.param("position_limit_yaw", position_limit_yaw, 0.0);
	nh_priv.param("position_limit_cameraholder", position_limit_cameraholder, 0.0);
	nh_priv.param("position_limit_x", position_limit_x, 0.0);
	nh_priv.param("position_limit_y", position_limit_y, 0.0);
	nh_priv.param("position_limit_z", position_limit_z, 0.0);

	XmlRpc::XmlRpcValue v;
	// Get joint names from the parameter server
	if (!nh_priv.param("joint_names", v, v)) {
		ROS_ERROR("No joint names found on parameter server");
		return false;
	} else {
		for (int i = 0; i < v.size(); i++) {
			std::string name = std::string(v[i]);
			joint_names_.emplace_back(name);
			double p_gain, i_gain, d_gain, i_max, i_min;
			bool anti_windup;

			if (name == "roll_joint") {
				p_gain = p_gain_roll;
				i_gain = i_gain_roll;
				d_gain = d_gain_roll;
				i_max = i_max_roll;
				i_min = i_min_roll;
				anti_windup = anti_windup_roll;
//				ROS_INFO_STREAM("UPDATING " << name << " in roll");
				max_velocity_["roll_joint"] = velocity_limit_roll;
				max_position_["roll_joint"] = position_limit_roll;
			} else if (name == "pitch_joint") {
				p_gain = p_gain_pitch;
				i_gain = i_gain_pitch;
				d_gain = d_gain_pitch;
				i_max = i_max_pitch;
				i_min = i_min_pitch;
				anti_windup = anti_windup_pitch;
//				ROS_INFO_STREAM("UPDATING " << name << " in pitch");
				max_velocity_["pitch_joint"] = velocity_limit_pitch;
				max_position_["pitch_joint"] = position_limit_pitch;
			} else if (name == "yaw_joint") {
				p_gain = p_gain_yaw;
				i_gain = i_gain_yaw;
				d_gain = d_gain_yaw;
				i_max = i_max_yaw;
				i_min = i_min_yaw;
				anti_windup = anti_windup_yaw;
//				ROS_INFO_STREAM("UPDATING " << name << " in yaw");
				max_velocity_["yaw_joint"] = velocity_limit_yaw;
				max_position_["yaw_joint"] = position_limit_yaw;
			}else if (name == "cameraholder_joint") {
				p_gain = p_gain_cameraholder;
				i_gain = i_gain_cameraholder;
				d_gain = d_gain_cameraholder;
				i_max = i_max_cameraholder;
				i_min = i_min_cameraholder;
				anti_windup = anti_windup_cameraholder;
				max_velocity_["cameraholder_joint"] = velocity_limit_cameraholder;
				max_position_["cameraholder_joint"] = position_limit_cameraholder;
				ROS_INFO_STREAM("UPDATING " << name << " in cameraholder");
				ROS_INFO_STREAM(p_gain_cameraholder << " " <<
					                i_gain_cameraholder << " " <<
					                d_gain_cameraholder << " " <<
						                i_max_cameraholder << " " <<
					                i_min_cameraholder << " " <<
					                anti_windup_cameraholder << " ");
			} else if (name == "x_joint") {
				p_gain = p_gain_x;
				i_gain = i_gain_x;
				d_gain = d_gain_x;
				i_max = i_max_x;
				i_min = i_min_x;
				anti_windup = anti_windup_x;
//				ROS_INFO_STREAM("UPDATING " << name << " in x");
				max_velocity_["x_joint"] = velocity_limit_x;
				max_position_["x_joint"] = position_limit_x;
			} else if (name == "y_joint") {
				p_gain = p_gain_y;
				i_gain = i_gain_y;
				d_gain = d_gain_y;
				i_max = i_max_y;
				i_min = i_min_y;
				anti_windup = anti_windup_y;
//				ROS_INFO_STREAM("UPDATING " << name << " in y");
				max_velocity_["y_joint"] = velocity_limit_y;
				max_position_["y_joint"] = position_limit_y;
			} else if (name == "z_joint") {
				p_gain = p_gain_z;
				i_gain = i_gain_z;
				d_gain = d_gain_z;
				i_max = i_max_z;
				i_min = i_min_z;
				anti_windup = anti_windup_z;
//				ROS_INFO_STREAM("UPDATING " << name << " in z");
				max_velocity_["z_joint"] = velocity_limit_z;
				max_position_["z_joint"] = position_limit_z;
			} else {
				ROS_ERROR("Unknown joint name: %s", name.c_str());
//				return false;
			}

			joint_pid_controllers_[name.c_str()] = control_toolbox::Pid(p_gain, i_gain, d_gain, i_max, i_min, anti_windup);
		}
	}

	n_joints_ = joint_names_.size();
	for (unsigned i = 0; i < n_joints_; i++)
		ROS_DEBUG("Got joint %s", joint_names_[i].c_str());

	// get publishing period
	if (!nh_priv.getParam("publish_rate", publish_rate_)) {
		ROS_ERROR("Parameter 'publish_rate' not set");
		return false;
	}

	std::string joint_command;
	// get publishing period
	if (!nh_priv.getParam("joint_command", joint_command)) {
		ROS_ERROR("Parameter 'joint_command' not set");
		return false;
	}

	std::string current_state;
	// get publishing period
	if (!nh_priv.getParam("joint_states", current_state)) {
		ROS_ERROR("Parameter 'joint_states' not set");
		return false;
	}

	std::string odom;
	// get publishing period
	if (!nh_priv.getParam("odom", odom)) {
		ROS_ERROR("Parameter 'odom' not set");
		return false;
	}

	std::string odomCam;
	// get publishing period
	if (!nh_priv.getParam("odomCam", odomCam)) {
		ROS_ERROR("Parameter 'odomCam' not set");
		return false;
	}

	std::string current_setpoint;
	// get publishing period
	if (!nh_priv.getParam("setpoint", current_setpoint)) {
		ROS_ERROR("Parameter 'setpoint' not set");
		return false;
	}

	std::string cmd_vel;
	if (!nh_priv.getParam("cmd_vel", cmd_vel)) {
		ROS_WARN_STREAM("Param 'cmd_vel' not set");
		cmd_vel = "cmd_vel";
	}

	std::string camera_cmd_vel;
	if (!nh_priv.getParam("camera_cmd_vel", camera_cmd_vel)) {
		ROS_WARN_STREAM("Param 'camera_cmd_vel' not set");
		camera_cmd_vel = "/camera/cmd_vel";
	}

	if (!nh_priv.getParam("robot_id", robot_id)) {
		ROS_ERROR("Param 'robot_id' not set");
		return false;
	}

	if (!nh_priv.getParam("frame_id", frame_id)) {
		ROS_ERROR("Param 'frame_id' not set");
		return false;
	}

	// realtime publisher
	joint_state_publisher_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(nh, joint_command, 4));

	odomRobotSub.subscribe(nh, odom, 1);
	odomCamSub.subscribe(nh, odomCam, 1);
	approx.reset(new sync_approx(MyApproxSyncPolicy (10), odomRobotSub, odomCamSub));
	approx->registerCallback(boost::bind(&CustomJointController::getOdom,this, _1, _2));
	odomRobotSub.getTopic();
	sub_joint_states2 = nh.subscribe<sensor_msgs::JointState>(current_state, 1, &CustomJointController::getJointStates,
	                                                          this);
	sub_nmpc_goal = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectory>(current_setpoint, 1,
	                                                                       &CustomJointController::getCurrentSetpoint,
	                                                                       this);

	sub_cmd_vel_goal = nh.subscribe<geometry_msgs::Twist>(cmd_vel, 1, &CustomJointController::getDesiredSpeed, this);
	sub_camera_cmd_vel_goal = nh.subscribe<geometry_msgs::Twist>(camera_cmd_vel, 1, &CustomJointController::getDesiredCameraSpeed, this);

	return true;
}

void CustomJointController::starting(const ros::Time &time, const std::map<std::string, double> &position_setpoints) {
	// initialize time
	last_publish_time_ = time;

	for (auto i: position_setpoints) {
		enforceLimits(i.first, i.second);
		commands_struct_[i.first].position_ = i.second;
		commands_struct_[i.first].velocity_ = false;
		commands_[i.first].initRT(commands_struct_[i.first]);
		joint_pid_controllers_[i.first].reset();
	}
}

void CustomJointController::enforceLimits(std::string name, double &position) {
	if (name != "yaw_joint") {
		if (name != "z_joint") {
			if (-max_position_[name] > position) {
				position = -max_position_[name];
			}
		} else {
			if (0.3 > position) {
				position = 0.3;
			}
		}
		if (max_position_[name] < position) {
			position = max_position_[name];
		}
	}
}

void CustomJointController::setCommand(const std::map <std::string, std::vector<double>> &setpoints) {
	first_commad_ = true;
	for (auto joint: setpoints) {
		commands_struct_[joint.first].position_ = joint.second[0];
		if (joint.second.size() > 1) {
			commands_struct_[joint.first].velocity_ = joint.second[1];
			commands_struct_[joint.first].has_velocity_ = true;
		}
		commands_[joint.first].writeFromNonRT(commands_struct_[joint.first]);
//		ROS_INFO_STREAM(joint.first << " " << commands_struct_[joint.first].position_ << " " << commands_[joint.first].readFromNonRT()->position_);
	}
}

void CustomJointController::limitVelocity(const std::string &name, double &velocity) {
	if (velocity > max_velocity_[name]) {
		velocity = max_velocity_[name];
	}
	if (velocity < -max_velocity_[name]) {
		velocity = -max_velocity_[name];
	}
}


// fixme update to setError/whatever. updatePid use OPPOSITE convention!!! https://github.com/ros-controls/ros_control/pull/12
void CustomJointController::update(const ros::Time &time, const ros::Duration &period,
                                   const std::map <std::string, std::vector<double>> &current_state) {
	if (not first_commad_) {
		first_commad_ = true;
		setCommand(current_state);
	}
	for (auto joint_name: joint_names_) {
		commands_struct_[joint_name] = *(commands_[joint_name].readFromRT());
		double command_position = commands_struct_[joint_name].position_;
		double command_velocity = commands_struct_[joint_name].velocity_;
		bool has_velocity_ = commands_struct_[joint_name].has_velocity_;

		double error, vel_error;
		double commanded_velocity;

		auto it = current_state.find(joint_name);
		double current_position = it->second[0];
		double current_velocity = it->second[1];

		enforceLimits(joint_name, command_position);
//		std::cout << "Limit enforced " << joint_name << " " << command_position << std::endl;

		if (joint_name == "roll_joint" || joint_name == "pitch_joint") {
			angles::shortest_angular_distance_with_large_limits(
				current_position,
				command_position,
				-max_position_[joint_name],
				max_position_[joint_name],
				error);
		} else if (joint_name == "yaw_joint") {
			error = angles::shortest_angular_distance(current_position, command_position);
		} else {
			error = command_position - current_position;
		}
		error = -error; //updatePID use this convention

		if (has_velocity_) {
			vel_error = -(command_velocity - current_velocity);
			if (!ignore_position)
				commanded_velocity = joint_pid_controllers_[joint_name].updatePid(error, vel_error, period);
			else {
				commanded_velocity = joint_pid_controllers_[joint_name].updatePid(vel_error, period);
			}
		} else {
			commanded_velocity = joint_pid_controllers_[joint_name].updatePid(error, period);
		}
		ROS_INFO_STREAM(commanded_velocity);
		limitVelocity(joint_name, commanded_velocity);
		if (isnan(commanded_velocity)){
			commanded_velocity = 0;
		}
		// if error < 0.1, set velocity to 0
		if (!ignore_position)
			if (fabs(error) < 0.05) {
				commanded_velocity = 0;
			}
		std::cout << "For joint " << joint_name << " error is " << error << " velerror " << vel_error << " current vel " << current_velocity << " and the vel " << commanded_velocity << std::endl;
		joint_pid_controllers_[joint_name].setCurrentCmd(commanded_velocity);
	}

	// limit rate of publishing
	if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time) {

		// try to publish
		if (joint_state_publisher_->trylock()) {
			if (robot_id == IROTATE){
				pubIRotate(time);
			}
			if (robot_id == DRONE){
				pubDrone(time);
			}
		}
	}
}

void CustomJointController::pubDrone(ros::Time time){
	// we're actually publishing, so increment time
	last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

	// populate joint state message:
	// - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
	// - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
	joint_state_publisher_->msg_ = sensor_msgs::JointState();
	joint_state_publisher_->msg_.header.stamp = time;

	for (const auto &joint: joint_pid_controllers_) {
		joint_state_publisher_->msg_.name.push_back(std::string(joint.first));
		joint_state_publisher_->msg_.velocity.push_back(
			joint_pid_controllers_[std::string(joint.first)].getCurrentCmd());
	}
	joint_state_publisher_->unlockAndPublish();
}

void CustomJointController::pubIRotate(ros::Time time) {


	double vx = 0; //vel_x * cos(w) - vel_y * sin(w);
	double vy = 0; //vel_x * sin(w) + vel_y * cos(w);
	double vw = 0;
	double vwcam = 0;

	// we're actually publishing, so increment time
	last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

	// populate joint state message:
	// - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
	// - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
	joint_state_publisher_->msg_ = sensor_msgs::JointState();
	joint_state_publisher_->msg_.header.stamp = time;

	for (const auto &joint: joint_pid_controllers_) {
		if (std::string(joint.first) == "x_joint") {
			joint_state_publisher_->msg_.name.push_back(std::string(joint.first));
			vx = joint_pid_controllers_[std::string(joint.first)].getCurrentCmd();
			joint_state_publisher_->msg_.velocity.push_back(vx);
			ROS_INFO_STREAM(joint.first << " " << vx);
		}
		if (std::string(joint.first) == "y_joint") {
			joint_state_publisher_->msg_.name.push_back(std::string(joint.first));
			vy = joint_pid_controllers_[std::string(joint.first)].getCurrentCmd();
			joint_state_publisher_->msg_.velocity.push_back(vy);
			ROS_INFO_STREAM(joint.first << " " << vy);
		}

		if (std::string(joint.first) == "yaw_joint") {
			joint_state_publisher_->msg_.name.push_back(std::string(joint.first));
			vw = joint_pid_controllers_[std::string(joint.first)].getCurrentCmd();
			joint_state_publisher_->msg_.velocity.push_back(vw);
			ROS_INFO_STREAM(joint.first << " " << vw);
		}

		if (std::string(joint.first) == "cameraholder_joint") {
			joint_state_publisher_->msg_.name.push_back(std::string(joint.first));
			vwcam = joint_pid_controllers_[std::string(joint.first)].getCurrentCmd();
			joint_state_publisher_->msg_.velocity.push_back(vwcam);
			ROS_INFO_STREAM(joint.first << " " << vwcam);
		}
	}

	// apply kinematics - current wheel config.
	// odometry obtained from literature_robotino...
	// vx sin(alpha i) + vy cos(alpha i) + vw R
	double v0 = -vx * SIN_PI_3 + vy * COS_PI_3 + vw * R_BASE;
	double v1 = -vy + vw * R_BASE;
	double v2 = +vx * SIN_PI_3 + vy * COS_PI_3 + vw * R_BASE;

	// get rad/s for each wheel
	double w0 = v0 / R_WHEEL;
	double w1 = v1 / R_WHEEL;
	double w2 = v2 / R_WHEEL;

	for (const auto &joint: joint_pid_controllers_) {
		if (std::string(joint.first) == "wheel0_joint") {
			joint_state_publisher_->msg_.name.push_back(std::string(joint.first));
			joint_state_publisher_->msg_.velocity.push_back(w0);
		}
		if (std::string(joint.first) == "wheel1_joint") {
			joint_state_publisher_->msg_.name.push_back(std::string(joint.first));
			joint_state_publisher_->msg_.velocity.push_back(w1);
		}
		if (std::string(joint.first) == "wheel2_joint") {
			joint_state_publisher_->msg_.name.push_back(std::string(joint.first));
			joint_state_publisher_->msg_.velocity.push_back(w2);
		}
	}

	joint_state_publisher_->unlockAndPublish();
}

void CustomJointController::getJointStates(const sensor_msgs::JointState::ConstPtr &msg) {
	std::map <std::string, std::vector<double>> current_state;
	for (int i = 0; i < msg->name.size(); i++) {
		current_state[msg->name[i]].emplace_back(msg->position[i]);
		current_state[msg->name[i]].emplace_back(msg->velocity[i]);
	}
	update(msg->header.stamp, ros::Duration(1 / publish_rate_), current_state);
}

void CustomJointController::getOdom(const nav_msgs::Odometry::ConstPtr &msg, const nav_msgs::Odometry::ConstPtr &msgCam) {
	std::map <std::string, std::vector<double>> current_state;
	for (const auto &name: joint_names_) {
		double position, velocity;

		tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
		                 msg->pose.pose.orientation.w);

		tf::quaternionTFToEigen(q, quaternion_odom);
		Eigen::Vector3d vel(msg->twist.twist.linear.x , msg->twist.twist.linear.y, msg->twist.twist.linear.z);
		vel = quaternion_odom * vel;

		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		if (name == "x_joint") {
			position = msg->pose.pose.position.x;
			velocity = vel(0);
		}
		else if (name == "y_joint") {
			position = msg->pose.pose.position.y;
			velocity = vel(1);
		}
		else if (name == "z_joint") {
			position = msg->pose.pose.position.z;
			velocity = vel(2);
		}
		else if (name == "roll_joint") {
			position = roll;
			velocity = msg->twist.twist.angular.x;
		}
		else if (name == "pitch_joint") {
			position = pitch;
			velocity = msg->twist.twist.angular.y;
		}
		else if (name == "yaw_joint") {
			position = yaw;
			velocity = msg->twist.twist.angular.z;
		}
		else if (name == "cameraholder_joint") {
			tf::Quaternion qcam(msgCam->pose.pose.orientation.x, msgCam->pose.pose.orientation.y, msgCam->pose.pose.orientation.z,
			                 msgCam->pose.pose.orientation.w);
			tf::Matrix3x3 mc(qcam);
			double rollcam, pitchcam, yawcam;
			m.getRPY(rollcam, pitchcam, yawcam);
			position = yawcam;
			velocity = msgCam->twist.twist.angular.z;
			ROS_INFO_STREAM("CAMERAHOLDER VEL " << velocity);
		}
		else {position = 0; velocity = 0;}

		current_state[name].emplace_back(position);
		current_state[name].emplace_back(velocity);
	}
	update(msg->header.stamp, ros::Duration(1 / publish_rate_), current_state);
}


void CustomJointController::getCurrentSetpoint(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr &msg) {
	std::map <std::string, std::vector<double>> local_setpoints;

	int index = 0;
	if (!msg->points.empty() && msg->points.size() > 1) {
		index = (int) ((msg->points.size() - 1));
	} else {
		return;
	}
	double vel_x, vel_y, vel_z;
	Eigen::Vector3d vel(msg->points[index].velocities[0].linear.x,	msg->points[index].velocities[0].linear.y,	msg->points[index].velocities[0].linear.z);
	if (frame_id != "world")
		vel = quaternion_odom * vel;

	local_setpoints["x_joint"].emplace_back(msg->points[index].transforms[0].translation.x);
	local_setpoints["x_joint"].emplace_back(vel(0));
	local_setpoints["y_joint"].emplace_back(msg->points[index].transforms[0].translation.y);
	local_setpoints["y_joint"].emplace_back(vel(1));
	local_setpoints["z_joint"].emplace_back(msg->points[index].transforms[0].translation.z);
	local_setpoints["z_joint"].emplace_back(vel(2));

	// convert ros quaternion to rpy
	tf::Quaternion q(msg->points[index].transforms[0].rotation.x, msg->points[index].transforms[0].rotation.y,
	                 msg->points[index].transforms[0].rotation.z, msg->points[index].transforms[0].rotation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	local_setpoints["roll_joint"].emplace_back(roll);
	local_setpoints["roll_joint"].emplace_back(msg->points[index].velocities[0].angular.x);
	local_setpoints["pitch_joint"].emplace_back(pitch);
	local_setpoints["pitch_joint"].emplace_back(msg->points[index].velocities[0].angular.y);
	local_setpoints["yaw_joint"].emplace_back(yaw);
	local_setpoints["yaw_joint"].emplace_back(msg->points[index].velocities[0].angular.z);

	setCommand(local_setpoints);
	ignore_position = false;
}

void CustomJointController::getDesiredSpeed(const geometry_msgs::Twist::ConstPtr &msg) {
	ROS_INFO_STREAM("Desired speed: " << msg->linear.x << " " << msg->linear.y << " " << msg->angular.z);

	std::map <std::string, std::vector<double>> local_setpoints;

	double vel_x, vel_y;
	Eigen::Vector3d vel(msg->linear.x , msg->linear.y, 0);
	if (frame_id != "world")
		vel = quaternion_odom * vel;

	local_setpoints["x_joint"].emplace_back(0);
	local_setpoints["x_joint"].emplace_back(vel(0));
	local_setpoints["y_joint"].emplace_back(0);
	local_setpoints["y_joint"].emplace_back(vel(1));

//	local_setpoints["z_joint"].emplace_back(0);
//	local_setpoints["z_joint"].emplace_back(msg->linear.z);
//	local_setpoints["roll_joint"].emplace_back(0);
//	local_setpoints["roll_joint"].emplace_back(msg->angular.x);
//	local_setpoints["pitch_joint"].emplace_back(0);
//	local_setpoints["pitch_joint"].emplace_back(msg->angular.y);

	local_setpoints["yaw_joint"].emplace_back(0);
	local_setpoints["yaw_joint"].emplace_back(msg->angular.z);

	ignore_position = true;

	setCommand(local_setpoints);
}


void CustomJointController::getDesiredCameraSpeed(const geometry_msgs::Twist::ConstPtr &msg) {
	ROS_INFO_STREAM("Desired Camera speed: " << msg->angular.z);

	std::map <std::string, std::vector<double>> local_setpoints;

	local_setpoints["cameraholder_joint"].emplace_back(0);
	local_setpoints["cameraholder_joint"].emplace_back(msg->angular.z);

	ignore_position = true;

	setCommand(local_setpoints);
}

void CustomJointController::stopping(const ros::Time &time) {}
