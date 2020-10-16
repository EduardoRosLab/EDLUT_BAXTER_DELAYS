/***************************************************************************
 *                           torque_final_node.cpp            					   *
 *                           -------------------                           *
 * copyright            : (C) 2019 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es			                             *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// This node transforms a given input torque command into an output
// robot/limb/left-right/joint_command that Baxter robot can actually process.


#include <edlut_ros/AnalogCompact.h>
#include <edlut_ros/LearningState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <edlut_ros/TorqueAddition_Delay.h>
#include <std_msgs/Empty.h>
#include "std_msgs/Bool.h"


#include "log4cxx/logger.h"
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <cstring>
#include <ctime>
#include <limits>
#include <signal.h>
#include <map>
#include <queue>


static bool stop_node;

void rosShutdownHandler(int sig)
{
	stop_node = true;
}


int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "baxter_final_torque_command", ros::init_options::NoSigintHandler);
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string output_topic;
	std::string cerebellar_torque_topic, pd_supervisor_torque_topic, mean_torque_topic, motor_delay_topic;
	std::vector<std::string> joint_list;
	std::string gravity_topic;
	bool disable_gravity_compensation, wireless_simulation;
	double sampling_frequency, T_iteration;

	double time_step = 1.0 / sampling_frequency;

	int n_iterations_backup, iteration_samples, past_cerebellar_torque_buffer_size;
	int backup_copy_size, min_future_samples;

	std::string iteration_filter_size_topic, future_buffer_samples_topic;

	double torque_reduction_factor;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("cerebellar_torque_topic", cerebellar_torque_topic);
	private_node_handle_.getParam("pd_supervisor_torque_topic", pd_supervisor_torque_topic);
	private_node_handle_.getParam("joint_list", joint_list);
	private_node_handle_.getParam("past_cerebellar_torque_buffer_size", past_cerebellar_torque_buffer_size);
	private_node_handle_.getParam("min_future_samples", min_future_samples);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("disable_gravity_compensation", disable_gravity_compensation);
	private_node_handle_.getParam("gravity_topic", gravity_topic);
	private_node_handle_.getParam("T_iteration", T_iteration);
	private_node_handle_.getParam("n_iterations_backup", n_iterations_backup);
	private_node_handle_.getParam("iteration_samples", iteration_samples);
	private_node_handle_.getParam("motor_delay_topic", motor_delay_topic);
	private_node_handle_.getParam("iteration_filter_size_topic", iteration_filter_size_topic);
	private_node_handle_.getParam("future_buffer_samples_topic", future_buffer_samples_topic);
	private_node_handle_.getParam("torque_reduction_factor", torque_reduction_factor);

	nh.getParam("wireless_simulation", wireless_simulation);

	// Create the subscriber
	ros::CallbackQueue CallbackQueue;
	nh.setCallbackQueue(&CallbackQueue);

	backup_copy_size = n_iterations_backup * iteration_samples;


	TorqueAddition_Delay objTorqueBuffer(wireless_simulation, cerebellar_torque_topic, pd_supervisor_torque_topic, output_topic, joint_list, past_cerebellar_torque_buffer_size,
																min_future_samples, T_iteration, iteration_samples, n_iterations_backup, disable_gravity_compensation, gravity_topic, &nh, &CallbackQueue, motor_delay_topic,
																time_step, iteration_filter_size_topic, future_buffer_samples_topic, torque_reduction_factor);

	// ROS_INFO("Baxter Torque Command node initialized");

	ros::Rate rate(sampling_frequency);


	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.001));
		// rate.sleep();
	}

	ros::shutdown();
	return 0;
} // end main()
