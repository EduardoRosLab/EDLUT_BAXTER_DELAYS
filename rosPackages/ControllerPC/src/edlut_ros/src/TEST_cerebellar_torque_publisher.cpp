/***************************************************************************
 *                           adaptive_mean_filter.cpp           			     *
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
	ros::init(argc, argv, "cerebellar_torque", ros::init_options::NoSigintHandler);
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string output_topic;
	std::vector<std::string> joint_list;
	double sampling_frequency;
	double delay, period;

	double amplitude, frequency;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("joint_list", joint_list);
	private_node_handle_.getParam("period", period);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("delay", delay);

	ros::Publisher publisher = nh.advertise<edlut_ros::AnalogCompact>(output_topic, 1);

	ROS_INFO("TEST Cerebellar torque node initialized");

	ros::Rate rate(sampling_frequency);

	edlut_ros::AnalogCompact newMessage;
	newMessage.names = joint_list;

	int count = 0;
	amplitude = 2.0;
	frequency = 1.0;
	const double pi = 3.1415926535897;

	while (!stop_node){
		std::vector<double> data_aux (joint_list.size(), count);
		double elapsedTime = ros::Time::now().toSec();

		for (unsigned int i=0; i<joint_list.size(); i++){
			data_aux[i] = amplitude * sin(2*pi*frequency*elapsedTime);
		}
		if (elapsedTime>period*count){
			count +=1;
			amplitude = 2.0;
		}
		newMessage.data = data_aux;
		newMessage.header.stamp = ros::Time::now() + ros::Duration(delay);
		count += 1;
		publisher.publish(newMessage);
		// ROS_INFO("CEREBELLAR TORQUE PUBLISHED");
		// std::cout<<newMessage;
		// ROS_INFO("Published Cerebellar torque: \n time stamp: %f \n torque[0]: %f", newMessage.header.stamp.toSec(), newMessage.data[0]);
		rate.sleep();
	}
	ros::shutdown();
	return 0;
} // end main()
