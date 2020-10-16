/***************************************************************************
 *                          full_state_node.cpp 			                     *
 *                           -------------------                           *
 * copyright            : (C) 2019 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es           			                   *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// This node computes an ouput message containing the desired position and velocity
// and the associated actual position and velocity. To do so it takes into account the
// transmission delays related to the sensorial information sent from the robot to
// the computer.


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <edlut_ros/FullStateCompactDelay.h>
#include <edlut_ros/AnalogCompactDelay.h>
#include <edlut_ros/ROSFullStateCalculatorCompact.h>
#include "log4cxx/logger.h"
#include "edlut_ros/ExternalClock.h"


#include <cstring>
#include <ctime>
#include <cmath>
#include <signal.h>

static bool stop_node;

void rosShutdownHandler(int sig)
{
	stop_node = true;
}



int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "full_state_node");
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string desired_position_topic, desired_velocity_topic, current_position_topic, current_velocity_topic, output_topic;
	std::vector<std::string> joint_list;
	double sampling_frequency, sensorial_delay;

	bool use_sim_time;

	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;

	ros::Time current_time(0.0);

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("desired_position_topic", desired_position_topic);
	private_node_handle_.getParam("desired_velocity_topic", desired_velocity_topic);
	private_node_handle_.getParam("current_position_topic", current_position_topic);
	private_node_handle_.getParam("current_velocity_topic", current_velocity_topic);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("joint_list", joint_list);
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("sensorial_delay", sensorial_delay);


	nh.getParam("use_sim_time", use_sim_time);

	// Create the subscriber
	ros::CallbackQueue CallbackQueue;
	nh.setCallbackQueue(&CallbackQueue);

	if (use_sim_time){
		ROS_DEBUG("Error estimator node: Subscribing to topic /clock_sync");
		clock_subscriber = nh.subscribe("/clock_sync", 1000, &ExternalClock::ClockCallback, &ext_clock);
	}

	// Create the publisher
	ros::Publisher output_publisher = nh.advertise<edlut_ros::FullStateCompactDelay>(output_topic, 1.0);
	ROSFullStateCalculatorCompact objFullStateCalculator(desired_position_topic,
			desired_velocity_topic,
			current_position_topic,
			current_velocity_topic,
			output_topic,
			joint_list,
			sensorial_delay,
			sampling_frequency);

	ROS_INFO("Full state node initialized: writing to topic %s with frequency %f", output_topic.c_str(), sampling_frequency);

	ros::Rate rate(sampling_frequency);

	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.003));
		if (use_sim_time){
			ros::Time new_time = ext_clock.GetLastConfirmedTime();
			if (new_time > current_time){
				current_time = new_time;
				ROS_DEBUG("Updating full state at time %f", current_time.toSec());
				objFullStateCalculator.UpdateError(current_time);
				rate.sleep();
				//ROS_INFO("Error node Current time: %f ", current_time.toSec());
			}
		}
		else{
			current_time = ros::Time::now();
			ROS_DEBUG("Updating full state at time %f", current_time.toSec());
			objFullStateCalculator.UpdateError(current_time);
			rate.sleep();
		}
		//ROS_INFO("Error node Current time 2: %f ", current_time.toSec());
	}

	ROS_INFO("Ending full state node");

	ros::shutdown();
	return 0;
} // end main()
