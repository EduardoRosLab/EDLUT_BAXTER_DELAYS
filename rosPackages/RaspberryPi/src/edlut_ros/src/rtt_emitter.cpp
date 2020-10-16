/***************************************************************************
 *                           rtt_emitter.cpp					     			           *
 *                           -------------------                           *
 * copyright            : (C) 2019 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es 			                             *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// This node calculates the round trip time of a message

#include <edlut_ros/AnalogCompact.h>
#include <edlut_ros/Analog.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <cstring>
#include <ctime>
#include <limits>
#include <signal.h>
#include <queue>
#include <math.h>


static bool stop_node;

void rosShutdownHandler(int sig)
{
	stop_node = true;
}

/*
 * Create a class
 */
class RTT {
private:
	ros::Publisher publisher;

	std::string output_topic;

	int size_msg;

public:
	RTT (std::string output_topic, int size_msg):
		output_topic(output_topic), size_msg(size_msg) {
		ros::NodeHandle nh;

		this->publisher = nh.advertise<edlut_ros::AnalogCompact>(output_topic, 10);

		return;
	}

	void PublishEvent(){
		edlut_ros::AnalogCompact new_msg;
		new_msg.header.stamp = ros::Time::now();
		new_msg.data = std::vector<double> (this->size_msg, 0.0);
		new_msg.data[0] = new_msg.header.stamp.toSec();
		this->publisher.publish(new_msg);
	}

};

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "rtt_emitter", ros::init_options::NoSigintHandler);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
			ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string output_topic;

	double publishing_rate;
	int size_msg;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("publishing_rate", publishing_rate);
	private_node_handle_.getParam("size_msg", size_msg);

	RTT rtt = RTT(output_topic, size_msg);

	ros::Rate rate(publishing_rate);

	while (!stop_node){
		rtt.PublishEvent();
		rate.sleep();
	}
	ROS_INFO("Ending RTT node");

	ros::shutdown();
	return 0;
} // end main()
