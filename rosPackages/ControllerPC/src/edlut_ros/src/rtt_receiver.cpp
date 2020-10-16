/***************************************************************************
 *                           rtt_receiver.cpp					     			           *
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
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <cstring>
#include <ctime>
#include <limits>
#include <signal.h>
#include <queue>
#include <math.h>

#include <edlut_ros/Spike_group.h>
#include "edlut_ros/ROSTopicInputDriver.h"


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

	void EventCallback(const edlut_ros::AnalogCompact::ConstPtr& msg){
		edlut_ros::AnalogCompact new_msg;
		new_msg.data = std::vector<double> (this->size_msg, 0.0);
		new_msg.data[0] = msg->data[0];
		new_msg.header.stamp = ros::Time::now();
		this->publisher.publish(new_msg);
		return;
	}


	void SpikeCallback(const edlut_ros::Spike_group::ConstPtr& msg){
		ROS_INFO("INPUT SPIKE CALLBACK ENTRY RTT RECEIVER");
		// if (msg->neuron_index.size()>0){
		// 	ROS_INFO("RECEIVING SPIKES ");
		// }
		// for (int i=0; i<msg->neuron_index.size(); i++){
		// 	ROS_DEBUG("EDLUT: Received spike: time %f and neuron %d. Current time: %f. Reference time: %f", msg->time[i], msg->neuron_index[i], ros::Time::now().toSec(), this->InitSimulationRosTime);
		// 	InputSpike * NewSpike = new InputSpike(msg->time[i] - this->InitSimulationRosTime, this->Net->GetNeuronAt(msg->neuron_index[i])->get_OpenMP_queue_index(), this->Net->GetNeuronAt(msg->neuron_index[i]));
		// 	this->Queue->InsertEvent(NewSpike->GetSource()->get_OpenMP_queue_index(),NewSpike);
		// }
	}

};

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "rtt_receiver", ros::init_options::NoSigintHandler);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
			ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string input_topic, output_topic;

	double publishing_rate;
	double callbackRate = 1.0 / publishing_rate;

	int size_msg;

	stop_node = false;


	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("input_topic", input_topic);
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("publishing_rate", publishing_rate);
	private_node_handle_.getParam("size_msg", size_msg);


	ros::Rate loop_rate(publishing_rate);

	RTT rtt = RTT(output_topic, size_msg);

	// Create the subscriber with options (TCP // UDP)
	ros::CallbackQueue CallbackQueue;
	nh.setCallbackQueue(&CallbackQueue);

	ros::SubscribeOptions RTTOptions =
	 ros::SubscribeOptions::create<edlut_ros::AnalogCompact>(
	 	input_topic, 1000, boost::bind(&RTT::EventCallback, &rtt, _1), ros::VoidPtr(), &CallbackQueue);

 	RTTOptions.transport_hints = ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

 	ros::Subscriber subscriber= nh.subscribe(RTTOptions);

	// Subscriber without options
	// ros::Subscriber subscriber = nh.subscribe(input_topic, 1, &RTT::EventCallback, &rtt);



	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.002));
	}

	ROS_INFO("Ending RTT node");

	ros::shutdown();
	return 0;
} // end main()
