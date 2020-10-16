/***************************************************************************
 *                           wireless_robot_state.cpp     			           *
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

// This node gets Baxter's current joint states and re-publishes with a timeStamp
// matching the PC's current time. (This is done to sync the joint_states messages
// with the PC's clock since Baxter runs its own clock)

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

#define PI 3.14159265

static bool stop_node;

void rosShutdownHandler(int sig)
{
	stop_node = true;
}

/*
 * Create a class
 */
class RobotState {
public:
	ros::Subscriber clock_subscriber;
private:
	ros::Publisher publisher, re_publisher;

	ros::Publisher time_publisher;

	std::string output_topic, re_output_topic;

	int amplitude, size_msg;

public:
	RobotState (std::string output_topic, int amplitude, int size_msg):
		output_topic(output_topic), amplitude(amplitude), size_msg(size_msg) {
		ros::NodeHandle nh;

		this->publisher = nh.advertise<edlut_ros::AnalogCompact>(output_topic, 10);
		this->re_publisher = nh.advertise<edlut_ros::Analog>(re_output_topic, 10);

		return;
	}


	void EventCallback(const edlut_ros::Analog::ConstPtr& msg){
		edlut_ros::Analog new_msg;
		new_msg.header.stamp = ros::Time::now();
		new_msg.data = msg->data + 1;
		this->re_publisher.publish(new_msg);
		return;
	}


	void PublishEvent(){
		edlut_ros::AnalogCompact new_msg;
		new_msg.header.stamp = ros::Time::now();
		new_msg.data = std::vector<double> (this->size_msg, 0);
		new_msg.data[0] = this->amplitude * sin (2*PI*ros::Time::now().toSec());

		this->publisher.publish(new_msg);
	}
};

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "wireless_robot_state", ros::init_options::NoSigintHandler);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
			ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string input_topic, output_topic;
	ros::Publisher publisher;
	double publish_rate;
	int amplitude, size_msg;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("input_topic", input_topic);
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("publish_rate", publish_rate);
	private_node_handle_.getParam("amplitude", amplitude);
	private_node_handle_.getParam("size_msg", size_msg);


	ros::Rate loop_rate(publish_rate);

	RobotState objRobotState = RobotState(output_topic, amplitude, size_msg);

	// // Create the subscriber
	// ros::CallbackQueue CallbackQueue;
	// nh.setCallbackQueue(&CallbackQueue);
 //
	// ros::SubscribeOptions baxterStateOptions =
	//  ros::SubscribeOptions::create<edlut_ros::Analog>(
	//  input_topic, 1000, boost::bind(&RobotState::EventCallback, &objRobotState, _1), ros::VoidPtr(), &CallbackQueue);
 //
 // // Because TCP causes bursty communication with high jitter,
 // // declare a preference on UDP connections for receiving
 // // joint states, which we want to get at a high rate.
 // // Note that we'll still accept TCP connections for this topic
 // // (e.g., from rospy nodes, which don't support UDP);
 // // we just prefer UDP.
 // baxterStateOptions.transport_hints =
	//  // ros::TransportHints().reliable().tcpNoDelay(true);
	//  ros::TransportHints().unreliable().reliable().tcpNoDelay(true);
 //
 // ros::Subscriber input_subscriber= nh.subscribe(baxterStateOptions);

	//ROS_INFO("Wireless Robot State node initialized: reading from topic %s and writing to %s", input_topic.c_str(), output_topic.c_str());

	while (!stop_node){
		// CallbackQueue.callAvailable(ros::WallDuration(0.002));
		objRobotState.PublishEvent();
		loop_rate.sleep();
	}

	//ROS_INFO("Ending Wireless Robot State node");

	ros::shutdown();
	return 0;
} // end main()
