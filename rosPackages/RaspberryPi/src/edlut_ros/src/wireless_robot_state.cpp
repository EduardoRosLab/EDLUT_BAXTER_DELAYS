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
#include <sensor_msgs/JointState.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <cstring>
#include <ctime>
#include <limits>
#include <signal.h>
#include <edlut_ros/ExternalClock.h>
#include <queue>


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
	ExternalClock ext_clock;
private:
	ros::Publisher robotState_publisher;

	ros::Publisher time_publisher;

	std::string output_topic;

	bool use_sim_time;


public:
	RobotState (bool sim_time, std::string output_topic):
		use_sim_time(sim_time), output_topic(output_topic) {
		ros::NodeHandle nh;

		this->robotState_publisher = nh.advertise<sensor_msgs::JointState>(output_topic, 10);

		return;
	}


	void EventCallback(const sensor_msgs::JointState::ConstPtr& msg){
		sensor_msgs::JointState robotStateMsg;
		ros::Time CurrentTime;

		if (!this->use_sim_time){
			CurrentTime = ros::Time::now();
		}
		else{
			CurrentTime = this->ext_clock.GetLastConfirmedTime();
		}

		robotStateMsg.header.stamp = CurrentTime;

		robotStateMsg.name = msg->name;
		robotStateMsg.position = msg->position;
		robotStateMsg.velocity = msg->velocity;
		robotStateMsg.effort = msg->effort;



		this->robotState_publisher.publish(robotStateMsg);

		return;
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
	bool sim_time;
	ros::Publisher time_publisher;


	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("input_topic", input_topic);
	private_node_handle_.getParam("output_topic", output_topic);



	nh.getParam("use_sim_time", sim_time);


	RobotState objRobotState = RobotState(sim_time, output_topic);

	// Create the subscriber
	ros::CallbackQueue CallbackQueue;
	nh.setCallbackQueue(&CallbackQueue);

	ros::SubscribeOptions baxterStateOptions =
	 ros::SubscribeOptions::create<sensor_msgs::JointState>(
	 input_topic, 1000, boost::bind(&RobotState::EventCallback, &objRobotState, _1), ros::VoidPtr(), &CallbackQueue);

 // Because TCP causes bursty communication with high jitter,
 // declare a preference on UDP connections for receiving
 // joint states, which we want to get at a high rate.
 // Note that we'll still accept TCP connections for this topic
 // (e.g., from rospy nodes, which don't support UDP);
 // we just prefer UDP.
 baxterStateOptions.transport_hints =
	 // ros::TransportHints().reliable().tcpNoDelay(true);
	 ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

 ros::Subscriber input_subscriber= nh.subscribe(baxterStateOptions);

	if (sim_time){
		//ROS_DEBUG("Wireless Robot State: Subscribing to topic /clock_sync");
		objRobotState.clock_subscriber = nh.subscribe("/clock_sync", 1000, &ExternalClock::ClockCallback, &(objRobotState.ext_clock));
	}

	//ROS_INFO("Wireless  Robot State node initialized: reading from topic %s and writing to %s", input_topic.c_str(), output_topic.c_str());

	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.002));
	}

	//ROS_INFO("Ending Wireless Robot State node");

	ros::shutdown();
	return 0;
} // end main()
