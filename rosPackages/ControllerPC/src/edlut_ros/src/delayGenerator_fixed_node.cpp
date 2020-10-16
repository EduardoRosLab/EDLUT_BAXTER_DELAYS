/***************************************************************************
 *                           delayGenerator_node.cpp 				               *
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

// This node generates an artificial delay and publishes to a topic

#include <edlut_ros/Time.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <cstring>
#include <ctime>
#include <limits>
#include <signal.h>
#include "edlut_ros/ExternalClock.h"
#include <queue>
#include <dynamic_reconfigure/server.h>
#include "edlut_ros/DelayGeneratorDynParametersConfig.h"
#include <edlut_ros/DelayGeneratorDynParams.h>



static bool stop_node;

void rosShutdownHandler(int sig)
{
	stop_node = true;
}


/*
 * Create a class
 */
class Delay {
public:
	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;
private:
	ros::Publisher delayPublisher;

	double delay;

	edlut_ros::Time delayMsg;

	bool use_sim_time;

	ros::NodeHandle *nh_;


public:
	Delay (std::string delay_topic,
		bool sim_time,
		double delay,
		ros::NodeHandle *nh): use_sim_time(sim_time), nh_(nh) {

		this->delay =  0.0;
		this->delayPublisher = nh_->advertise<edlut_ros::Time>(delay_topic, 10);

		return;
	}

	void SetDelay(double delay){
		this->delay = delay;
	}


	void PublishDelay(ros::Time msgTime){
		this->delayMsg.header.stamp = msgTime;
		this->delayMsg.data = this->delay;

		this->delayPublisher.publish(this->delayMsg);

		return;
	}
};

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "delayGenerator", ros::init_options::NoSigintHandler);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
			ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string delay_topic;
	bool sim_time, wireless_simulation;
	double delay;

	double sampling_frequency, checking_frequency;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("delay_topic", delay_topic);
	private_node_handle_.getParam("checking_frequency", checking_frequency);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("delay", delay);

	nh.getParam("use_sim_time", sim_time);
	nh.getParam("wireless_simulation", wireless_simulation);


	// Synchronizer node will shutdown when running in real time
	if (!wireless_simulation){
		ROS_WARN("Delay Generator: Not a simulated wireless scenario. Delay Generator node will shutdown. If you want to enable simulated wireless scenario set wireless_simulation variable to true.");
		ros::shutdown();
		return 0;
	}

	// Create the subscriber
	ros::CallbackQueue CallbackQueue;
	nh.setCallbackQueue(&CallbackQueue);

	private_node_handle_.setCallbackQueue(&CallbackQueue);

	Delay objDelay = Delay(delay_topic, sim_time, delay, &nh);

	DelayGeneratorDynParams delayParameters(&nh, &private_node_handle_);

 ros::Rate rate(sampling_frequency);
 ros::Time time(0.0), last_sent_time(0.0);
 ros::WallRate sim_rate(checking_frequency);
 ros::WallRate init_rate(1.0);

	if (sim_time){
		ROS_DEBUG("Delay Generator node: Subscribing to topic /clock_sync");
		objDelay.clock_subscriber = nh.subscribe("/clock_sync", 1000, &ExternalClock::ClockCallback, &(objDelay.ext_clock));

		while(!objDelay.ext_clock.FirstReceived()){
			ROS_DEBUG("Delay Generator node: Synchronizing");
			CallbackQueue.callAvailable(ros::WallDuration(0.001));
			init_rate.sleep();
		}
		time = objDelay.ext_clock.GetLastConfirmedTime();
		ROS_DEBUG("Delay Generator node: Node synchronized");
	}

	while (!stop_node){
		CallbackQueue.callAvailable();
		if (sim_time){
			ros::Time new_time = objDelay.ext_clock.GetLastConfirmedTime();
			if (new_time>time){
				time = new_time;
				objDelay.PublishDelay(time);
			}
			sim_rate.sleep();
		} else{
			objDelay.SetDelay(delayParameters.GetFixedDelay());
			objDelay.PublishDelay(ros::Time::now());
			rate.sleep();
		}
	}

	ROS_INFO("Ending Delay Generator node");

	ros::shutdown();
	return 0;
} // end main()
