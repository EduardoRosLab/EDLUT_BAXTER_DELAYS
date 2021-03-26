/***************************************************************************
 *                           delayGenerator_normalDistributionnode.cpp     *
 *                           -------------------                           *
 * copyright            : (C) 2021 by Ignacio Abadia                       *
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

// This node generates an artificial delay following a normal distribution and
// publishes it to a topic

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
#include <random>

#include <iostream>
#include <fstream>

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
	double previous_delay;

	bool first_delay;

	edlut_ros::Time delayMsg;

	bool use_sim_time;

	ros::NodeHandle *nh_;

	std::default_random_engine generator;
	std::gamma_distribution<double> distribution;

	double time_step;
	double offset;
	double alpha;
	double beta;
	double symmetry_factor;
	bool inverted;

	double mean;

	std::vector<double> delay_data;

public:
	Delay (std::string delay_topic,
		bool sim_time,
		double alpha,
		double beta,
		double offset,
		bool inverted,
		bool symmetric,
		ros::NodeHandle *nh,
	 	double time_step): use_sim_time(sim_time), alpha(alpha), beta(beta), offset(offset), inverted(inverted), nh_(nh), time_step(time_step) {

		this->delay =  0.0;
		this->delayPublisher = nh_->advertise<edlut_ros::Time>(delay_topic, 10);

		this->distribution = std::gamma_distribution<double> (alpha, beta);

		this->first_delay = true;
		this->previous_delay = 0.0;

		if (symmetric){
			this->symmetry_factor = 2.0;
		}
		else{
			this->symmetry_factor = 1.0;
		}

		this->mean = this->alpha * this->beta;

		return;
	}

	void SetDelay(){
		if (!this->inverted){
			double d = (this->distribution(this->generator) + this->offset) / 1000.0; // divide by 1000 to convert ms to s
			if (d < 0.0){
					d = 0.0;
			}
			if (this->first_delay){
				this->delay = d;
				this->first_delay = false;
			}
			else if (d < this->previous_delay){
					this->delay = this->previous_delay - this->time_step;
			}
			else {
					this->delay = d;
			}
			this->previous_delay = this->delay;
		}
		else{
			double d = ((-1.0*this->distribution(this->generator)) + (2.0*this->mean) + this->offset) / 1000.0; // divide by 1000 to convert ms to s
			if (d < 0.0){
					d = 0.0;
			}
			if (this->first_delay){
				this->delay = d;
				this->first_delay = false;
			}
			else if (d < this->previous_delay){
					this->delay = this->previous_delay - this->time_step;
			}
			else {
					this->delay = d;
			}
			this->previous_delay = this->delay;
		}
	}


	void PublishDelay(ros::Time msgTime){
		this->delayMsg.header.stamp = msgTime;
		// If symmetric delay: The delay is divided in Sensorial and Motor delay --> divide by 2
		this->delayMsg.data = this->delay / this->symmetry_factor;

		this->delayPublisher.publish(this->delayMsg);

		this->delay_data.push_back(this->delay);

		return;
	}

	void WriteDataToFile(){
		std::ofstream file;
		file.open("/home/baxter/catkin_ws_WiFi/src/BaxterCerebellum/src/edlut_ros/rosbags/DelaysDistribution/test_distributionGamma_k3-5_theta_3-5_offset_data.txt");
		for (int i=0; i<this->delay_data.size(); i++){
			file << this->delay_data[i] << std::endl ;
		}
		file.close();
	}

};

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "delayGenerator_normalDistribution", ros::init_options::NoSigintHandler);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
			ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string delay_topic;
	bool sim_time, wireless_simulation, inverted, symmetric;
	double k, theta, offset;

	double sampling_frequency, checking_frequency, time_step;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("delay_topic", delay_topic);
	private_node_handle_.getParam("checking_frequency", checking_frequency);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("k", k);
	private_node_handle_.getParam("theta", theta);
	private_node_handle_.getParam("offset", offset);
	private_node_handle_.getParam("inverted", inverted);
	private_node_handle_.getParam("symmetric", symmetric);


	double alpha, beta;
	alpha = k;
	beta = theta;

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

	time_step = 1.0 / sampling_frequency;

	Delay objDelay = Delay(delay_topic, sim_time, alpha, beta, offset, inverted, symmetric, &nh, time_step);

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
				objDelay.SetDelay();
				objDelay.PublishDelay(time);
			}
			sim_rate.sleep();
		} else{
			// objDelay.SetDelay(delayParameters.GetFixedDelay());
			objDelay.SetDelay();
			objDelay.PublishDelay(ros::Time::now());
			rate.sleep();
		}
	}
	objDelay.WriteDataToFile();
	ROS_INFO("Ending Delay Generator node");

	ros::shutdown();
	return 0;
} // end main()
