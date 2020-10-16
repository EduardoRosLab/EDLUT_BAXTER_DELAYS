/***************************************************************************
 *                           baxter_arm_state_node_sync.cpp                *
 *                           -------------------                           *
 * copyright            : (C) 2016 by Jesus Garrido                        *
 * email                : jesusgarrido@ugr.es                              *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// This node gets Baxter's current state (position and velocity) and
// publishes that sensorial information into custom topics

#include <edlut_ros/Analog.h>
#include <edlut_ros/AnalogCompact.h>
#include <edlut_ros/AnalogCompactDelay.h>
#include <edlut_ros/Time.h>
#include <sensor_msgs/JointState.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <cstring>
#include <ctime>
#include <limits>
#include <signal.h>
#include "edlut_ros/ExternalClock.h"



static bool stop_node;

void rosShutdownHandler(int sig)
{
	stop_node = true;
}

/*
 * Create a class
 */
class Splitter {
public:
	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;

	ros::Subscriber delay_subscriber;
private:
	ros::Publisher position_joint_publisher;

	ros::Publisher velocity_joint_publisher;

	ros::Publisher sensorial_delay_publisher;

	std::vector<std::string> joint_list;

	double delay, last_delay, sampling_frequency, time_step;
	double last_time;
	std::vector<double> delays;

	edlut_ros::AnalogCompactDelay msgPosition, msgVelocity;

	bool use_sim_time, wireless_simulation;

	std::vector<edlut_ros::AnalogCompactDelay> buffer_msgPosition;
	std::vector<edlut_ros::AnalogCompactDelay> buffer_msgVelocity;


	int FindJointIndex(std::vector<std::string> strvector, std::string name){
		std::vector<std::string>::iterator first = strvector.begin();
		std::vector<std::string>::iterator last = strvector.end();
		unsigned int index = 0;
		bool found = false;

		while (first!=last && !found) {
			if (*first==name)
				found = true;
			else {
				++first;
				++index;
			}
		}

		if (found) {
			return index;
		} else {
			return -1;
		}
	};

public:
	Splitter (std::vector<std::string> & joint_list,
		std::string position_joint_topic,
		std::string velocity_joint_topic,
		bool sim_time,
		bool wireless_simulation,
		std::string sensorial_delay_topic,
		double sampling_frequency):
			joint_list(joint_list), use_sim_time(sim_time), wireless_simulation(wireless_simulation), sampling_frequency(sampling_frequency) {

		ros::NodeHandle nh;

		this->position_joint_publisher = nh.advertise<edlut_ros::AnalogCompactDelay>(position_joint_topic, 1);
		ROS_DEBUG("Joint State Splitter: Writing position to topic %s", position_joint_topic.c_str());

		this->velocity_joint_publisher = nh.advertise<edlut_ros::AnalogCompactDelay>(velocity_joint_topic, 1);
		ROS_DEBUG("Joint State Splitter: Writing velocity to topic %s", velocity_joint_topic.c_str());

		this->sensorial_delay_publisher = nh.advertise<edlut_ros::Analog>(sensorial_delay_topic, 1);
		ROS_DEBUG("Joint State Splitter: Writing sensorial delay to topic %s", sensorial_delay_topic.c_str());

		msgPosition.names = msgVelocity.names = this->joint_list;
		msgPosition.data.resize(this->joint_list.size());
		msgVelocity.data.resize(this->joint_list.size());

		// this->delay = this->last_delay = this->last_time = 0.0;

		this->time_step = 1.0 / sampling_frequency;

		return;
	};

	// void SetDelay(double time_delay){
	// 	msgPosition.delay = msgVelocity.delay = time_delay;
	// 	return;
	// }

	// void DelayCallback(const edlut_ros::Time::ConstPtr& msg){
	// 	this->delay = msg->data;
	// 	return;
	// }

	void EventCallback(const sensor_msgs::JointState::ConstPtr& msg){
		// Get current time
		ros::Time CurrentTime;
		if (this->use_sim_time){
			CurrentTime = this->ext_clock.GetLastConfirmedTime();
		}
		else{
			CurrentTime = ros::Time::now();
		}

		// Compute the transmission delay of the received message
		// double delayMsg = round(sampling_frequency * ((CurrentTime - msg->header.stamp).toSec())) * inverse_sampling_frequency;
		double delayMsg = (CurrentTime - msg->header.stamp).toSec();
		if (delayMsg>0){
			msgPosition.delay = msgVelocity.delay = delayMsg;
		}
		else{
			msgPosition.delay = msgVelocity.delay = 0.0;
		}


		msgPosition.header.stamp = msgVelocity.header.stamp = msg->header.stamp;
		for (unsigned int i=0; i<msg->name.size(); ++i){
			int index = this->FindJointIndex(this->joint_list, msg->name[i]);
			if (index!=-1) {
				msgPosition.data[index] = msg->position[i];
				msgVelocity.data[index] = msg->velocity[i];
			}
		}
		this->InsertAndInterpolate();

		// Publish the current sensorial delay
		edlut_ros::Analog SensorialDelayMsg;
		SensorialDelayMsg.header.stamp = msgPosition.header.stamp;
		SensorialDelayMsg.data = msgPosition.delay;
		this->sensorial_delay_publisher.publish(SensorialDelayMsg);

		return;
	};

	// This function stores the sensorial data in a buffer and interpolate the missing samples, if any.
	// The samples must follow a time step of 2ms. If the distance between two samples is greater than 2ms,
	// the missing samples are interpolated.
	void InsertAndInterpolate(){
		this->buffer_msgPosition.insert(this->buffer_msgPosition.begin(), this->msgPosition);
		this->buffer_msgVelocity.insert(this->buffer_msgVelocity.begin(), this->msgVelocity);
		if (this->buffer_msgPosition.size() > 1 ){
		 	double time_distance = this->buffer_msgPosition[0].header.stamp.toSec() - this->buffer_msgPosition[1].header.stamp.toSec();
			if (time_distance > this->time_step+0.0001){

				int missing_samples = round(time_distance / this->time_step) - 1;

				for (int i=missing_samples; i>=1; i--){
					edlut_ros::AnalogCompactDelay missingMsgPosition;
					edlut_ros::AnalogCompactDelay missingMsgVelocity;

					//Set time stamp of the missing samples
					missingMsgPosition.header.stamp = missingMsgVelocity.header.stamp = ros::Time(this->msgPosition.header.stamp.toSec() - this->time_step*i);
					//Set joint names of the missing samples
					missingMsgPosition.names = missingMsgVelocity.names = this->msgPosition.names;
					//Interpolate data of the missing samples
					missingMsgPosition.data.resize(this->joint_list.size());
					missingMsgVelocity.data.resize(this->joint_list.size());

					for (int j=0; j<msgPosition.data.size(); j++){
						missingMsgPosition.data[j] = this->buffer_msgPosition.front().data[j] + (this->buffer_msgPosition.front().data[j] - this->buffer_msgPosition.back().data[j]) * (i / (missing_samples+1) );
						missingMsgVelocity.data[j] = this->buffer_msgVelocity.front().data[j] + (this->buffer_msgVelocity.front().data[j] - this->buffer_msgVelocity.back().data[j]) * (i / (missing_samples+1) );
					}

					// Insert the missing samples in the buffer
					this->buffer_msgPosition.insert(this->buffer_msgPosition.begin() + 1, missingMsgPosition);
					this->buffer_msgVelocity.insert(this->buffer_msgVelocity.begin() + 1, missingMsgVelocity);

				}

			}
		}

	}

	void PublishState(){
		// Publish the current values stored in the buffer. The last element in the buffer
		// is the sample that was published in the previous step. That's why we start publishing
		// from the last - 1 sample.
		for (int i=this->buffer_msgPosition.size() - 1; i>=1; i--){
				this->position_joint_publisher.publish(this->buffer_msgPosition[i-1]);
				this->velocity_joint_publisher.publish(this->buffer_msgVelocity[i-1]);
				this->buffer_msgPosition.pop_back();
				this->buffer_msgVelocity.pop_back();
		}
		return;
	};
};

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "baxter_arm_state", ros::init_options::NoSigintHandler);
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
			ros::console::notifyLoggerLevelsChanged();
	}
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string input_topic, position_joint_topic, velocity_joint_topic, clock_topic, artificial_delay_topic, sensorial_delay_topic;
	std::vector<std::string> joint_list;
	bool sim_time, wireless_simulation;
	double checking_frequency, sampling_frequency;
	ros::Publisher time_publisher;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("input_topic", input_topic);
	private_node_handle_.getParam("joint_list", joint_list);
	private_node_handle_.getParam("current_position_topic", position_joint_topic);
	private_node_handle_.getParam("current_velocity_topic", velocity_joint_topic);
	private_node_handle_.getParam("checking_frequency", checking_frequency);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("clock_topic", clock_topic);
	// private_node_handle_.getParam("artificial_delay_topic", artificial_delay_topic);
	private_node_handle_.getParam("sensorial_delay_topic", sensorial_delay_topic);


	nh.getParam("use_sim_time", sim_time);
	nh.getParam("wireless_simulation", wireless_simulation);


	Splitter objSplitter = Splitter(joint_list, position_joint_topic, velocity_joint_topic, sim_time, wireless_simulation, sensorial_delay_topic, sampling_frequency);


	// Create the subscriber
	ros::CallbackQueue CallbackQueue;
	nh.setCallbackQueue(&CallbackQueue);

	ros::SubscribeOptions baxterStateOptions =
	 ros::SubscribeOptions::create<sensor_msgs::JointState>(
	 input_topic, 1000, boost::bind(&Splitter::EventCallback, &objSplitter, _1), ros::VoidPtr(), &CallbackQueue);

 // Because TCP causes bursty communication with high jitter,
 // declare a preference on UDP connections for receiving
 // joint states, which we want to get at a high rate.
 // Note that we'll still accept TCP connections for this topic
 // (e.g., from rospy nodes, which don't support UDP);
 // we just prefer UDP.
 baxterStateOptions.transport_hints = ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

 ros::Subscriber input_subscriber= nh.subscribe(baxterStateOptions);

 ros::Rate rate(sampling_frequency);
 ros::Time time(0.0), last_sent_time(0.0);
 ros::WallRate sim_rate(checking_frequency);
 ros::WallRate init_rate(1.0);

	// if (wireless_simulation){
	// 	objSplitter.delay_subscriber = nh.subscribe(artificial_delay_topic, 1000, &Splitter::DelayCallback, &objSplitter);
	// }

	if (sim_time){
		ROS_DEBUG("Baxter Arm State: Subscribing to topic /clock_sync");
		objSplitter.clock_subscriber = nh.subscribe("/clock_sync", 1000, &ExternalClock::ClockCallback, &(objSplitter.ext_clock));
		time_publisher  = nh.advertise<rosgraph_msgs::Clock>(clock_topic, 1000);

		while(!objSplitter.ext_clock.FirstReceived()){
			ROS_DEBUG("Baxter Arm State: Synchronizing");
			CallbackQueue.callAvailable(ros::WallDuration(0.001));
			rosgraph_msgs::Clock current_time;
			current_time.clock = ros::Time(0.0);
			ROS_DEBUG("Baxter Arm State: Publishing simulation time %f", time.toSec());
			time_publisher.publish(current_time);
			init_rate.sleep();
		}
		time = objSplitter.ext_clock.GetLastConfirmedTime();
		ROS_DEBUG("Baxter Arm State: Node synchronized");
	}

	ROS_INFO("Baxter Arm State node initialized: reading from topic %s and writing position to %s, velocity to %s",
			input_topic.c_str(), position_joint_topic.c_str(), velocity_joint_topic.c_str());

	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.003));

		if (sim_time){
			ros::Time new_time = objSplitter.ext_clock.GetLastConfirmedTime();
			if (new_time>time){
				time = new_time;
				objSplitter.PublishState();
			}
			if (time!=last_sent_time){
				// Publish the simulation time
				rosgraph_msgs::Clock current_time;
				current_time.clock = time;
				ROS_DEBUG("BAS: Publishing simulation time %fs", time.toSec());
				time_publisher.publish(current_time);
				last_sent_time = time;
			}
			sim_rate.sleep();

		} else{
			// // If in a simulated wireless scenario: Generate a transmission delay of the
			// // received message and set it as the message delay to be published forward.
			// if (wireless_simulation){
			// 	objSplitter.SetDelay(objDelayGenerator_RandomTCP.GenerateDelayRandomTCP(ros::Time::now().toSec()));
			// }
			objSplitter.PublishState();

			// rate.sleep();
		}
	}

	ROS_INFO("Ending State Splitter node");

	ros::shutdown();
	return 0;
} // end main()
