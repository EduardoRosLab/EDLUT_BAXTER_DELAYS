/***************************************************************************
 *                           wireless_robot_state.cpp     		   *
 *                           -------------------                           *
 * copyright            : (C) 2019 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es 			           *
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
#include <edlut_ros/Time.h>
#include <cmath>



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
	ros::Subscriber delay_subscriber;
	ExternalClock ext_clock;
private:
	ros::Publisher robotState_publisher;

	ros::Publisher time_publisher;

	std::string output_topic;

	bool use_sim_time, wireless_simulation, discard_sample;

	std::vector<sensor_msgs::JointState> state_queue;
	std::vector<double> delay_queue;

	double last_delay = 0.0;

	double last_time_stamp = 0.0;

	double sampling_frequency, time_step;

public:
	RobotState (bool sim_time, std::string output_topic, bool wireless_simulation, double sampling_frequency):
		use_sim_time(sim_time), output_topic(output_topic), wireless_simulation(wireless_simulation), sampling_frequency(sampling_frequency) {
		ros::NodeHandle nh;
		this->time_step = 1.0 / this->sampling_frequency;
		this->robotState_publisher = nh.advertise<sensor_msgs::JointState>(output_topic, 10);

		return;
	}

	void EventCallback(const sensor_msgs::JointState::ConstPtr& msg){
		sensor_msgs::JointState robotStateMsg;
		ros::Time CurrentTime;
		if (!this->use_sim_time){
			// Round Current Time to ensure a 2ms time step between samples
			// If the rounded time_stamp is the same as the last one that was published, the sample is discarded
			double time_stamp = ros::Time::now().toSec();
			CurrentTime = ros::Time(round(this->sampling_frequency * time_stamp ) * this->time_step);
		}
		else{
			CurrentTime = this->ext_clock.GetLastConfirmedTime();
		}

			robotStateMsg.header.stamp = CurrentTime;

			robotStateMsg.name = msg->name;
			robotStateMsg.position = msg->position;
			robotStateMsg.velocity = msg->velocity;
			robotStateMsg.effort = msg->effort;

			// If it is a real wireless scenario publish the robot state right away
			if (!this->wireless_simulation){
				this->robotState_publisher.publish(robotStateMsg);
			}
			// If it is a simulated wireless scenario, append the robot state and the artificial delay to a buffer
			// and publish the robot state when the delay is over
			else{
				this->InsertandOrderBuffers(robotStateMsg, this->state_queue, last_delay, this->delay_queue);

				sensor_msgs::JointState top_value;
				double top_delay;
				top_value = this->state_queue.back();
				top_delay = this->delay_queue.back();

				while (!this->state_queue.empty() && (top_value.header.stamp.toSec() + top_delay <= CurrentTime.toSec()) ){
					this->robotState_publisher.publish(top_value);

					this->state_queue.pop_back();
					this->delay_queue.pop_back();

					if (!this->state_queue.empty()){
						top_value = this->state_queue.back();
						top_delay = this->delay_queue.back();
					}
				}
			}
		return;
	}

	void DelayCallback(const edlut_ros::Time::ConstPtr& msg){
		this->last_delay = msg->data;
		return;
	}


	// Order the Robot State Buffer according to time_stamps and delays.
	// The last element in the buffer will be the oldest (i.e. smallest message_time_stamp + associated_delay)
	void InsertandOrderBuffers(sensor_msgs::JointState & newMessage, std::vector<sensor_msgs::JointState>  & state_queue, double & newDelay, std::vector<double> & delay_queue){
		state_queue.insert(state_queue.begin(), newMessage);
		delay_queue.insert(delay_queue.begin(), newDelay);

		sensor_msgs::JointState aux_robotStateMsg;
		double aux_delay;

		if (state_queue.size()>1){
			int index = 1;
			while (index < (state_queue.size()) && (state_queue[index-1].header.stamp.toSec() + this->delay_queue[index-1]) < (state_queue[index].header.stamp.toSec() + this->delay_queue[index])){
					aux_robotStateMsg = this->state_queue[index];
					aux_delay = this->delay_queue[index];

					this->state_queue[index] = this->state_queue[index-1];
					this->delay_queue[index] = this->delay_queue[index-1];
					this->state_queue[index-1] = aux_robotStateMsg;
					this->delay_queue[index-1] = aux_delay;

					index++;
			}
		}
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
	std::string input_topic, output_topic, artificial_delay_topic;
	bool sim_time, wireless_simulation;
	ros::Publisher time_publisher;

	double sampling_frequency;


	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("input_topic", input_topic);
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("artificial_delay_topic", artificial_delay_topic);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);

	nh.getParam("use_sim_time", sim_time);
	nh.getParam("wireless_simulation", wireless_simulation);

	RobotState objRobotState = RobotState(sim_time, output_topic, wireless_simulation, sampling_frequency);

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

// In case of a simulated wireless scenario create a subscriber to receive the artificial delay generated at the
// Delay Generator Node.
 if (wireless_simulation){
	 objRobotState.delay_subscriber = nh.subscribe(artificial_delay_topic, 1000, &RobotState::DelayCallback, &objRobotState);
 }

	if (sim_time){
		//ROS_DEBUG("Wireless Robot State: Subscribing to topic /clock_sync");
		objRobotState.clock_subscriber = nh.subscribe("/clock_sync", 1000, &ExternalClock::ClockCallback, &(objRobotState.ext_clock));
	}

	//ROS_INFO("Wireless  Robot State node initialized: reading from topic %s and writing to %s", input_topic.c_str(), output_topic.c_str());

	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.003));
	}

	//ROS_INFO("Ending Wireless Robot State node");

	ros::shutdown();
	return 0;
} // end main()
