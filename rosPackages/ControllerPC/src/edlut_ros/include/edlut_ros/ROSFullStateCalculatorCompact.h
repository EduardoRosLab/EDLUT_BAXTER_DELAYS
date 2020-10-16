/***************************************************************************
 *                           ROFullStateCalculatorCompact.h                *
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

 // This node computes an ouput message containing the desired position and velocity
 // and the associated actual position and velocity. To do so it takes into account the
 // transmission delays related to the sensorial information sent from the robot to
 // the computer.

#ifndef ROSFULLSTATECALCULATORCOMPACT_H_
#define ROSFULLSTATECALCULATORCOMPACT_H_

#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>


#include <edlut_ros/AnalogCompact.h>
#include <edlut_ros/AnalogCompactDelay.h>



class analog_comparison {
public:
	bool operator() (const edlut_ros::AnalogCompactDelay lsp, const edlut_ros::AnalogCompactDelay rsp) const{
		if (lsp.header.stamp>rsp.header.stamp) {
			return true;
		} else {
			return false;
		}
	}
};

/*
 * This class defines a spike-to-analog decoder.
 */
class ROSFullStateCalculatorCompact {
private:

	// ROS Node Handler
	ros::NodeHandle NodeHandler;

	// Desired position subscriber
	ros::Subscriber subscriber_desired_position;

	// Actual position subscriber
	ros::Subscriber subscriber_current_position;

	// Desired velocity subscriber
	ros::Subscriber subscriber_desired_velocity;

	// Actual velocity subscriber
	ros::Subscriber subscriber_current_velocity;

	// Output publisher positive and negative error
	ros::Publisher publisher;

	// ROS Callback queue
	ros::CallbackQueue CallbackQueue;

	// Queue of desired position analog signals not processed yet
	std::priority_queue<edlut_ros::AnalogCompactDelay, std::vector<edlut_ros::AnalogCompactDelay>, analog_comparison > activity_queue_desired_pos;

	// Queue of actual position analog signals not processed yet
	std::priority_queue<edlut_ros::AnalogCompactDelay, std::vector<edlut_ros::AnalogCompactDelay>, analog_comparison > activity_queue_current_pos;

	// Queue of desired velocity analog signals not processed yet
	std::priority_queue<edlut_ros::AnalogCompactDelay, std::vector<edlut_ros::AnalogCompactDelay>, analog_comparison > activity_queue_desired_vel;

	// Queue of actual velocity analog signals not processed yet
	std::priority_queue<edlut_ros::AnalogCompactDelay, std::vector<edlut_ros::AnalogCompactDelay>, analog_comparison > activity_queue_current_vel;

	// Joint list
	std::vector<std::string> joint_list;

	// Current values
	std::vector<double> desired_position, current_position, desired_velocity, current_velocity;

	// Value of the output variable (positive and negative)
	std::vector<double> output_var;

	// Value of the transmission delay of the message
	double currentState_delayMsg;
	double desiredState_delayMsg;

	// Sampling frequency
	double sampling_frequency;

	// Time step between samples
	double time_step;

	// Sensorial delay to correlate desired to current values (Current value at t
	// corresponds to desired value at t-sensorial_delay)
	double sensorial_delay;


	// Last time when the decoder has been updated
	double last_time_desired_pos, last_time_desired_vel, last_time_current_pos, last_time_current_vel;

	// Callback function for reading input activity
	void DesiredPositionCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg);

	// Callback function for reading input activity
	void DesiredVelocityCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg);

	// Callback function for reading input activity
	void CurrentPositionCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg);

	// Callback function for reading input activity
	void CurrentVelocityCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg);

	// Clean the input value queue and retrieve the last value
	void CleanQueue(std::priority_queue<edlut_ros::AnalogCompactDelay, std::vector<edlut_ros::AnalogCompactDelay>, analog_comparison > & queue,
			std::vector<double> & updateVar,
			double & updateDelay,
			double compensateDelay,
			double & lastTime,
			double end_time);

	// Find the index of name in vector strvector. -1 if not found
	int FindJointIndex(std::vector<std::string> strvector, std::string name);

public:

	/*
	 * This function initializes a spike decoder taking the activity from the specified ros topic.
	 */
	ROSFullStateCalculatorCompact(std::string desired_position_topic,
			std::string desired_velocity_topic,
			std::string current_position_topic,
			std::string current_velocity_topic,
			std::string output_topic_name,
			std::vector<std::string> joint_list,
			double sensorial_delay,
			double sampling_frequency);

	/*
	 * Update the output variables with the current time and the output activity and send the output message
	 */
	void UpdateError(ros::Time current_time);

	/*
	 * Destructor of the class
	 */
	virtual ~ROSFullStateCalculatorCompact();

};

#endif /* ROSFULLSTATECALCULATORCOMPACT_H_ */
