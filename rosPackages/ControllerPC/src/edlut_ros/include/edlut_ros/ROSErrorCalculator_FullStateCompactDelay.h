/***************************************************************************
 *                 ROSErrorCalculator_FullStateCompactDelay.h              *
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

#ifndef ROSERRORCALCULATOR_FULLSTATECOMPACTDELAY_H_
#define ROSERRORCALCULATOR_FULLSTATECOMPACTDELAY_H_

#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>


#include <edlut_ros/AnalogCompact.h>
#include "edlut_ros/FullStateCompactDelay.h"



class analog_comparison {
public:
	bool operator() (const edlut_ros::FullStateCompactDelay lsp, const edlut_ros::FullStateCompactDelay rsp) const{
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
class ROSErrorCalculator_FullStateCompactDelay {
private:

	// ROS Node Handler
	ros::NodeHandle NodeHandler;

	// Desired position subscriber
	ros::Subscriber subscriber_full_state;

	// Output publisher positive and negative error
	ros::Publisher publisher;

	// ROS Callback queue
	ros::CallbackQueue CallbackQueue;

	// Queue of desired position analog signals not processed yet
	std::priority_queue<edlut_ros::FullStateCompactDelay, std::vector<edlut_ros::FullStateCompactDelay>, analog_comparison > activity_queue;

	// // Queue of actual position analog signals not processed yet
	// std::priority_queue<edlut_ros::AnalogCompact, std::vector<edlut_ros::AnalogCompact>, analog_comparison > activity_queue_pos_minus;

	// // Queue of desired velocity analog signals not processed yet
	// std::priority_queue<edlut_ros::AnalogCompact, std::vector<edlut_ros::AnalogCompact>, analog_comparison > activity_queue_vel_plus;

	// // Queue of actual velocity analog signals not processed yet
	// std::priority_queue<edlut_ros::AnalogCompact, std::vector<edlut_ros::AnalogCompact>, analog_comparison > activity_queue_vel_minus;

	// Desired position state
	std::vector<double> desired_position;

	// Desired velocity state
	std::vector<double> desired_velocity;

	// Current position state
	std::vector<double> current_position;

	// Current velocity state
	std::vector<double> current_velocity;

	// Position error gain
	std::vector<double> position_gain;

	// Velocity error gain
	std::vector<double> velocity_gain;

	// Joint list
	std::vector<std::string> joint_list;

	// Value of the output variable (positive and negative)
	std::vector<double> output_var;

	// Last time when the decoder has been updated
	double last_time;

	// Delay of the received message
	double msgDelay;


	// Callback function for reading input activity
	void FullStateCallback(const edlut_ros::FullStateCompactDelay::ConstPtr& msg);

	// Clean the input value queue and retrieve the last value
	void CleanQueue(std::priority_queue<edlut_ros::FullStateCompactDelay, std::vector<edlut_ros::FullStateCompactDelay>, analog_comparison > & queue,
			std::vector<double> & desired_position,
			std::vector<double> & desired_velocity,
			std::vector<double> & current_position,
			std::vector<double> & current_velocity,
			double msgDelay,
			double & lastTime,
			double end_time);

	// Find the index of name in vector strvector. -1 if not found
	int FindJointIndex(std::vector<std::string> strvector, std::string name);

public:

	/*
	 * This function initializes a spike decoder taking the activity from the specified ros topic.
	 */
	ROSErrorCalculator_FullStateCompactDelay(std::string full_state_topic,
			std::string output_topic_name,
			std::vector<std::string> joint_list,
			std::vector<double> position_gain,
			std::vector<double> velocity_gain);

	/*
	 * Update the output variables with the current time and the output activity and send the output message
	 */
	void UpdateError(ros::Time current_time);

	/*
	 * Destructor of the class
	 */
	virtual ~ROSErrorCalculator_FullStateCompactDelay();

};

#endif /* ROSERRORCALCULATOR_FULLSTATECOMPACTDELAY_H_ */
