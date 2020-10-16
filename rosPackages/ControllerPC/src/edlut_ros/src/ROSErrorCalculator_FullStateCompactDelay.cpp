/***************************************************************************
 *              ROSErrorCalculator_FullStateCompactDelay.cpp               *
 *                           -------------------                           *
 * copyright            : (C) 2019 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es           			                   *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// This node computes the error comparing the current position and velocity
// topics. Eeach joint's position and velocity has a gain associated for the
// computation of the error.

#include <vector>
#include <iostream>
#include <iterator>
#include <cmath>

#include "edlut_ros/ROSErrorCalculator_FullStateCompactDelay.h"
#include "edlut_ros/AnalogCompact.h"
#include "edlut_ros/AnalogCompactDelay.h"
#include "edlut_ros/FullStateCompactDelay.h"


void ROSErrorCalculator_FullStateCompactDelay::FullStateCallback(const edlut_ros::FullStateCompactDelay::ConstPtr& msg){
	// Check if the state should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time){
		ROS_WARN("Error Calculator: Positive position received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		edlut_ros::FullStateCompactDelay newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.desired_position = msg->desired_position;
		newMessage.current_position = msg->current_position;
		newMessage.desired_velocity = msg->desired_velocity;
		newMessage.current_velocity = msg->current_velocity;
		newMessage.names = msg->names;
		newMessage.delay = msg->delay;
		this->activity_queue.push(newMessage);

		// for (unsigned int i=0; i<msg->names.size(); ++i){
		// 	int index = this->FindJointIndex(this->joint_list, msg->names[i]);
		//
		// 	if (index!=-1) {
		// 		this->desired_position[index] = msg->desired_position[i];
		// 		this->desired_velocity[index] = msg->desired_velocity[i];
		// 		this->current_position[index] = msg->current_position[i];
		// 		this->current_velocity[index] = msg->current_velocity[i];
		// 	}
		// }
		// this->msgDelay = msg->delay;
		//
		// this->last_time = msg->header.stamp.toSec();

		// edlut_ros::AnalogCompactDelay newMessage;
		// newMessage.header.stamp = msg->header.stamp;
		// newMessage.data = msg->data;
		// newMessage.names = msg->names;
		// this->activity_queue_pos_plus.push(newMessage);
	}
}


int ROSErrorCalculator_FullStateCompactDelay::FindJointIndex(std::vector<std::string> strvector, std::string name){
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

ROSErrorCalculator_FullStateCompactDelay::ROSErrorCalculator_FullStateCompactDelay(
	std::string full_state_topic,
	std::string output_topic_name,
	std::vector<std::string> joint_list,
	std::vector<double> position_gain,
	std::vector<double> velocity_gain):
				NodeHandler(),
				CallbackQueue(),
				joint_list(joint_list),
				position_gain(position_gain),
				velocity_gain(velocity_gain),
				last_time(0.0)
{
	this->desired_position = std::vector<double> (joint_list.size(), 0.0);
	this->current_position = std::vector<double> (joint_list.size(), 0.0);
	this->desired_velocity = std::vector<double> (joint_list.size(), 0.0);
	this->current_velocity = std::vector<double> (joint_list.size(), 0.0);
	this->output_var = std::vector<double> (joint_list.size(), 0.0);

	this->NodeHandler.setCallbackQueue(&this->CallbackQueue);
	this->subscriber_full_state = this->NodeHandler.subscribe(full_state_topic, 10.0, &ROSErrorCalculator_FullStateCompactDelay::FullStateCallback, this);
	this->publisher = this->NodeHandler.advertise<edlut_ros::AnalogCompactDelay>(output_topic_name, 1.0);
}

ROSErrorCalculator_FullStateCompactDelay::~ROSErrorCalculator_FullStateCompactDelay() {
	// TODO Auto-generated destructor stub
}

void ROSErrorCalculator_FullStateCompactDelay::UpdateError(ros::Time current_time){

	// Process all the messages in the queue
	this->CallbackQueue.callAvailable();

	double end_time = current_time.toSec();

	this->CleanQueue(this->activity_queue, this->desired_position, this->desired_velocity,
		this->current_position, this->current_velocity, this->msgDelay, this->last_time, end_time);

	std::vector<double> ErrorPos(this->joint_list.size());
	std::vector<double> ErrorVel(this->joint_list.size());
	for (unsigned int i=0; i<this->joint_list.size(); ++i){
		ErrorPos[i] = this->desired_position[i] - this->current_position[i];
		ErrorVel[i] = this->desired_velocity[i] - this->current_velocity[i];
		this->output_var[i] = this->position_gain[i]*ErrorPos[i] + this->velocity_gain[i]*ErrorVel[i];
	}

	// Create the message and publish it
	edlut_ros::AnalogCompactDelay newMsg;
	newMsg.header.stamp = current_time;
	newMsg.names = this->joint_list;
	newMsg.data = this->output_var;
	newMsg.delay = this->msgDelay;
	this->publisher.publish(newMsg);

	return;
}

void ROSErrorCalculator_FullStateCompactDelay::CleanQueue(std::priority_queue<edlut_ros::FullStateCompactDelay, std::vector<edlut_ros::FullStateCompactDelay>, analog_comparison > & queue,
		std::vector<double> & desired_position,
		std::vector<double> & desired_velocity,
		std::vector<double> & current_position,
		std::vector<double> & current_velocity,
		double msgDelay,
		double & lastTime,
		double end_time){

	edlut_ros::FullStateCompactDelay top_value;
	// Clean the queue (just in case any spike has to be discarded in the queue)
	if (!queue.empty()){
		top_value= queue.top();
		while (!(queue.empty()) && top_value.header.stamp.toSec()<lastTime){
			queue.pop();
			ROS_WARN("Error calculator: Discarded analog value from queue with time %f. Current time: %f", top_value.header.stamp.toSec(), ros::Time::now().toSec());
			if (!queue.empty()){
				top_value = queue.top();
			}
		}
	}

	if (!queue.empty()){
		while (!(queue.empty()) && top_value.header.stamp.toSec()<=end_time){
			for (unsigned int i=0; i<top_value.names.size(); ++i){
				int index = this->FindJointIndex(this->joint_list, top_value.names[i]);

				if (index!=-1) {
					desired_position[index] = top_value.desired_position[i];
					desired_velocity[index] = top_value.desired_velocity[i];
					current_position[index] = top_value.current_position[i];
					current_velocity[index] = top_value.current_velocity[i];
				}
			}
			msgDelay = top_value.delay;
			lastTime = top_value.header.stamp.toSec();
			queue.pop();
			if (!queue.empty()){
				top_value = queue.top();
			}
		}
	}
}
