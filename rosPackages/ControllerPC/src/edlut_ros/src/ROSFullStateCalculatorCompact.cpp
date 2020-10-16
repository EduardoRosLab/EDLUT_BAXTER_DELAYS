/***************************************************************************
 *                           ROSFullStateCalculatorCompact.cpp             *
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

#include <vector>
#include <iostream>
#include <iterator>
#include <cmath>

#include "edlut_ros/ROSFullStateCalculatorCompact.h"
#include "edlut_ros/AnalogCompact.h"
#include "edlut_ros/FullStateCompactDelay.h"


void ROSFullStateCalculatorCompact::DesiredPositionCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg){
	// Check if the spike should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_desired_pos){
		ROS_WARN("Error Calculator: Positive position received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		edlut_ros::AnalogCompactDelay newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		newMessage.delay = msg->delay;
		this->activity_queue_desired_pos.push(newMessage);
	}
}

void ROSFullStateCalculatorCompact::CurrentPositionCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg){
	// Check if the spike should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_current_pos){
		ROS_WARN("Error Calculator: Current position received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		edlut_ros::AnalogCompactDelay newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		newMessage.delay = msg->delay;
		this->activity_queue_current_pos.push(newMessage);
	}
}

void ROSFullStateCalculatorCompact::DesiredVelocityCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg){
	// Check if the spike should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_desired_vel){
		ROS_WARN("Error Calculator: Desired velocity received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		edlut_ros::AnalogCompactDelay newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		newMessage.delay = msg->delay;
		this->activity_queue_desired_vel.push(newMessage);
	}
}

void ROSFullStateCalculatorCompact::CurrentVelocityCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg){
	// Check if the spike should have been processed previously
	if (msg->header.stamp.toSec()<this->last_time_current_vel){
		ROS_WARN("Error Calculator: Current velocity received from a past time. Discarded. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		edlut_ros::AnalogCompactDelay newMessage;
		newMessage.header.stamp = msg->header.stamp;
		newMessage.data = msg->data;
		newMessage.names = msg->names;
		newMessage.delay = msg->delay;
		this->activity_queue_current_vel.push(newMessage);
	}
}

int ROSFullStateCalculatorCompact::FindJointIndex(std::vector<std::string> strvector, std::string name){
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

ROSFullStateCalculatorCompact::ROSFullStateCalculatorCompact(
	std::string desired_position_topic,
	std::string desired_velocity_topic,
	std::string current_position_topic,
	std::string current_velocity_topic,
	std::string output_topic_name,
	std::vector<std::string> joint_list,
	double sensorial_delay,
	double sampling_frequency):
				NodeHandler(),
				CallbackQueue(),
				activity_queue_desired_pos(),
				activity_queue_current_pos(),
				activity_queue_desired_vel(),
				activity_queue_current_vel(),
				joint_list(joint_list),
				sensorial_delay(sensorial_delay),
				last_time_desired_pos(0.0),
				last_time_current_pos(0.0),
				last_time_desired_vel(0.0),
				last_time_current_vel(0.0),
				sampling_frequency(sampling_frequency)
{

	this->desired_position = std::vector<double> (joint_list.size(), 0.0);
	this->current_position = std::vector<double> (joint_list.size(), 0.0);
	this->desired_velocity = std::vector<double> (joint_list.size(), 0.0);
	this->current_velocity = std::vector<double> (joint_list.size(), 0.0);
	this->output_var = std::vector<double> (joint_list.size(), 0.0);

	this->currentState_delayMsg = 0.0;
	this->desiredState_delayMsg = 0.0;


	this->NodeHandler.setCallbackQueue(&this->CallbackQueue);
	this->subscriber_desired_position = this->NodeHandler.subscribe(desired_position_topic, 10.0, &ROSFullStateCalculatorCompact::DesiredPositionCallback, this);
	this->subscriber_current_position = this->NodeHandler.subscribe(current_position_topic, 10.0, &ROSFullStateCalculatorCompact::CurrentPositionCallback, this);
	this->subscriber_desired_velocity = this->NodeHandler.subscribe(desired_velocity_topic, 10.0, &ROSFullStateCalculatorCompact::DesiredVelocityCallback, this);
	this->subscriber_current_velocity = this->NodeHandler.subscribe(current_velocity_topic, 10.0, &ROSFullStateCalculatorCompact::CurrentVelocityCallback, this);
	this->publisher = this->NodeHandler.advertise<edlut_ros::FullStateCompactDelay>(output_topic_name, 1.0);

	this->time_step = 1.0 / sampling_frequency;
}

ROSFullStateCalculatorCompact::~ROSFullStateCalculatorCompact() {
	// TODO Auto-generated destructor stub
}

void ROSFullStateCalculatorCompact::UpdateError(ros::Time current_time){

	// Process all the msgs in the queue
	this->CallbackQueue.callAvailable();

	double end_time = current_time.toSec();
	double compensateDelay_current = 0.0;
	double compensateDelay_desired = this->sensorial_delay;

	this->CleanQueue(this->activity_queue_current_pos, this->current_position, this->currentState_delayMsg, compensateDelay_current, this->last_time_current_pos, end_time);
	this->CleanQueue(this->activity_queue_current_vel, this->current_velocity, this->currentState_delayMsg, compensateDelay_current, this->last_time_current_vel, end_time);


	compensateDelay_desired += this->currentState_delayMsg;
	this->CleanQueue(this->activity_queue_desired_pos, this->desired_position, this->desiredState_delayMsg, compensateDelay_desired, this->last_time_desired_pos, end_time);
	this->CleanQueue(this->activity_queue_desired_vel, this->desired_velocity, this->desiredState_delayMsg, compensateDelay_desired, this->last_time_desired_vel, end_time);


	// Create the message and publish it
	edlut_ros::FullStateCompactDelay newMsg;
	newMsg.header.stamp = current_time;
	newMsg.desired_position = this->desired_position;
	newMsg.current_position = this->current_position;
	newMsg.desired_velocity = this->desired_velocity;
	newMsg.current_velocity = this->current_velocity;
	newMsg.names = this->joint_list;
	newMsg.delay = this->currentState_delayMsg;

	this->publisher.publish(newMsg);

	return;
}

void ROSFullStateCalculatorCompact::CleanQueue(std::priority_queue<edlut_ros::AnalogCompactDelay, std::vector<edlut_ros::AnalogCompactDelay>, analog_comparison > & queue,
		std::vector<double> & updateVar,
		double & updateDelay,
		double compensateDelay,
		double & lastTime,
		double end_time){
	edlut_ros::AnalogCompactDelay top_value;

	// Round time to 2ms. Since the samples are generated with a 2ms time stamp
	double time = round(this->sampling_frequency * (end_time-compensateDelay)) * this->time_step;

	// Clean the queue (just in case any message has to be discarded in the queue)
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
		// while (!(queue.empty()) && top_value.header.stamp.toSec()<= (end_time-compensateDelay)){
		while (!(queue.empty()) && top_value.header.stamp.toSec()<= time){
			for (unsigned int i=0; i<top_value.names.size(); ++i){
				int index = this->FindJointIndex(this->joint_list, top_value.names[i]);

				if (index!=-1) {
					updateVar[index] = top_value.data[i];
				}
			}
			updateDelay = top_value.delay;
			lastTime = top_value.header.stamp.toSec();
			queue.pop();
			if (!queue.empty()){
				top_value = queue.top();
			}
		}
		// ROS_INFO("TIME_STAMP %f =  || msgDelay = %f", lastTime, updateDelay);
	}
}
