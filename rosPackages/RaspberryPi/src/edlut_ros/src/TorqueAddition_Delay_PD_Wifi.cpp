/***************************************************************************
 *                           TorqueAddition_Delay_PD_Wifi.cpp							 *
 *                           -------------------                           *
 * copyright            : (C) 2019 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es		                               *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

// This node adds the cerebellar torque to the PD supervisor torque and sends
// the resulting motor command to Baxter.

#include <vector>
#include <iostream>
#include <iterator>
#include <cmath>
#include <algorithm>

#include <edlut_ros/TorqueAddition_Delay_PD_Wifi.h>
#include <edlut_ros/AnalogCompact.h>
#include <edlut_ros/AnalogCompactDelay.h>
#include <edlut_ros/Analog.h>
#include <baxter_core_msgs/JointCommand.h>
#include <std_msgs/Empty.h>


// Callback for cerebellar torque commands
void TorqueAddition_Delay_PD_Wifi::TorqueCerebellumCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg){
	ros::Time current_time = ros::Time::now();

	if (!this->first_msg){
		this->first_msg = true;
		this->init_time_stamp = msg->header.stamp.toSec();
	}
	edlut_ros::AnalogCompact newMessage;
	// if (wireless_simulation){
	// 	// WHAT TO DO HERE???? Change time stamp here?? (don't like it). Create buffer as in wireless_robot_state here or in spike_decoder_node_compact_future?
	// 	newMessage.header.stamp = msg->header.stamp + ros::Duration(msg->delay);
	// }
	// else{
		newMessage.header.stamp = msg->header.stamp;
		// Compute the motor delay (time spent since the generation and reception of the motor command). msg->delay is the generation time of the torque command
		// this->motor_delayMsg.data = current_time.toSec() - msg->delay;
		// this->motor_delayMsg.header.stamp = ros::Time::now();
		// this->motor_delay_publisher.publish(this->motor_delayMsg);
// }
	newMessage.data = msg->data;
	newMessage.names = msg->names;

	// Check if the received torque command should have been already processed.
	// If it is a past sample then it is stored in the past buffer. Otherwise it
	// is stored in the future buffer.
	if (newMessage.header.stamp.toSec() < current_time.toSec()){
		this->InserAndOrderCerebellarBufferPast(newMessage, this->past_cerebellar_torque_buffer);
		if (this->past_cerebellar_torque_buffer.size()>this->max_past_buffer_size){
			this->past_cerebellar_torque_buffer.pop_back();
		}
	} else {
		this->InserAndOrderCerebellarBufferFuture(newMessage, this->future_cerebellar_torque_buffer);
	}
}

// Order the torque buffer. The last element is the oldest motor command.
void TorqueAddition_Delay_PD_Wifi::InserAndOrderCerebellarBufferFuture(edlut_ros::AnalogCompact & newMessage, std::vector<edlut_ros::AnalogCompact>  & buffer){
  //insert new message in first position
	buffer.insert(buffer.begin(), newMessage);

  //reorder the buffer just for the new inserted element
	edlut_ros::AnalogCompact aux;
	if (buffer.size()>1){
		int index = 1;
		while (index < (buffer.size()) && buffer[index-1].header.stamp.toSec() < buffer[index].header.stamp.toSec() ){
			aux = buffer[index];
			buffer[index] = buffer[index-1];
			buffer[index-1] = aux;
			index++;
		}
	}
}

// Order the torque buffer. The last element is the oldest motor command.
void TorqueAddition_Delay_PD_Wifi::InserAndOrderCerebellarBufferPast(edlut_ros::AnalogCompact & newMessage, std::vector<edlut_ros::AnalogCompact>  & buffer){
  //insert new message in first position
	buffer.insert(buffer.begin(), newMessage);

  //reorder the buffer just for the new inserted element
	edlut_ros::AnalogCompact aux;
	if (buffer.size()>1){
		int index = 1;
		while (index < (buffer.size()) && buffer[index-1].header.stamp.toSec() < buffer[index].header.stamp.toSec() ){
			aux = buffer[index];
			buffer[index] = buffer[index-1];
			buffer[index-1] = aux;
			index++;
		}
	}
}

// Callback for torque commands from the PD Supervisor
void TorqueAddition_Delay_PD_Wifi::TorqueSupervisorCallback(const edlut_ros::AnalogCompact::ConstPtr& msg){
	if (!this->first_msg){
		this->first_msg = true;
		this->init_time_stamp = msg->header.stamp.toSec();
	}

	ros::Time current_time = ros::Time::now();
	// Compute the motor delay
	this->motor_delayMsg.data = current_time.toSec() - msg->header.stamp.toSec();
	this->motor_delayMsg.header.stamp = ros::Time::now();
	this->motor_delay_publisher.publish(this->motor_delayMsg);

	// std::ostringstream oss;
  // std::copy(msg->data.begin(), msg->data.end(), std::ostream_iterator<double>(oss, ","));
	// ROS_INFO("Received torque supervisor: time %f and value %s.", msg->header.stamp.toSec(), oss.str().c_str());
	for (unsigned int i=0; i<msg->names.size(); ++i){
		int index = this->FindJointIndex(this->joint_list, msg->names[i]);
		if (index!=-1) {
			this->torque_supervisor[index] = msg->data[i];
		}
	}
	this->UpdateTorque(ros::Time::now());
}

int TorqueAddition_Delay_PD_Wifi::FindJointIndex(std::vector<std::string> strvector, std::string name){
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

TorqueAddition_Delay_PD_Wifi::TorqueAddition_Delay_PD_Wifi(
	bool wireless_simulation,
	std::string torque_cerebellum_topic_name,
	std::string torque_supervisor_topic_name,
	std::string output_topic_name,
	std::vector<std::string> joint_list,
	int max_past_buffer_size,
	int min_future_samples,
	double T_iteration,
	int samples_iteration,
	int N_iterations,
	bool disable_gravity_compensation,
	std::string gravity_topic,
	ros::NodeHandle * NodeHandler,
	ros::CallbackQueue * CallbackQueue,
	std::string motor_delay_topic,
	double time_step,
	std::string iteration_filter_size_topic,
	std::string future_buffer_samples_topic,
	double torque_reduction_factor):
				wireless_simulation(wireless_simulation),
				joint_list(joint_list),
				max_past_buffer_size(max_past_buffer_size),
				min_future_samples(min_future_samples),
				T_iteration(T_iteration),
				samples_iteration(samples_iteration),
				N_iterations(N_iterations),
				last_time_torque_supervisor(0.0),
				disable_gravity_compensation(disable_gravity_compensation),
				NodeHandler(NodeHandler),
				CallbackQueue (CallbackQueue),
				time_step(time_step),
				torque_reduction_factor(torque_reduction_factor)
{
	this->torque_cerebellum = std::vector<double> (joint_list.size(), 0.0);
	this->last_torque_cerebellum = std::vector<double> (joint_list.size(), 0.0);
	this->torque_supervisor = std::vector<double> (joint_list.size(), 0.0);
	this->output_torque  = std::vector<double> (joint_list.size(), 0.0);
	this->mean_torque = std::vector<double> (joint_list.size(), 0.0);
	this->dif_value = std::vector<double>(joint_list.size(), 0.0);

	this->backup_copy_size = this->N_iterations * this->samples_iteration;
	this->T_step = this->T_iteration / this->samples_iteration;

	this->torque_backup_copy = std::vector<std::vector <double> > (this->backup_copy_size, std::vector<double> (joint_list.size(), 0.0));


ros::SubscribeOptions subscriberOptions = ros::SubscribeOptions::create<edlut_ros::AnalogCompactDelay>(
	torque_cerebellum_topic_name, 10.0, boost::bind(&TorqueAddition_Delay_PD_Wifi::TorqueCerebellumCallback, this, _1), ros::VoidPtr(), this->CallbackQueue);

subscriberOptions.transport_hints = ros::TransportHints().unreliable().reliable().tcpNoDelay(true);
//subscriberOptions.transport_hints = ros::TransportHints().tcp();
this->subscriber_torque_cerebellum = this->NodeHandler->subscribe(subscriberOptions);


	//this->subscriber_torque_cerebellum = this->NodeHandler->subscribe(torque_cerebellum_topic_name, 10.0, &TorqueAddition_Delay_PD_Wifi::TorqueCerebellumCallback, this);
	this->subscriber_torque_supervisor = this->NodeHandler->subscribe(torque_supervisor_topic_name, 10.0, &TorqueAddition_Delay_PD_Wifi::TorqueSupervisorCallback, this);
	this->torque_final_publisher = this->NodeHandler->advertise<baxter_core_msgs::JointCommand>(output_topic_name, 10.0);

	this->control_publisher = this->NodeHandler->advertise<edlut_ros::Analog>(iteration_filter_size_topic, 10.0);
	this->future_buffer_size_publisher = this->NodeHandler->advertise<edlut_ros::Analog>(future_buffer_samples_topic, 10.0);

        this->motor_delay_publisher = this->NodeHandler->advertise<edlut_ros::Analog>(motor_delay_topic, 10.0);

	this->init_time_stamp = 0.0;
	this->first_msg = false;

	this->last_index_copy = 0;

	if (disable_gravity_compensation){
		this->gravity_Publisher = this->NodeHandler->advertise<std_msgs::Empty>(gravity_topic, 1);
	}
}

TorqueAddition_Delay_PD_Wifi::~TorqueAddition_Delay_PD_Wifi() {
	// TODO Auto-generated destructor stub
}



void TorqueAddition_Delay_PD_Wifi::UpdateTorque(ros::Time current_time){
	double end_time = current_time.toSec();

	this->CleanCerebellarBuffer(end_time);

	for (unsigned int i=0; i<this->joint_list.size(); ++i){
		this->output_torque[i] = this->torque_cerebellum[i] + this->torque_supervisor[i];
	}

	// std::ostringstream oss;
  //   std::copy(this->output_torque.begin(), this->output_torque.end(), std::ostream_iterator<double>(oss, ","));
  //   std::ostringstream oss2;
  //   std::copy(this->torque_cerebellum.begin(), this->torque_cerebellum.end(), std::ostream_iterator<double>(oss2, ","));
  //   std::ostringstream oss3;
  //   std::copy(this->torque_supervisor.begin(), this->torque_supervisor.end(), std::ostream_iterator<double>(oss3, ","));
  // ROS_DEBUG("Torque Calculator: Calculated torque value: time %f and value %s. Torque 1 %s, Torque 2 %s", current_time.toSec(), oss.str().c_str(), oss2.str().c_str(), oss3.str().c_str());

	// Create the message and publish it
	baxter_core_msgs::JointCommand newMsg;
	newMsg.mode = 3;
	newMsg.names = this->joint_list;
	newMsg.command = this->output_torque;
	this->torque_final_publisher.publish(newMsg);

	if (this->disable_gravity_compensation){
			std_msgs::Empty msg;
			this->gravity_Publisher.publish(msg);
	}

	// ROS_INFO("ROBOT JOINT COMMAND published. \n Torque[0] cerebellar + supervisor: [%s] + [%s] = [%s]", oss2.str().c_str(), oss3.str().c_str(), oss.str().c_str());
	return;
}


// Compute the cerebellar output torque from the future and past buffers
void TorqueAddition_Delay_PD_Wifi::CleanCerebellarBuffer(double end_time){

		edlut_ros::AnalogCompact top_value;
		edlut_ros::Analog control_msg, future_torque_buffer_msg;

		std::vector<double> mean_joint (this->joint_list.size(), 0.0);
		std::vector<int> samples (this->joint_list.size(), 0);

		int iterator_past_buffer = 0;

		// Clean the buffer
		if (!this->future_cerebellar_torque_buffer.empty()){
			top_value = this->future_cerebellar_torque_buffer.back();
			while (!(this->future_cerebellar_torque_buffer.empty()) && top_value.header.stamp.toSec()<end_time){
				this->InserAndOrderCerebellarBufferPast(top_value, this->past_cerebellar_torque_buffer);
				if (this->past_cerebellar_torque_buffer.size()>this->max_past_buffer_size){
					this->past_cerebellar_torque_buffer.pop_back();
				}
				this->future_cerebellar_torque_buffer.pop_back();
				// ROS_WARN("Torque calculator: Discarded torque from buffer with time %f. Current time: %f", top_value.header.stamp.toSec(), ros::Time::now().toSec());
				if (!this->future_cerebellar_torque_buffer.empty()){
					top_value = this->future_cerebellar_torque_buffer.back();
				}
			}
		}

		// Get the amount of future and past samples that will be used.
		int future_samples = 0;
		int past_samples = 0;
		if (!this->future_cerebellar_torque_buffer.empty()){
			if (this->future_cerebellar_torque_buffer.size() > this->max_past_buffer_size){
				future_samples = this->max_past_buffer_size + 1;
			}
			else{
				future_samples = this->future_cerebellar_torque_buffer.size();
			}
		}
		// If the future buffer stores at least min_future_samples + 1 samples (i.e. future samples + present sample)
		// proceed with the past samples and compute the mean filter. Otherwise the generated torque equals the last
		// commanded torque reduced by a factor.
		if (future_samples >= (this->min_future_samples + 1) ){
			if (!this->past_cerebellar_torque_buffer.empty()){
				if (this->past_cerebellar_torque_buffer.size() >= future_samples - 1){
					past_samples = future_samples - 1;
				}
				else{
					past_samples = this->past_cerebellar_torque_buffer.size();
				}
			}
			// Compute future and past samples
			int last_future_sample = this->future_cerebellar_torque_buffer.size() - 1;
			for (int i=0; i<future_samples; i++){
				for (unsigned int j=0; j<this->joint_list.size(); j++){
					int index = this->FindJointIndex(this->joint_list, this->future_cerebellar_torque_buffer[last_future_sample - i].names[j]);
					if (index!=-1) {
						mean_joint[index] += this->future_cerebellar_torque_buffer[last_future_sample - i].data[index];
						samples[index] += 1;
					}
				}
			}
			for (int i=0; i<past_samples; i++){
				for (unsigned int j=0; j<this->joint_list.size(); j++){
					int index = this->FindJointIndex(this->joint_list, this->past_cerebellar_torque_buffer[i].names[j]);
					if (index!=-1) {
						mean_joint[index] += this->past_cerebellar_torque_buffer[i].data[index];
						samples[index] += 1;
					}
				}
			}

		  //Normalize the mean value
			for (unsigned int j=0; j<samples.size(); j++){
				//With at least 10 samples, the mean filter is large enough to obtain a smooth mean value.
				if (samples[j] > 0){
					mean_joint[j]/=samples[j];
				}
				else{
					mean_joint[j] = 0;
				}

				// if(samples[j] >= 10){
				// 	mean_joint[j]/=samples[j];
				// }
				// //otherwhise, we soften the signal replacing the missing samples using a weighted backup_copy.
				// else{
				// 	mean_joint[j]=mean_joint[j]/10.0 +  this->torque_backup_copy[sample_index][j] * (10.0 - samples[j]) /10.0;
				// }

				this->torque_cerebellum[j] = mean_joint[j];
				this->last_torque_cerebellum[j] = this->torque_cerebellum[j];
			}

			// compute backup copy index based on time
			double time_sample = end_time - this->init_time_stamp;
			int sample_index = round ( time_sample / this->T_step );
			sample_index = sample_index % this->backup_copy_size;
			if (sample_index < 0){
				sample_index = 0;
			}
	    // Store torque command in the backup copy
			this->torque_backup_copy[sample_index] = mean_joint;
		}
		// If not enough future samples: generate the last commanded torque reduced by a given factor.
		else{
			for (unsigned int j=0; j<samples.size(); j++){
				this->torque_cerebellum[j] = this->last_torque_cerebellum[j] * this->torque_reduction_factor;
				this->last_torque_cerebellum[j] = this->torque_cerebellum[j];
			}
		}

		// Publish the amount of samples used in the mean filter
		control_msg.data = samples[0];
		control_msg.header.stamp =  ros::Time::now();
		control_publisher.publish(control_msg);

		// Publish the amount of samples available in the future buffer
		future_torque_buffer_msg.data = this->future_cerebellar_torque_buffer.size();
		future_torque_buffer_msg.header.stamp = ros::Time::now();
		future_buffer_size_publisher.publish(future_torque_buffer_msg);
}
