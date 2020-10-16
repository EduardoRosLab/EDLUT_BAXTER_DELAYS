/***************************************************************************
 *                           TorqueAddition.cpp							               *
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

#include "edlut_ros/TorqueAddition.h"
#include "edlut_ros/AnalogCompact.h"
#include "edlut_ros/Analog.h"
#include <baxter_core_msgs/JointCommand.h>
#include <std_msgs/Empty.h>



// Callback for cerebellar torque commands
void TorqueAddition::TorqueCerebellumCallback(const edlut_ros::AnalogCompact::ConstPtr& msg){
	if (!this->first_msg){
		this->first_msg = true;
		this->init_time_stamp = msg->header.stamp.toSec();
	}
	edlut_ros::AnalogCompact newMessage;
	newMessage.header.stamp = msg->header.stamp;
	newMessage.data = msg->data;
	newMessage.names = msg->names;

	// // Store torque command in the backup copy
	// std::vector<double> torque_copy(this->joint_list.size(), 0.0);
	// for (unsigned int i=0; i<msg->names.size(); ++i){
	// 	int index = this->FindJointIndex(this->joint_list, msg->names[i]);
	// 	if (index!=-1) {
	// 		torque_copy[index] = msg->data[i];
	// 	}
	// }
	// double time_sample = msg->header.stamp.toSec() - this->init_time_stamp;
	// int sample_index = round ( time_sample / this->T_step );
	// sample_index = sample_index % this->backup_copy_size;
	// this->torque_backup_copy[sample_index] = torque_copy;
	//
	//
	// // Interpolate missing samples in the backup copy
	// int dif_samples = sample_index - this->last_index_copy;
	//
	// // ROS_INFO("DIF = %i, Sample_index = %i, Last_index = %i", dif_samples, sample_index, this->last_index_copy);
	//
	// if (dif_samples > 1){
	// 	for (unsigned int j=0; j<this->joint_list.size(); j++){
	// 		this->dif_value[j] = (this->torque_backup_copy[sample_index][j] - this->torque_backup_copy[this->last_index_copy][j])/dif_samples;
	// 	}
	// 	for (unsigned int i=1; i<=dif_samples; i++){
	// 		for (unsigned int j=0; j<this->joint_list.size(); j++){
	// 			this->torque_backup_copy[this->last_index_copy + i][j] = this->torque_backup_copy[this->last_index_copy][j] + this->dif_value[j]*i;
	// 		}
	// 	}
	// }
	// else if (dif_samples < 0){
	// 	int back_distance = this->torque_backup_copy.size() - 1 - this->last_index_copy;
	// 	int front_distance = sample_index;
	// 	dif_samples =  back_distance + front_distance;
	// 	for (unsigned int j=0; j<this->joint_list.size(); j++){
	// 		this->dif_value[j] = (this->torque_backup_copy[sample_index][j] - this->torque_backup_copy[this->last_index_copy][j])/dif_samples;
	// 	}
	// 	for (unsigned int i=1; i<=back_distance; i++){
	// 		for (unsigned int j=0; j<this->joint_list.size(); j++){
	// 			this->torque_backup_copy[this->last_index_copy + i][j] = this->torque_backup_copy[this->last_index_copy][j] + this->dif_value[j]*i;
	// 		}
	// 	}
	// 	for (unsigned int i=1; i<=front_distance; i++){
	// 		for (unsigned int j=0; j<this->joint_list.size(); j++){
	// 			this->torque_backup_copy[sample_index - i][j] = this->torque_backup_copy[sample_index][j] - this->dif_value[j]*i;
	// 		}
	// 	}
	// }
	// for (unsigned int j=0; j<this->joint_list.size(); j++){
	// 	this->dif_value[j] = 0.0;
	// }
	// this->last_index_copy = sample_index;

	// Check if the received torque command should have been already processed.
	// If it is a past sample then it is stored in the past buffer. Otherwise it
	// is stored in the future buffer.
	if (msg->header.stamp.toSec()<ros::Time::now().toSec()){
		// ROS_INFO("Received cerebellar torque from a past time. Moved to past buffer. Time: %f, Current time: %f.", msg->header.stamp.toSec(), ros::Time::now().toSec());
		this->past_cerebellar_torque_buffer.insert(this->past_cerebellar_torque_buffer.begin(), newMessage);
		this->OrderCerebellarBuffer(this->past_cerebellar_torque_buffer);
		if (this->past_cerebellar_torque_buffer.size()>this->max_past_buffer_size){
			this->past_cerebellar_torque_buffer.pop_back();
		}
		ROS_INFO("PAST TORQUE %f now = %f", msg->header.stamp.toSec(), ros::Time::now().toSec());
	} else {
		// std::ostringstream oss;
    // std::copy(msg->data.begin(), msg->data.end(), std::ostream_iterator<double>(oss, ","));
		// ROS_INFO("Received cerebellar torque: time %f and value %s.", msg->header.stamp.toSec(), oss.str().c_str());
		this->future_cerebellar_torque_buffer.push_back(newMessage);
		this->OrderCerebellarBuffer(this->future_cerebellar_torque_buffer);
		ROS_INFO("FUTURE TORQUE %f now = %f", msg->header.stamp.toSec(), ros::Time::now().toSec());
	}
}

// Order the torque buffer. The last element is the oldest motor command.
void TorqueAddition::OrderCerebellarBuffer(std::vector<edlut_ros::AnalogCompact>  & buffer){
	edlut_ros::AnalogCompact aux;
	if (buffer.size()>1){
		for (unsigned int i=0; i<buffer.size()-1; i++){
			for (unsigned int j=i+1; j<buffer.size(); j++){
				if (buffer[i].header.stamp.toSec() < buffer[j].header.stamp.toSec()){
					aux = buffer[i];
					buffer[i] = buffer[j];
					buffer[j] = aux;
				}
			}
		}
	}
}

// Callback for torque commands from the PD Supervisor
void TorqueAddition::TorqueSupervisorCallback(const edlut_ros::AnalogCompact::ConstPtr& msg){
	if (!this->first_msg){
		this->first_msg = true;
		this->init_time_stamp = msg->header.stamp.toSec();
	}
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

int TorqueAddition::FindJointIndex(std::vector<std::string> strvector, std::string name){
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

TorqueAddition::TorqueAddition(
	std::string torque_cerebellum_topic_name,
	std::string torque_supervisor_topic_name,
	std::string output_topic_name,
	std::vector<std::string> joint_list,
	int max_past_buffer_size,
	int min_filter_size,
	double T_iteration,
	int samples_iteration,
	int N_iterations,
	bool disable_gravity_compensation,
	std::string gravity_topic,
	ros::NodeHandle * NodeHandler,
	ros::CallbackQueue * CallbackQueue):
				joint_list(joint_list),
				max_past_buffer_size(max_past_buffer_size),
				min_filter_size(min_filter_size),
				T_iteration(T_iteration),
				samples_iteration(samples_iteration),
				N_iterations(N_iterations),
				last_time_torque_supervisor(0.0),
				disable_gravity_compensation(disable_gravity_compensation),
				NodeHandler(NodeHandler)
{
	this->torque_cerebellum = std::vector<double> (joint_list.size(), 0.0);
	this->torque_supervisor = std::vector<double> (joint_list.size(), 0.0);
	this->output_torque  = std::vector<double> (joint_list.size(), 0.0);
	this->mean_torque = std::vector<double> (joint_list.size(), 0.0);
	this->dif_value = std::vector<double>(joint_list.size(), 0.0);

	this->backup_copy_size = this->N_iterations * this->samples_iteration;
	this->T_step = this->T_iteration / this->samples_iteration;

	this->torque_backup_copy = std::vector<std::vector <double> > (this->backup_copy_size, std::vector<double> (joint_list.size(), 0.0));

	this->subscriber_torque_cerebellum = this->NodeHandler->subscribe(torque_cerebellum_topic_name, 10.0, &TorqueAddition::TorqueCerebellumCallback, this);
	this->subscriber_torque_supervisor = this->NodeHandler->subscribe(torque_supervisor_topic_name, 10.0, &TorqueAddition::TorqueSupervisorCallback, this);
	this->torque_final_publisher = this->NodeHandler->advertise<baxter_core_msgs::JointCommand>(output_topic_name, 1.0);

	this->control_publisher = this->NodeHandler->advertise<edlut_ros::Analog>("torque_final/control_topic", 1.0);


	this->init_time_stamp = 0.0;
	this->first_msg = false;

	this->last_index_copy = 0;

	if (disable_gravity_compensation){
		this->gravity_Publisher = this->NodeHandler->advertise<std_msgs::Empty>(gravity_topic, 1);
	}
}

TorqueAddition::~TorqueAddition() {
	// TODO Auto-generated destructor stub
}

void TorqueAddition::UpdateTorque(ros::Time current_time){
	double end_time = current_time.toSec();

	this->CleanCerebellarBuffer(this->future_cerebellar_torque_buffer, this->torque_cerebellum, end_time);

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
void TorqueAddition::CleanCerebellarBuffer(std::vector<edlut_ros::AnalogCompact>  & buffer,
	std::vector<double> & updateVar,
	double end_time){
		edlut_ros::AnalogCompact top_value;
		edlut_ros::Analog control_msg;


		std::vector<double> mean_joint (this->joint_list.size(), 0.0);
		std::vector<int> samples (this->joint_list.size(), 0);

		int iterator_past_buffer = 0;

		// Clean the buffer
		if (!buffer.empty()){
			top_value = buffer.back();
			while (!(buffer.empty()) && top_value.header.stamp.toSec()<end_time){
				this->past_cerebellar_torque_buffer.insert(this->past_cerebellar_torque_buffer.begin(), top_value);
				if (this->past_cerebellar_torque_buffer.size()>this->max_past_buffer_size){
					this->past_cerebellar_torque_buffer.pop_back();
				}
				buffer.pop_back();
				// ROS_WARN("Torque calculator: Discarded torque from buffer with time %f. Current time: %f", top_value.header.stamp.toSec(), ros::Time::now().toSec());
				if (!buffer.empty()){
					top_value = buffer.back();
				}
			}
		}

		// Compute output torque as mean of the N_future samples and N_past samples
		// If the future buffer is empty the output command is obtained from the
		// backup torque copy
		if (!buffer.empty()){
			for (unsigned int i=0; i<this->future_cerebellar_torque_buffer.size(); i++){
				for (unsigned int j=0; j<this->joint_list.size(); j++){
					int index = this->FindJointIndex(this->joint_list, this->future_cerebellar_torque_buffer[i].names[j]);
					if (index!=-1) {
						mean_joint[index] += this->future_cerebellar_torque_buffer[i].data[index];
						samples[index] += 1;
					}
				}
			}
			int past_samples = 0;
			if (this->min_filter_size > this->future_cerebellar_torque_buffer.size()){
				past_samples = this->min_filter_size - this->future_cerebellar_torque_buffer.size();
			}
			while (iterator_past_buffer<this->past_cerebellar_torque_buffer.size() && iterator_past_buffer<past_samples){
				for (unsigned int j=0; j<this->joint_list.size(); j++){
					int index = this->FindJointIndex(this->joint_list, this->past_cerebellar_torque_buffer[iterator_past_buffer].names[j]);
					if (index!=-1) {
						mean_joint[index] += this->past_cerebellar_torque_buffer[iterator_past_buffer].data[index];
						samples[index] += 1;
					}
				}
				iterator_past_buffer += 1;
			}
			for (unsigned int i=0; i<mean_joint.size(); i++){
				mean_joint[i] = mean_joint[i] / samples[i];
				updateVar[i] = mean_joint[i];
				// updateVar[i] = 10;
			}

			// Store torque command in the backup copy
			double time_sample = end_time - this->init_time_stamp;
			int sample_index = round ( time_sample / this->T_step );
			sample_index = sample_index % this->backup_copy_size;
			if (sample_index < 0){
				sample_index = 0;
			}
			this->torque_backup_copy[sample_index] = mean_joint;

			// Interpolate missing samples in the backup copy
			int dif_samples = sample_index - this->last_index_copy;

			ROS_INFO("DIF = %i, Sample_index = %i, Last_index = %i", dif_samples, sample_index, this->last_index_copy);

			if (dif_samples > 1){
				for (unsigned int j=0; j<this->joint_list.size(); j++){
					this->dif_value[j] = (this->torque_backup_copy[sample_index][j] - this->torque_backup_copy[this->last_index_copy][j])/dif_samples;
				}
				for (unsigned int i=1; i<=dif_samples; i++){
					for (unsigned int j=0; j<this->joint_list.size(); j++){
						this->torque_backup_copy[this->last_index_copy + i][j] = this->torque_backup_copy[this->last_index_copy][j] + this->dif_value[j]*i;
					}
				}
			}
			else if (dif_samples < 0){
				int back_distance = this->torque_backup_copy.size() - 1 - this->last_index_copy;
				int front_distance = sample_index;
				dif_samples =  back_distance + front_distance;
				for (unsigned int j=0; j<this->joint_list.size(); j++){
					this->dif_value[j] = (this->torque_backup_copy[sample_index][j] - this->torque_backup_copy[this->last_index_copy][j])/dif_samples;
				}
				for (unsigned int i=1; i<=back_distance; i++){
					for (unsigned int j=0; j<this->joint_list.size(); j++){
						this->torque_backup_copy[this->last_index_copy + i][j] = this->torque_backup_copy[this->last_index_copy][j] + this->dif_value[j]*i;
					}
				}
				for (unsigned int i=1; i<=front_distance; i++){
					for (unsigned int j=0; j<this->joint_list.size(); j++){
						this->torque_backup_copy[sample_index - i][j] = this->torque_backup_copy[sample_index][j] - this->dif_value[j]*i;
					}
				}
			}
			for (unsigned int j=0; j<this->joint_list.size(); j++){
				this->dif_value[j] = 0.0;
			}
			this->last_index_copy = sample_index;
			// Control topic to know if the future buffer is being used
			control_msg.data = 5.0;
			control_msg.header.stamp = ros::Time::now();
			control_publisher.publish(control_msg);
		}
		else{
			int sample_index;
			sample_index = round ((end_time-this->init_time_stamp) / this->T_step);
			sample_index = sample_index % this->samples_iteration;
			for (unsigned int i=0; i<this->N_iterations; i++){
				for (unsigned int j=0; j<this->joint_list.size(); j++){
					this->mean_torque[j] += this->torque_backup_copy[sample_index + i*this->samples_iteration][j];
				}
			}
			for (unsigned int j=0; j<this->mean_torque.size(); j++){
				this->mean_torque[j] = this->mean_torque[j] / this->N_iterations;
				updateVar[j] = this->mean_torque[j];
				// updateVar[j] = 20;
				this->mean_torque[j] = 0.0;
			}
			// Control topic to know if the torque backup_copy is being used
			control_msg.data = -5.0;
			control_msg.header.stamp = ros::Time::now();
			control_publisher.publish(control_msg);
		}
		// Publish control topic
}
