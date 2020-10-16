/***************************************************************************
 *                           ROSSpikeDecoderCompactFutureTorque.cpp        *
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

// This node performs the spike-to-analogue conversion.

#include <vector>
#include <iostream>
#include <iterator>
#include <cmath>

#include "edlut_ros/ROSSpikeDecoderCompactFutureTorque.h"
#include "edlut_ros/Spike.h"
#include "edlut_ros/Spike_group.h"

void ROSSpikeDecoderCompactFutureTorque::SpikeCallback(const edlut_ros::Spike_group::ConstPtr& msg){
	// Check if the spike should have been processed previously
	for (int j=0; j<msg->time.size(); j++){
			bool found = false;
			for (unsigned int i=0; i<this->min_neuron_index_pos.size() && !found; ++i){
				found = ((msg->neuron_index[j]<=this->max_neuron_index_pos[i] && msg->neuron_index[j]>=this->min_neuron_index_pos[i]) ||
								(msg->neuron_index[j]<=this->max_neuron_index_neg[i] && msg->neuron_index[j]>=this->min_neuron_index_neg[i]));
			}

			if (!found){
				//ROS_DEBUG("Spike Decoder: Spike received from a neuron out of the range. Discarded. Spike time: %f, Neuron index: %d, Current time: %f.", msg->time, msg->neuron_index, ros::Time::now().toSec());
			} else {
				//ROS_DEBUG("Spike Decoder: Received spike: time %f and neuron %d.", msg->time, msg->neuron_index);
				edlut_ros::Spike spike;
				spike.time = msg->time[j];
				spike.neuron_index = msg->neuron_index[j];
				this->activity_queue.push(spike);

				if (spike.time > this->last_spike_time){
					this->last_spike_time = spike.time;
				}
			}
	}
}


ROSSpikeDecoderCompactFutureTorque::ROSSpikeDecoderCompactFutureTorque(std::string input_topic_name,
		std::vector<int> min_neuron_index_pos,
		std::vector<int> max_neuron_index_pos,
		std::vector<int> min_neuron_index_neg,
		std::vector<int> max_neuron_index_neg,
		double tau_time_constant,
		std::vector<double> spike_increment_pos,
		std::vector<double> spike_increment_neg,
		double time_step,
		std::vector<std::string> joint_list):
				NodeHandler(),
				CallbackQueue(),
				activity_queue(),
				tau_time_constant(tau_time_constant),
				spike_increment_pos(spike_increment_pos),
				spike_increment_neg(spike_increment_neg),
				output_var(0.0),
				last_time(0.0),
				time_step(time_step),
				joint_list(joint_list),
				last_spike_time(0.0)
{

	this->min_neuron_index_pos = std::vector<unsigned int> (min_neuron_index_pos.size());
	for (unsigned int i=0; i<min_neuron_index_pos.size(); ++i){
		this->min_neuron_index_pos[i] = (unsigned int) min_neuron_index_pos[i];
	}
	this->max_neuron_index_pos = std::vector<unsigned int> (max_neuron_index_pos.size());
	for (unsigned int i=0; i<max_neuron_index_pos.size(); ++i){
		this->max_neuron_index_pos[i] = (unsigned int) max_neuron_index_pos[i];
	}
	this->min_neuron_index_neg = std::vector<unsigned int> (min_neuron_index_neg.size());
	for (unsigned int i=0; i<min_neuron_index_neg.size(); ++i){
		this->min_neuron_index_neg[i] = (unsigned int) min_neuron_index_neg[i];
	}
	this->max_neuron_index_neg = std::vector<unsigned int> (max_neuron_index_neg.size());
	for (unsigned int i=0; i<max_neuron_index_neg.size(); ++i){
		this->max_neuron_index_neg[i] = (unsigned int) max_neuron_index_neg[i];
	}


	this->output_var = std::vector<double>(min_neuron_index_pos.size(),0.0);
	this->NodeHandler.setCallbackQueue(&this->CallbackQueue);

	ros::SubscribeOptions subscriberOptions = ros::SubscribeOptions::create<edlut_ros::Spike_group>(
	 		input_topic_name, 1000, boost::bind(&ROSSpikeDecoderCompactFutureTorque::SpikeCallback, this, _1), ros::VoidPtr(), &this->CallbackQueue);

	subscriberOptions.transport_hints =
 	 ros::TransportHints().unreliable().reliable().tcpNoDelay(true);

	this->subscriber = this->NodeHandler.subscribe(subscriberOptions);
}

ROSSpikeDecoderCompactFutureTorque::~ROSSpikeDecoderCompactFutureTorque() {
	// TODO Auto-generated destructor stub
}

std::vector<edlut_ros::AnalogCompact> ROSSpikeDecoderCompactFutureTorque::UpdateDecoder(double end_time){

	std::vector<edlut_ros::AnalogCompact> torque_buffer;

	double checkpoint = end_time;

	// Process all the spikes in the queue
	this->CallbackQueue.callAvailable();

	edlut_ros::Spike top_spike;

	// Update the output variable to half of the time bin
	// double elapsed_time = end_time - this->last_time;
	double elapsed_time = this->last_time + this->time_step;
	for (unsigned int i=0; i<this->output_var.size(); ++i){
		if(this->tau_time_constant > 0){
			this->output_var[i] *= exp(-elapsed_time/this->tau_time_constant);
		}else{
			this->output_var[i] = 0;
		}
	}

	if (!this->activity_queue.empty()){
		// ROS_INFO("1111111111111111111111111 FUTURE TORQUE PUBLISHER 1111111111111111111111111");
		if (this->last_time + this->time_step > this->last_spike_time){
			checkpoint = this->last_time;
		}else{
			checkpoint = this->last_time + this->time_step;
		}
		// if (this->last_time > checkpoint){
		// 	checkpoint = this->last_time + this->time_step;
		// }else{
		// 	checkpoint += this->time_step;
		// }
		top_spike = this->activity_queue.top();
		// ROS_INFO("SPIKE TIME %f", top_spike.time);
		// ROS_INFO("CheckPoint %f", checkpoint);
		while (!(this->activity_queue.empty()) &&	top_spike.time<=checkpoint){
			//ROS_DEBUG("Spike Decoder: Processing spike with time %f and neuron %d. End time: %f. Current time: %f", top_spike.time, top_spike.neuron_index, end_time, ros::Time::now().toSec());
			for (unsigned int i=0; i<this->min_neuron_index_pos.size(); ++i){
				if (top_spike.neuron_index<=this->max_neuron_index_pos[i] && top_spike.neuron_index>=this->min_neuron_index_pos[i]){
					this->output_var[i] += this->spike_increment_pos[i];
					//ROS_DEBUG("Spike Decoder: Increasing neuron %i in index %i with value %f. Total: %f", top_spike.neuron_index, i, this->spike_increment[i], this->output_var[i]);
				}
				if (top_spike.neuron_index<=this->max_neuron_index_neg[i] && top_spike.neuron_index>=this->min_neuron_index_neg[i]){
					this->output_var[i] -= this->spike_increment_neg[i];
					//ROS_DEBUG("Spike Decoder: Increasing neuron %i in index %i with value %f. Total: %f", top_spike.neuron_index, i, this->spike_increment[i], this->output_var[i]);
				}
			}
			this->activity_queue.pop();
			if (!this->activity_queue.empty()){
				top_spike = this->activity_queue.top();
			}

			if (top_spike.time > checkpoint){
				edlut_ros::AnalogCompact newTorque;
				newTorque.data = this->output_var;
				newTorque.header.stamp = ros::Time(checkpoint);
				newTorque.names = this->joint_list;
				torque_buffer.push_back(newTorque);
				// ROS_INFO("222222222222222222 FUTURE TORQUE PUBLISHER 22222222222222222222");

				// Update the output variable to half of the time bin
				elapsed_time = checkpoint + this->time_step;
				for (unsigned int i=0; i<this->output_var.size(); ++i){
					if(this->tau_time_constant > 0){
						this->output_var[i] *= exp(-elapsed_time/this->tau_time_constant);
					}else{
						this->output_var[i] = 0;
					}
				}
				if(checkpoint+this->time_step < this->last_spike_time){
					checkpoint += this->time_step;
				}
			}
		}
		// while (!(this->activity_queue.empty()) &&	top_spike.time<=end_time){
		// 	//ROS_DEBUG("Spike Decoder: Processing spike with time %f and neuron %d. End time: %f. Current time: %f", top_spike.time, top_spike.neuron_index, end_time, ros::Time::now().toSec());
		// 	for (unsigned int i=0; i<this->min_neuron_index_pos.size(); ++i){
		// 		if (top_spike.neuron_index<=this->max_neuron_index_pos[i] && top_spike.neuron_index>=this->min_neuron_index_pos[i]){
		// 			this->output_var[i] += this->spike_increment_pos[i];
		// 			//ROS_DEBUG("Spike Decoder: Increasing neuron %i in index %i with value %f. Total: %f", top_spike.neuron_index, i, this->spike_increment[i], this->output_var[i]);
		// 		}
		// 		if (top_spike.neuron_index<=this->max_neuron_index_neg[i] && top_spike.neuron_index>=this->min_neuron_index_neg[i]){
		// 			this->output_var[i] -= this->spike_increment_neg[i];
		// 			//ROS_DEBUG("Spike Decoder: Increasing neuron %i in index %i with value %f. Total: %f", top_spike.neuron_index, i, this->spike_increment[i], this->output_var[i]);
		// 		}
		// 	}
		// 	this->activity_queue.pop();
		// 	if (!this->activity_queue.empty()){
		// 		top_spike = this->activity_queue.top();
		// 	}
		// }
	}
	// this->last_time = end_time;
this->last_time = checkpoint;

	// return this->output_var;
return torque_buffer;
}
