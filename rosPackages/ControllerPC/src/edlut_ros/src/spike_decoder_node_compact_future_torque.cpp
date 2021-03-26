/***************************************************************************
 *                           spike_decoder_node_compact_future_torque.cpp  *
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

// This is the spike decoder node. It translates spiking activity to analogue
// signals.

#include <edlut_ros/AnalogCompact.h>
#include <edlut_ros/AnalogCompactDelay.h>
#include <edlut_ros/Time.h>
#include <edlut_ros/ROSSpikeDecoderCompactFutureTorque_compactDelay.h>
#include <ros/ros.h>
#include "log4cxx/logger.h"


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
class Delay {
public:
	ros::Subscriber delay_subscriber;
private:
	double delay;

public:
	Delay (){

		this->delay = 0.0;

		return;
	};

	void DelayCallback(const edlut_ros::Time::ConstPtr& msg){
		this->delay = msg->data;
		return;
	}

	double GetDelay(){
		return this->delay;
	}

};

/*
 * Create a class
 */
class TorqueQueueManager {

private:

	std::vector<edlut_ros::AnalogCompactDelay> torque_queue;
	std::vector<double> delay_queue;


public:
	TorqueQueueManager (){
		return;
	};

	// Order the Robot State Buffer according to time_stamps and delays.
	// The last element in the buffer will be the oldest (i.e. smallest message_time_stamp + associated_delay)
	void InsertandOrderBuffers(edlut_ros::AnalogCompactDelay newMessage, double newDelay){
		this->torque_queue.insert(this->torque_queue.begin(), newMessage);
		this->delay_queue.insert(this->delay_queue.begin(), newDelay);

		edlut_ros::AnalogCompactDelay aux_torqueMsg;
		double aux_delay;

		if (this->torque_queue.size()>1){
			int index = 1;
			while (index < (this->torque_queue.size()) && (this->torque_queue[index-1].delay + this->delay_queue[index-1]) < (this->torque_queue[index].delay + this->delay_queue[index])){
					aux_torqueMsg = this->torque_queue[index];
					aux_delay = this->delay_queue[index];

					this->torque_queue[index] = this->torque_queue[index-1];
					this->delay_queue[index] = this->delay_queue[index-1];
					this->torque_queue[index-1] = aux_torqueMsg;
					this->delay_queue[index-1] = aux_delay;

					index++;
			}
		}
	}

	edlut_ros::AnalogCompactDelay GetTopValue_Torque(){
		return this->torque_queue.back();
	}

	double GetTopValue_Delay(){
		return this->delay_queue.back();
	}

	void PopBack_Torque(){
		this->torque_queue.pop_back();
	}

	void PopBack_Delay(){
		this->delay_queue.pop_back();
	}

	bool TorqueQueue_Empty(){
		return this->torque_queue.empty();
	}

};



int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "spike_decoder", ros::init_options::NoSigintHandler);
	log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
	my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
	ros::NodeHandle nh;

	signal(SIGINT, rosShutdownHandler);

	// Declare variables that can be modified by launch file or command line.
	std::string input_topic, output_topic, artificial_delay_topic;
	std::vector<int> min_neuron_index_pos, max_neuron_index_pos, min_neuron_index_neg, max_neuron_index_neg;
	std::vector<double> delta_spike_pos, delta_spike_neg;
	std::vector<std::string> joint_list;
	double sampling_frequency, tau_time_constant, time_step;
	bool use_sim_time, wireless_simulation;
	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;

	std::vector<edlut_ros::AnalogCompactDelay> torque_buffer;

	std::vector<double> delay_queue;


	ros::Time current_time;

	stop_node = false;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.getParam("input_topic", input_topic);
	private_node_handle_.getParam("output_topic", output_topic);
	private_node_handle_.getParam("joint_list", joint_list);
	private_node_handle_.getParam("min_neuron_index_list_pos", min_neuron_index_pos);
	private_node_handle_.getParam("max_neuron_index_list_pos", max_neuron_index_pos);
	private_node_handle_.getParam("min_neuron_index_list_neg", min_neuron_index_neg);
	private_node_handle_.getParam("max_neuron_index_list_neg", max_neuron_index_neg);
	private_node_handle_.getParam("sampling_frequency", sampling_frequency);
	private_node_handle_.getParam("tau_time_constant", tau_time_constant);
	private_node_handle_.getParam("delta_spike_list_pos", delta_spike_pos);
	private_node_handle_.getParam("delta_spike_list_neg", delta_spike_neg);
	private_node_handle_.getParam("delay_topic", artificial_delay_topic);


	time_step = 1/sampling_frequency;

	nh.getParam("use_sim_time", use_sim_time);
	nh.getParam("wireless_simulation", wireless_simulation);


	// Create the subscriber
  ros::CallbackQueue CallbackQueue;
  nh.setCallbackQueue(&CallbackQueue);

	Delay objDelay = Delay();

	TorqueQueueManager objTorqueManager = TorqueQueueManager();

	if (wireless_simulation){
		objDelay.delay_subscriber = nh.subscribe(artificial_delay_topic, 1000, &Delay::DelayCallback, &objDelay);
	}

	// Create the publisher
	ros::Publisher output_publisher = nh.advertise<edlut_ros::AnalogCompactDelay>(output_topic, 1.0);
	ROSSpikeDecoderCompactFutureTorque_compactDelay objSpikeDecoder(
			input_topic,
			min_neuron_index_pos,
			max_neuron_index_pos,
			min_neuron_index_neg,
			max_neuron_index_neg,
			tau_time_constant,
			delta_spike_pos,
			delta_spike_neg,
			time_step,
			joint_list
	);



	ROS_INFO("Spike Decoder node initialized: reading from topic %s and writing to topic %s", input_topic.c_str(), output_topic.c_str());

	ros::Rate rate(sampling_frequency);

	if (use_sim_time){
		clock_subscriber = nh.subscribe("/clock_sync", 1000, &ExternalClock::ClockCallback, &ext_clock);
	}

	while (!stop_node){
		CallbackQueue.callAvailable(ros::WallDuration(0.003));
		if (use_sim_time){
			current_time = ext_clock.GetLastConfirmedTime();
		}
		else{
			current_time = ros::Time::now();
		}

		// std::vector<double> analog_value = objSpikeDecoder.UpdateDecoder(current_time.toSec());

		// Torque buffer. Future torque commands are published when ready.
		torque_buffer = objSpikeDecoder.UpdateDecoder(current_time.toSec());

		edlut_ros::AnalogCompactDelay msg;

		// ROS_INFO("xxxxxxxxxxxx FUTURE TORQUE PUBLISHER xxxxxxxxxxxx %i", torque_buffer.size());
		for (int x=0; x<torque_buffer.size(); x++){
			msg = torque_buffer[x];

//start new buffer delays manager
			// This is used in the receiver node (Torque_final_node_delay) to compute the motor_delay
			// In this case msg.delay equals the generation time of the current message. It is compared
			// to the reception time at the receiver node to get the motor_delay.
			// msg.delay = current_time.toSec();
			msg.delay = ros::Time::now().toSec();
			// ROS_INFO("MSG TIME STAMP SPIKE DECODER = %f ", msg.delay);

			// If it is a real wireless scenario: publish the torque commands right away
			if (!wireless_simulation){
				output_publisher.publish(msg);
			}
			// If it is a simulated wireless scenario: append the torque command and the artificial delay to a buffer
			// and publish the torque command when the delay is over
			else{
				objTorqueManager.InsertandOrderBuffers(msg, objDelay.GetDelay());
			}
		}
		if (wireless_simulation && !objTorqueManager.TorqueQueue_Empty()){
			edlut_ros::AnalogCompactDelay top_value;
			double top_delay;
			top_value = objTorqueManager.GetTopValue_Torque();
			top_delay = objTorqueManager.GetTopValue_Delay();

			// In this case, the msg.delay field is the generation time of each message. The msg.header.stamp here corresponds to the time at which the
			// torque command should be applied, we are generating future torque commands. top_delay is the corresponding artificial delay associated
			// to each message. When the delay is over (i.e. current_time is greater than generation_time + associated_delay) the message can be published.
			while (!objTorqueManager.TorqueQueue_Empty() && (top_value.delay + top_delay <= current_time.toSec()) ){
				output_publisher.publish(top_value);
				// ROS_INFO("MSG PUBLISHED SPIKE DECODER time = %f , msg_time = %f, msg_delay = %f", ros::Time::now().toSec(), top_value.delay, top_delay);

				objTorqueManager.PopBack_Torque();
				objTorqueManager.PopBack_Delay();

				if (!objTorqueManager.TorqueQueue_Empty()){
					top_value = objTorqueManager.GetTopValue_Torque();
					top_delay = objTorqueManager.GetTopValue_Delay();
				}
			}
		}

//end new buffer delays manager

		// 	if (wireless_simulation){
		// 		msg.delay = objDelay.GetDelay();
		// 	}
		// 	else{
		// 		// This is used in the receiver node (Torque_final_node_delay) to compute the motor_delay
		// 		// In this case msg.delay equals the generation time of the current message. It is compared
		// 		// to the reception time at the receiver node to get the motor_delay.
		// 		msg.delay = ros::Time::now().toSec();
		// 	}
		// 	output_publisher.publish(msg);
		// 	// ROS_INFO("Published TORQUE with time %f", msg.header.stamp.toSec());
		// }

		// std::ostringstream oss;
    // 	std::copy(analog_value.begin(), analog_value.end(), std::ostream_iterator<double>(oss, ","));
		// ROS_DEBUG("Updating spike decoder activity at time %f. Value returned: %s", current_time.toSec(), oss.str().c_str());
		//
		// edlut_ros::AnalogCompact msg;
		// msg.data = analog_value;
		// msg.names = joint_list;
		// msg.header.stamp = current_time;
		// output_publisher.publish(msg);
		////
		// rate.sleep();
	}

	ROS_INFO("Ending Spike Decoder node");

	ros::shutdown();
	return 0;
} // end main()
