/***************************************************************************
 *                           TorqueAddition_Delay_PD_Wifi.h                *
 *                           -------------------                           *
 * copyright            : (C) 2020 by Ignacio Abadía                       *
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

#ifndef TorqueAddition_Delay_PD_Wifi_H_
#define TorqueAddition_Delay_PD_Wifi_H_

#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <edlut_ros/Analog.h>
#include <edlut_ros/AnalogCompact.h>
#include <edlut_ros/AnalogCompactDelay.h>


// class torque_comparison {
// public:
// 	bool operator() (const edlut_ros::AnalogCompact lsp, const edlut_ros::AnalogCompact rsp) const{
// 		if (lsp.header.stamp>rsp.header.stamp) {
// 			return true;
// 		} else {
// 			return false;
// 		}
// 	}
// };

/*
 * This class defines a spike-to-analog decoder.
 */
class TorqueAddition_Delay_PD_Wifi {
private:

	// ROS Node Handler
	ros::NodeHandle * NodeHandler;

	// Cerebellar torque subscriber
	ros::Subscriber subscriber_torque_cerebellum;

	// PD supervisor torque subscriber
	ros::Subscriber subscriber_torque_supervisor;

	// Output publisher
	ros::Publisher torque_final_publisher, gravity_Publisher, control_publisher, future_buffer_size_publisher, motor_delay_publisher;

	// Disable gravity compensation or not
	bool disable_gravity_compensation;

	// ROS Callback queue
	ros::CallbackQueue * CallbackQueue;

	// Queue of desired position analog signals not processed yet
	// std::priority_queue<edlut_ros::AnalogCompact, std::vector<edlut_ros::AnalogCompact>, torque_comparison > activity_queue_torque_cerebellum;

	// Buffer to store the future cerebellar torque commands
	std::vector<edlut_ros::AnalogCompact> future_cerebellar_torque_buffer;

	// Buffer to store the past cerebellar torque commands
	std::vector<edlut_ros::AnalogCompact> past_cerebellar_torque_buffer;

	// Torque backup copy to compute the mean torque of the last N iterations
	std::vector<std::vector<double> > torque_backup_copy;

	// Size of the backup copy vector
	int backup_copy_size;

	// Iteration period
	double T_iteration;

	// Number of samples per iteration
	int samples_iteration;

	// Time step between samples of the iteration
	double T_step;

	// Time stamp of the first message. Used as time reference
	double init_time_stamp;

	// True when the first message is received
	bool first_msg;

	// Number of iterations to store in the backup copy
	int N_iterations;

	// Total samples of backup copy
	int total_samples;

	// Number of past torque commands samples to store in buffer
	int max_past_buffer_size;

	// Minimum number of future samples to implement the mean filter
	int min_future_samples;

	// Index of last sample stored in the buffer
	int last_index_copy;

	// Message to publish the delay in the reception of the torque commands
	edlut_ros::Analog motor_delayMsg;

	// Vector to store torque differences
	std::vector<double> dif_value;

	// Working time step (inverse of working frequency)
	double time_step;

	// Reduction factor used when there are not enough future samples
	double torque_reduction_factor;

	// Queue of actual position analog signals not processed yet
	// std::priority_queue<edlut_ros::AnalogCompact, std::vector<edlut_ros::AnalogCompact>, torque_comparison > activity_queue_torque_supervisor;

	// Queue to store the torque supplied by the cerebellum in the last iteration
	// queue<std::vector<double>> torque_cerebellum_copy;
	// int max_size_torque_cerebellum_copy;

	// Joint list
	std::vector<std::string> joint_list;

	// Current values
	std::vector<double> torque_cerebellum, last_torque_cerebellum, torque_supervisor, mean_torque;

	// Value of the output variables
	std::vector<double> output_torque;

	// Last time when the decoder has been updated
	double last_time_torque_supervisor;

	// Real or simulated wireless scenario
	bool wireless_simulation;

	// Callback function for reading cerebellar torque
	void TorqueCerebellumCallback(const edlut_ros::AnalogCompactDelay::ConstPtr& msg);


	// Order cerebellar future torque buffer
	void InserAndOrderCerebellarBufferFuture(edlut_ros::AnalogCompact & newMessage, std::vector<edlut_ros::AnalogCompact>  & buffer);

	// Order cerebellar past torque buffer
	void InserAndOrderCerebellarBufferPast(edlut_ros::AnalogCompact & newMessage, std::vector<edlut_ros::AnalogCompact>  & buffer);


	// Callback function for reading torque from PD Supervisor
	void TorqueSupervisorCallback(const edlut_ros::AnalogCompact::ConstPtr& msg);

	// Callback function for reading mean torque values
	void MeanTorqueCallback(const edlut_ros::AnalogCompact::ConstPtr& msg);

	// Clean the input value queue and retrieve the last value
	// void CleanCerebellarQueue(std::priority_queue<edlut_ros::AnalogCompact, std::vector<edlut_ros::AnalogCompact>,torque_comparison > & queue,
	// 		std::vector<double> & updateVar,
	// 		double & lastTime,
	// 		double end_time);

	// Clean the input value queue and retrieve the last value
	void CleanCerebellarBuffer(double end_time);

	// Find the index of name in vector strvector. -1 if not found
	int FindJointIndex(std::vector<std::string> strvector, std::string name);


public:

	/*
	 * This function initializes a spike decoder taking the activity from the specified ros topic.
	 */
	TorqueAddition_Delay_PD_Wifi(bool wireless_simulation,
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
		double torque_reduction_factor);

	/*
	 * Update the output variables with the current time and the output activity and send the output message
	 */
	void UpdateTorque(ros::Time current_time);

	/*
	 * Destructor of the class
	 */
	virtual ~TorqueAddition_Delay_PD_Wifi();

};

#endif /* TorqueAddition_Delay_PD_Wifi_H_ */
