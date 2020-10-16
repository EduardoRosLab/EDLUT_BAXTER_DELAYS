/***************************************************************************
 *                           PDSupervisor.cpp                              *
 *                           -------------------                           *
 * copyright            : (C) 2018 by Francisco Naveros                    *
 * email                : fnaveros@ugr.es                                  *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

 // This is the PD Supervisor node
 // Its function is to supervise that the joints are kept within their working
 // area. Only at the event of a joint going outside the working-area, the PD
 // supervisor node adds a torque command to the cerebellar controller torques.


#include <edlut_ros/raspberry_PDSupervisor.h>
#include <sensor_msgs/JointState.h>

#include <edlut_ros/AnalogCompact.h>
#include <edlut_ros/AnalogCompactDelay.h>
#include <baxter_core_msgs/JointCommand.h>

#include <ros/ros.h>
#include <vector>
#include <cmath>

int raspberry_PDSupervisor::FindJointIndex(std::vector<std::string> strvector, std::string name){
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


void raspberry_PDSupervisor::RobotStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
	for (unsigned int i=0; i<msg->name.size(); ++i){
			int index = this->FindJointIndex(this->joint_list, msg->name[i]);
			if (index!=-1) {
				this->current_position_value[index] = msg->position[i];
				this->current_velocity_value[index] = msg->velocity[i];
			}
		}
		return;
}


raspberry_PDSupervisor::raspberry_PDSupervisor(std::string robot_state_topic,
		std::string torque_topic,
		std::vector<std::string> joint_list,
		std::vector<double> kp,
		std::vector<double> kd,
		std::vector<double> min_position,
		std::vector<double> max_position,
		bool sim_time):
				NodeHandler(),
				CallbackQueue(),
				joint_list(joint_list),
				kp(kp),
				kd(kd),
				min_position(min_position),
				max_position(max_position),
				use_sim_time(sim_time) {

	this->NodeHandler.setCallbackQueue(&this->CallbackQueue);

	// Initialize the vectors of publishers
	this->robot_state_sub = this->NodeHandler.subscribe(robot_state_topic, 1e9, &raspberry_PDSupervisor::RobotStateCallback, this);
	ROS_DEBUG("raspberry PD Supervisor: Subscribed to topic %s ", robot_state_topic.c_str());

	this->clock_subscriber = this->NodeHandler.subscribe("/clock_sync", 1000, &ExternalClock::ClockCallback, &ext_clock);

	this->current_position_time = ros::Time(0.0);
	this->current_velocity_time = ros::Time(0.0);

	this->current_position_value = std::vector<double>(this->joint_list.size(),0.0);
	this->current_velocity_value = std::vector<double>(this->joint_list.size(),0.0);


	this->torque_command_pub = this->NodeHandler.advertise<edlut_ros::AnalogCompact>(torque_topic, 1e9);
	ROS_DEBUG("PD Supervisor: Advertised topic %s for torque command", torque_topic.c_str());

	this->smoothing_factor = std::vector<double>(this->joint_list.size(),0.0);
}


raspberry_PDSupervisor::~raspberry_PDSupervisor() {
	// TODO Auto-generated destructor stub
}


/*
 * This function executes a controller step. It returns
 * true if the simulation is not ended yet and false otherwise.
 */
void raspberry_PDSupervisor::NextSupervisorStep(){

	// Update the current state with the last messages
	this->CallbackQueue.callAvailable(ros::WallDuration(0.001));

	edlut_ros::AnalogCompact analog;

	if (this->use_sim_time){
		analog.header.stamp = ext_clock.GetLastConfirmedTime();
	}
	else{
		analog.header.stamp = ros::Time::now();
	}
	analog.names = this->joint_list;
	analog.data.resize(this->joint_list.size());

	for (unsigned int i=0; i<this->joint_list.size(); ++i){
		analog.data[i]=0;
		if (this->current_position_value[i] < this->min_position[i]){
			if(this->smoothing_factor[i] < 1){
				this->smoothing_factor[i]+=0.02;
			}
			analog.data[i] = this->kp[i] * (this->min_position[i]-this->current_position_value[i]) - this->smoothing_factor[i] * this->kd[i]*this->current_velocity_value[i];
		}else if(this->current_position_value[i] > this->max_position[i]){
			if(this->smoothing_factor[i] < 1){
				this->smoothing_factor[i]+=0.02;
			}
			analog.data[i] = this->kp[i] * (this->max_position[i]-this->current_position_value[i]) - this->smoothing_factor[i] * this->kd[i]*this->current_velocity_value[i];
		}else{
			this->smoothing_factor[i]=0;
		}
	}
	this->torque_command_pub.publish(analog);
	ROS_DEBUG("PD Supervisor: Sent torque command.");
}
