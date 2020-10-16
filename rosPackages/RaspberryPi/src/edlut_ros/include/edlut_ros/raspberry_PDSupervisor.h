/***************************************************************************
 *                           PDSupervisor.h                                *
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

#ifndef RASPBERRY_PDSUPERVISOR_H_
#define RASPBERRY_PDSUPERVISOR_H_

#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <edlut_ros/AnalogCompact.h>
#include <edlut_ros/AnalogCompactDelay.h>
#include <edlut_ros/ExternalClock.h>
#include <sensor_msgs/JointState.h>


/*
 * This class defines a position supervisor
 */
class raspberry_PDSupervisor {

private:
	// ROS Node Handler
	ros::NodeHandle NodeHandler;

	// ROS CallbackQueue
	ros::CallbackQueue CallbackQueue;

	// Actual position subscriber
	ros::Subscriber robot_state_sub;

	// Joint command publisher
	ros::Publisher torque_command_pub;

	//Subscriber and external clock for synchronizer clock
	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;

	//Simulation time active
	bool use_sim_time;

	// P constants
	std::vector<double> kp;

	// D constants
	std::vector<double> kd;

	std::vector<double> current_position_value;
	ros::Time current_position_time;

	std::vector<double> current_velocity_value;
	ros::Time current_velocity_time;

	// List of joints
	std::vector<std::string> joint_list;

  // List of min and max positions from which the supervisor will begin to act.
	std::vector<double> min_position, max_position;

	// Callback function for Baxter state
	void RobotStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

	// Find index of the joint name
	int FindJointIndex(std::vector<std::string> strvector, std::string name);

  //this variable smoothes the contribution of the velocity to the torque in the
	//first instants in which the supervisor start to compensate;
	std::vector<double> smoothing_factor;


public:

	/*
	 * This function initializes the controller.
	 */
	raspberry_PDSupervisor(std::string robot_state_topic,
			std::string torque_topic,
			std::vector<std::string> joint_list,
			std::vector<double> kp,
			std::vector<double> kd,
			std::vector<double> min_position,
			std::vector<double> max_position,
			bool sim_time);

	/*
	 * This function executes a supervision step and publish the motor commands.
	 */
	void NextSupervisorStep();


	/*
	 * Destructor of the class
	 */
	virtual ~raspberry_PDSupervisor();

};

#endif /* RASPBERRY_PDSUPERVISOR_H_ */
