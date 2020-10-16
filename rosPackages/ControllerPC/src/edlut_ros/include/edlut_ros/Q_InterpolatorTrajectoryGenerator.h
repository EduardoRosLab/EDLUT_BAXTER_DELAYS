/***************************************************************************
 *                          Q_InterpolatorTrajectoryGenerator.h            *
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

#ifndef Q_INTERPOLATORTRAJECTORYGENERATOR_H_
#define Q_INTERPOLATORTRAJECTORYGENERATOR_H_

/*!
 * \file Q_InterpolatorTrajectoryGenerator.h
 *
 * \author Francisco Naveros
 * \date November 2018
 *
 * This file declares a class that load two trajectories from the same position and velocity files
 * and intorpolate a final trajectory in between both of them.
 */

#include <ros/ros.h>
#include "edlut_ros/ExternalClock.h"
#include <ros/callback_queue.h>
#include <iostream>
#include <fstream>
#include <string>

#include <vector>

class InterpolatorDynParameters;
/*!
 * \class Q_InterpolatorTrajectoryGenerator
 *
 * \brief Class interpolating a trajectory in between two base trajectories.
 *
 * This class provides the positions and velocities trajectories.
 *
 * \author Francisco Naveros
 * \date November 2018
 */
class Q_InterpolatorTrajectoryGenerator {

	private:

  //Interpolation factor
	double interpolation_factor;

	// ROS Node Handler
	ros::NodeHandle NodeHandler;

	//Subscriber and external clock for synchronizer clock
	ros::Subscriber clock_subscriber;
	ExternalClock ext_clock;

	//Callback
	ros::CallbackQueue CallbackQueue;

	//Simulation time active
	bool use_sim_time;

	//First iteration
	bool first_iteration;

	/*!
	 * Initial time of the trajectory. It is set when calling Reset function.
	 */
	ros::Time InitTime;

	/*!
	 * Baxter max position limit per joint
	 */
	std::vector<double> max_pos_amplitude;

	/*!
	 * Baxter min position limit per joint
	 */
	std::vector<double> min_pos_amplitude;

	/*!
	 * Baxter max velocity limit per joint
	 */
	std::vector<double> max_vel_amplitude;

	/*!
	 * Baxter min velocity limit per joint
	 */
	std::vector<double> min_vel_amplitude;

	/*!
	 * Number of samples of Q matrix
	 */
	int samples;


	/*!
	 * Matrix containing joints positions for the desired trajectory
	 */
	std::vector<std::vector<std::vector<double>>> Q_positions;

	/*!
	 * Matrix containing joints velocities for the desired trajectory
	 */
	std::vector<std::vector<std::vector<double>>> Qd_velocities;

	/*!
	 * File containing the joints positions for the desired trajectory
	 */
	std::ifstream file_positions;
	/*!
	 * File containing the joints positions for the desired trajectory
	 */
	std::ifstream file_velocities;


	/*!
	 * Index counter to move through Q_positions rows
	 */
	int index;

	/*!
	 * Auxiliar variable to compute the index
	 */
	double aux_index_computation;

	/*!
	 * Trajectory frequency
	 */
	double trajectory_frequency;

	/*!
	 * Inverse trajectory frequency
	 */
	double inverse_trajectory_frequency;

	/*!
	 * Time counter to know whether to move to next Q_positions row or not
	 */
	double last_time;


	InterpolatorDynParameters * controller_interpolation;
	ros::CallbackQueue * DynamicCallbackQueue;

	public:
	/*!
	 * \brief Class constructor.
	 *
	 * It creates a new object.
	 *
	 */
	Q_InterpolatorTrajectoryGenerator(bool sim_time,
			std::vector<double> max_pos_amplitude,
			std::vector<double> min_pos_amplitude,
			std::vector<double> max_vel_amplitude,
			std::vector<double> min_vel_amplitude,
			int samples,
			std::string positions_file_name,
			std::string velocities_file_name,
			double trajectory_frequency,
			ros::NodeHandle *nh,
			ros::NodeHandle *pnh,
			ros::CallbackQueue *DynamicCallbackQueue);

	/*!
	 * \brief Class desctructor.
	 *
	 * Class desctructor.
	 */
	~Q_InterpolatorTrajectoryGenerator();

	/*!
	 * \brief Set the initial time of the trajectory.
	 *
	 * This function has to be called at the time when the trajectory starts.
	 */
	ros::Time ResetGenerator();

	/*!
	 * \brief This function provides the current position and velocity according to the trajectory.
	 *
	 * This function provides the current position and velocity vectors according to the trajectory functions.
	 */
	// void GetState(std::vector<double> & CurrentPosition, std::vector<double> & CurrentVelocity);

	/*!
	 * \brief This function provides the current position and velocity according to the trajectory.
	 *
	 * This function provides the current position and velocity vectors according to the trajectory functions.
	 */
	void GetState(std::vector<double> & CurrentPosition, std::vector<double> & CurrentVelocity, double current_time);

	/*!
	 * \brief This function provides the starting position of the trajectory.
	 *
	 * This function provides the starting position of the trajectory.
	 */
	std::vector<double> GetStartingPoint();


};

#endif /*Q_INTERPOLATORTRAJECTORYGENERATOR_H_*/
