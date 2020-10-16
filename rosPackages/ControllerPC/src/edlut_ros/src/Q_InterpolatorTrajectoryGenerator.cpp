/***************************************************************************
 *                           Q_InterpolatorTrajectoryGenerator.cpp         *
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

 // This is the interpolator trajectory generator node.
 // It generates trajectory position and velocity signals for every joint out of
 // .txt files in the format (one file for position, one for velocity):
 // J0 J1 J2 J3 J4 J5 J6
 // ...
 // J0 J1 J2 J3 J4 J5 J6
 // In the .txt files two trajectories are specified, one after the other
 // (that is, after the last sample of the first trajectory, the second trajectory
 // is written). The interpolation is designed for circular trajectories. If
 // trajectory 1 has a radius R1 and trajectory 2 has a radius R2, this node can
 // generate the circular trajectories with a radius varying in the range [R1, R2]
 // by interpolation of the joint coordinates. The radius can be modified online
 // via ROS dynamic reconfigure parameters.

#include "edlut_ros/Q_InterpolatorTrajectoryGenerator.h"
#include <edlut_ros/InterpolatorDynParameters.h>

#include <cmath>
#include <vector>
#include <string>

Q_InterpolatorTrajectoryGenerator::Q_InterpolatorTrajectoryGenerator(bool sim_time, std::vector<double> max_pos_amplitude, std::vector<double> min_pos_amplitude,
std::vector<double> max_vel_amplitude, std::vector<double> min_vel_amplitude, int samples, std::string positions_file_name, std::string velocities_file_name,
double trajectory_frequency, ros::NodeHandle *nh, ros::NodeHandle *pnh, ros::CallbackQueue *DynamicCallbackQueue):
use_sim_time(sim_time), max_pos_amplitude(max_pos_amplitude), min_pos_amplitude(min_pos_amplitude), max_vel_amplitude(max_vel_amplitude),
min_vel_amplitude(min_vel_amplitude), samples(samples), trajectory_frequency(trajectory_frequency), interpolation_factor(0.0){
	this->NodeHandler.setCallbackQueue(&CallbackQueue);
	if (use_sim_time){
		this->clock_subscriber = this->NodeHandler.subscribe("/clock_sync", 1000, &ExternalClock::ClockCallback, &ext_clock);
	}
	this->index = 0;
	this->aux_index_computation = this->trajectory_frequency*(this->samples-1);
	this->inverse_trajectory_frequency=1.0/this->trajectory_frequency;
	this->last_time = 0.0;
	this->first_iteration = true;
	this->file_positions.open(positions_file_name);
	this->file_velocities.open(velocities_file_name);
	std::string word;
	std::vector<double> row_positions;
	std::vector<double> row_velocities;
	std::vector<std::vector<double>> trajectory_positions;
	std::vector<std::vector<double>> trajectory_velocities;

	this->controller_interpolation = new InterpolatorDynParameters(nh, pnh);
	this->DynamicCallbackQueue = DynamicCallbackQueue;

	int joint=0;
	int counter = 0;

	std::cout << "----LOADING POSITION FILE---- " << "\n";
	while (file_positions >> word){
		row_positions.push_back(std::stod(word));
		joint++;
		if (joint == 7){
			counter++;
			trajectory_positions.push_back(row_positions);
			row_positions.clear();
			joint = 0;
			if(counter==samples){
				this->Q_positions.push_back(trajectory_positions);
				trajectory_positions.clear();
				counter=0;
			}
		}

	}

	std::cout << "----LOADING VELOCITY FILE---- " << "\n";

	joint=0;
	counter = 0;
	while (file_velocities >> word){
		row_velocities.push_back(std::stod(word));
		joint++;
		if (joint == 7){
			counter++;
			trajectory_velocities.push_back(row_velocities);
			row_velocities.clear();
			joint = 0;
			if(counter==samples){
				this->Qd_velocities.push_back(trajectory_velocities);
				trajectory_velocities.clear();
				counter=0;
			}
		}
	}

	std::cout << "----FINISH LOADING----" << "\n";

	this->file_positions.close();
	this->file_velocities.close();

}


Q_InterpolatorTrajectoryGenerator::~Q_InterpolatorTrajectoryGenerator() {
	// TODO Auto-generated destructor stub
//	delete this->controller_interpolation;
}

ros::Time Q_InterpolatorTrajectoryGenerator::ResetGenerator(){
	CallbackQueue.callAvailable(ros::WallDuration(0.001));
	if(use_sim_time){
		this->InitTime = ext_clock.GetLastConfirmedTime();
	}
	else{
		this->InitTime = ros::Time::now();
	}
	return this->InitTime;
}


void Q_InterpolatorTrajectoryGenerator::GetState(std::vector<double> & CurrentPosition, std::vector<double> & CurrentVelocity, double current_time){
	this->DynamicCallbackQueue->callAvailable(ros::WallDuration(0.0001));
	this->interpolation_factor = this->controller_interpolation->GetInterpolationFactor();

	double ElapsedTime = current_time - this->InitTime.toSec();
	if(ElapsedTime > this->last_time || this->first_iteration){
		this->last_time = ElapsedTime;
		this->index = fmod(ElapsedTime, this->inverse_trajectory_frequency) * this->aux_index_computation;
		for (unsigned int i=0; i<7; ++i){
			 CurrentPosition[i] = this->Q_positions[0][this->index][i] + (this->Q_positions[1][this->index][i] - this->Q_positions[0][this->index][i]) * this->interpolation_factor;
			 CurrentVelocity[i] = this->Qd_velocities[0][this->index][i] + (this->Qd_velocities[1][this->index][i] - this->Qd_velocities[0][this->index][i]) * this->interpolation_factor;
		}
		this->first_iteration = false;
	}


	return;
}

std::vector<double> Q_InterpolatorTrajectoryGenerator::GetStartingPoint(){
	std::vector<double> CurrentPosition(7);
	std::vector<double> CurrentVelocity(7);
	this->first_iteration = true;
	this->GetState(CurrentPosition, CurrentVelocity, 0.0);

	return CurrentPosition;
}
