/***************************************************************************
 *                           InterpolatorDynParameters.cpp                     *
 *                           -------------------                           *
 * copyright            : (C) 2018 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es                                   *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

 // Used for modifying the dynamic parameters of the trajectory_interpolator
 // node (modify the radius of the trajectory)


#include "edlut_ros/InterpolatorDynParameters.h"
#include <iostream>


InterpolatorDynParameters::InterpolatorDynParameters(ros::NodeHandle *nh, ros::NodeHandle *pnh) : nh_(nh), pnh(pnh), dr_srv_(*pnh), interpolation_factor(0.5) {
	dynamic_reconfigure::Server<edlut_ros::InterpolatorDynParametersConfig>::CallbackType cb;
  cb = boost::bind(&InterpolatorDynParameters::callback, this, _1, _2);
  dr_srv_.setCallback(cb);
	std::cout << "CONSTRUCTOR INTERPOLATOR" << std::endl;
}


InterpolatorDynParameters::~InterpolatorDynParameters() {
	// TODO Auto-generated destructor stub
}


void InterpolatorDynParameters::callback(edlut_ros::InterpolatorDynParametersConfig &config, uint32_t level) {
	this->interpolation_factor = config.Interpolation_factor;
	ROS_INFO("INTERPOLATION %f", this->interpolation_factor);
	//ROS_INFO("SYNCHRONIZER CALLBACK");
	//std::cout<<"SYNCHRONIZER DYN Callback";

	//return;
}

double InterpolatorDynParameters::GetInterpolationFactor(){
	//std::cout<<"DYN PARAM GET TIME STAMP";
	return this->interpolation_factor;
}
