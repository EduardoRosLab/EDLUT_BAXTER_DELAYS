/***************************************************************************
 *                           DelayGeneratorDynParams.cpp                   *
 *                           -------------------                           *
 * copyright            : (C) 2020 by Ignacio Abadia                       *
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

// Dynamic parameters for the delay generator node.


#include "edlut_ros/DelayGeneratorDynParams.h"
#include <iostream>


DelayGeneratorDynParams::DelayGeneratorDynParams(ros::NodeHandle *nh, ros::NodeHandle *pnh) : nh_(nh), pnh(pnh), dr_srv_(*pnh), min_delay(0.0), max_delay(0.0) {
	dynamic_reconfigure::Server<edlut_ros::DelayGeneratorDynParametersConfig>::CallbackType cb;
  cb = boost::bind(&DelayGeneratorDynParams::callback, this, _1, _2);
  dr_srv_.setCallback(cb);
}

DelayGeneratorDynParams::~DelayGeneratorDynParams() {
	// TODO Auto-generated destructor stub
}

void DelayGeneratorDynParams::callback(edlut_ros::DelayGeneratorDynParametersConfig &config, uint32_t level) {
	this->min_delay = config.min_delay;
	this->max_delay = config.max_delay;
	this->fixed_delay = config.fixed_delay;
}

double DelayGeneratorDynParams::GetMinDelay(){
	//std::cout<<"DYN PARAM GET PAUSED";
	return this->min_delay;
}

double DelayGeneratorDynParams::GetMaxDelay(){
	//std::cout<<"DYN PARAM GET PAUSED";
	return this->max_delay;
}

double DelayGeneratorDynParams::GetFixedDelay(){
	//std::cout<<"DYN PARAM GET PAUSED";
	return this->fixed_delay;
}
