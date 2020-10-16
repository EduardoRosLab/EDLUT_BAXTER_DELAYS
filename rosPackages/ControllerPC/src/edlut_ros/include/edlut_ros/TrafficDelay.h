/***************************************************************************
 *                           TrafficDelay.h                                *
 *                           -------------------                           *
 * copyright            : (C) 2019 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es                  			           *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef TRAFFICDELAY_H_
#define TRAFFICDELAY_H_

#include "ros/ros.h"
#include <edlut_ros/AnalogCompactDelay.h>


class TrafficDelay{
private:
	double msgDelay = 0.0;

public:
	TrafficDelay(): msgDelay(0.0) {}

	void InputCallback (const edlut_ros::AnalogCompactDelay::ConstPtr& msg){
		this->msgDelay = msg->delay;
		return;
	}

	double GetLastDelay(){
		return this->msgDelay;
	}
};

#endif
