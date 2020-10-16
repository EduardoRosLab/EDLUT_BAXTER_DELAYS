/***************************************************************************
 *                          DelayGeneratorDynParams.h                      *
 *                           -------------------                           *
 * copyright            : (C) 2020 by Ignacio Abadia                       *
 * email                : iabadia@ugr.es                              		 *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef DELAYGENERATORDYNPARAMS_H_
#define DELAYGENERATORDYNPARAMS_H_

/*!
 * \file DelayGeneratorDynParams.h
 *
 * \author Ignacio Abadia
 * \date May 2020
 *
 * This file declares a class for dynamically change some parameters of the delay generator node
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <edlut_ros/DelayGeneratorDynParametersConfig.h>
#include <ros/callback_queue.h>


/*!
 * \class DelayGeneratorDynParams
 *
 * \brief Class to modify dynamic parameters of the synchronizer node
 *
 * This class allows the user to stop the synchronizer node or make it run until a
 * given time stamp
 * \author Ignacio Abadia
 * \date May 2018
 */

 class DelayGeneratorDynParams{
 private:
 	double min_delay, max_delay, fixed_delay;

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<edlut_ros::DelayGeneratorDynParametersConfig> dr_srv_;
  //! ROS node handle.
  ros::NodeHandle *nh_;
  //! ROS node handle.
  ros::NodeHandle *pnh;
 public:
	 /*!
		* \brief Class constructor.
		*
		* It creates a new object.
		*
		*/
	 DelayGeneratorDynParams(ros::NodeHandle *nh, ros::NodeHandle *pnh);

	/*!
	 * \brief Class desctructor.
	 *
	 * Class desctructor.
	 */
	 ~DelayGeneratorDynParams();


   /*!
    * \brief This function stores the values of the dynamic parameters given by the user
    *
    */
   void callback(edlut_ros::DelayGeneratorDynParametersConfig &config, uint32_t level);


	 /*!
	  * \brief This function returns the value of the minimum delay
	  *
	  */
	 double GetMinDelay();

   /*!
    * \brief This function returns the value of the maximum delay
    *
    */
   double GetMaxDelay();

   /*!
    * \brief This function returns the value of a fixed delay. A fixed delay can be used instead of
    * a delay between a min and max limit.
    *
    */
   double GetFixedDelay();

};

#endif /*DELAYGENERATORDYNPARAMS_H_*/
