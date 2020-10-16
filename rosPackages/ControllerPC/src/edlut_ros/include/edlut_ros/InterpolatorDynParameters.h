/***************************************************************************
 *                          InterpolatorDynParameters.h                        *
 *                           -------------------                           *
 * copyright            : (C) 2018 by Ignacio Abadia                       *
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

#ifndef INTERPOLATORDYNPARAMETERS_H_
#define INTERPOLATORDYNPARAMETERS_H_


#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <edlut_ros/InterpolatorDynParametersConfig.h>
#include <ros/callback_queue.h>


 class InterpolatorDynParameters{
 private:
 	double interpolation_factor;

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<edlut_ros::InterpolatorDynParametersConfig> dr_srv_;
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
	 InterpolatorDynParameters(ros::NodeHandle *nh, ros::NodeHandle *pnh);

   // InterpolatorDynParameters(ros::NodeHandle *nh);


	/*!
	 * \brief Class desctructor.
	 *
	 * Class desctructor.
	 */
	 ~InterpolatorDynParameters();

   /*!
    * \brief This function stores the values of the dynamic parameters given by the user
    *
    */
   void callback(edlut_ros::InterpolatorDynParametersConfig &config, uint32_t level);


	 /*!
	  * \brief This function returns the value at which the synchronizer should stop
	  *
	  */
	 double GetInterpolationFactor();
};

#endif /*INTERPOLATORDYNPARAMETERS_H_*/
