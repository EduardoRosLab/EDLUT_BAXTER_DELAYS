/***************************************************************************
 *                           RK4_VOR.h                                     *
 *                           -------------------                           *
 * copyright            : (C) 2015 by Niceto Luque and Francisco Naveros   *
 * email                : nluque@ugr.es and fnaveros@ugr.es                *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef RK4_VOR_H_
#define RK4_VOR_H_

/*!
 * \file RK4_VOR.h
 *
 * \author Niceto Luque
 * \author Francisco Naveros
 * \date October 2015
 *
 * This file declares a class which implements fourth order Runge-Kutta integrator method.
 */

class Individual;

class RK4_VOR{
	public:

   		/*!
   		 * \brief Default constructor.
   		 * 
   		 * It creates and initializes a new spike object.
   		 */
		RK4_VOR();

   		/*!
   		 * \brief Class destructor.
   		 * 
   		 * It destroies an object of this class.
   		 */
		~RK4_VOR();

		/*!
		 * \brief It calculates the next differential value for one individual with a determined input.
		 *
		 * It calculates the next differential value for one individual with a determined input.
		 *
		 * \param individual individual object.
		 * \param StepTime integration step time 
		 * \param input input signal
		 */
		void NextDifferentialEcuationValues(Individual * individual, double StepTime, double input);

};

#endif /* RK4_VOR_H_ */
