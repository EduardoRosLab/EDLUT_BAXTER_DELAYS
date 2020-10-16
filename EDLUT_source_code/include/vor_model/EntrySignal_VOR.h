/***************************************************************************
 *                           EntrySignal.h                                 *
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

#ifndef ENTRYSIGNALVOR_H_
#define ENTRYSIGNALVOR_H_

/*!
 * \file EntrySignal.h
 *
 * \author Niceto Luque
 * \author Francisco Naveros
 * \date October 2015
 *
 * This file declares a class which implements the entry signal for the VOR experiment.
 */


class EntrySignalVOR{

public: 

	/*!
   	 * Vector that stores the signal values.
   	 */
	double * SignalVOR;

	/*!
   	 * Signal step time.
   	 */
	double StepTime;
	
	/*!
   	 * Signal Current Simulation Time.
   	 */
	double Tsimul;
	/*!
   	 * signal type (0=sinusoidal, 1=square).
   	 */
	int Type;

	/*!
   	 * Signal amplitude.
   	 */
	double Amplitude;

	/*!
   	 * Signal Frequency.
   	 */
	double Frequency;

	/*!
   	 * Number of Microzones.
   	 */
	int NJoints;


	/*!
	* \brief Empty constructor.
	*
	* Empty constructor.
	*/
	EntrySignalVOR();


	/*!
	 * \brief Constructor with parameters.
	 * 
	 * It creates and initializes a new entry signal.
	 * 
	 * \param NewTimeStep step time.
	 * \param Newtsimul actual simulation step.
	 * \param NewAmplitude signal amplitude.
	 * \param NewFrequency signal frequency
	 * \param NewType signal type 0=sinusoidal, 1=square).
	 * \param NewJoint number of microzones).
	 
	 */
	 EntrySignalVOR(double NewTimeStep, double Newtsimul,double NewAmplitude,double NewFrequency,int NewType,int NewJoints);
	/*!
	 * \brief Class destructor.
	 * 
	 * It destroies an object of this class.
	 */
	~EntrySignalVOR();

	/*!
	 * \brief It gets the entry signal.
	 * 
	 * It gets the entry signal.
	 * 
	 * \return The entry signal.
	 */
	double * GetSignalVOR();

	/*!
	* \brief It gets an element of the entry signal.
	*
	* It gets and element of the entry signal.
	*
	* \return An element of the entry signal.
	*/
	double GetSignalVOR(int index);

	/*!
	 * \brief It gets the number of microzones.
	 * 
	 * It gets the number of microzones.
	 * 
	 * \return The number of microzones.
	 */
	int GetNElements();

};

#endif