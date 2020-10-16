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

#ifndef ENTRYSIGNAL_H_
#define ENTRYSIGNAL_H_

/*!
 * \file EntrySignal.h
 *
 * \author Niceto Luque
 * \author Francisco Naveros
 * \date October 2015
 *
 * This file declares a class which implements the entry signal for the VOR experiment.
 */


class EntrySignal{

public: 

	/*!
   	 * Vector that stores the signal values.
   	 */
	float * Signal;

	/*!
   	 * Number of elements inside the vector.
   	 */
	int NElements;

	/*!
   	 * Signal step time.
   	 */
	float StepTime;

	/*!
   	 * Initial time.
   	 */
	float TInit;

	/*!
   	 * Final time.
   	 */
	float TEnd;

	/*!
   	 * signal type (0=sinusoidal, 1=square).
   	 */
	int Type;

	/*!
   	 * Signal amplitude.
   	 */
	float Amplitude;

	/*!
   	 * Signal Frequency.
   	 */
	float Frequency;



	/*!
	 * \brief Constructor with parameters.
	 * 
	 * It creates and initializes a new entry signal.
	 * 
	 * \param NewStepTime step time.
	 * \param NewTInit initial time.
	 * \param NewTEnd final time.
	 * \param NewType signal type 0=sinusoidal, 1=square).
	 * \param NewAmplitude signal amplitude.
	 */
	EntrySignal(float NewStepTime, float NewTInit, float NewTEnd, int NewType, float NewAmplitude,float NewFrequency);

	/*!
	 * \brief Class destructor.
	 * 
	 * It destroies an object of this class.
	 */
	~EntrySignal();

	/*!
	 * \brief It gets the entry signal.
	 * 
	 * It gets the entry signal.
	 * 
	 * \return The entry signal.
	 */
	float * GetSignal();

	/*!
	 * \brief It gets the number of elements.
	 * 
	 * It gets the number of elements.
	 * 
	 * \return The number of elements.
	 */
	int GetNElements();

};

#endif