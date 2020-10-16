/***************************************************************************
 *                           Individual.h                                  *
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

#ifndef INDIVIDUALVOR_H_
#define INDIVIDUALVOR_H_

/*!
 * \file Individual.h
 *
 * \author Niceto Luque
 * \author Francisco Naveros
 * \date October 2015
 *
 * This file declares a class which implements a individual of the Genetic algoritm.
 */

class RK4_VOR;
class EntrySignalVOR;

class Individual{

public:

	 /*!
   	 * 
   	 */
	double K;

	/*!
   	 * 
   	 */
	double TC1;

	/*!
   	 * 
   	 */
	double TC2;

	/*!
   	 * Number of diffential variables.
   	 */
	static const int NStates=2;

	/*!
   	 * Differential variables (initial values {0,0}).
   	 */
	double States[NStates]; 

	/*!
   	 * Auxiliar parameters (a={1/(TC1*TC2), (TC1+TC2)/(TC1*TC2)}).
   	 */
	double A[NStates];  //a=[1/(TC1s*TC2s) (TC1s+TC2s)/(TC1s*TC2s)];
	
	/*!
   	 * Auxiliar parameters (b={0, K*TC1/(TC1*TC2)}).
   	 */
	double B[NStates];


	/*!
	 * \brief Constructor with parameters.
	 * 
	 * It creates and initializes a new individual.
	 * 
	 * \param new_K parameter K.
	 * \param new_TC1 parameter TC1.
	 * \param new_TC2 parameter TC2.
	 */
	Individual(double new_K, double new_TC1, double new_TC2);

	/*!
	 * \brief Copy constructor.
	 * 
	 * It creates and initializes a new individual coping it from another individual.
	 * 
	 * \param new_individual individual.
	 */
	Individual(Individual * new_Individual);

	/*!
	 * \brief Class destructor.
	 * 
	 * It destroies an object of this class.
	 */
	~Individual();

	/*!
	 * \brief It sets the individual parameters.
	 * 
	 * It sets the individual parameters.
	 * 
	 * \param new_K
	 * \param new_TC1
	 * \param new_TC2
	 */
	void SetParameters(double new_K, double new_TC1, double new_TC2);

	/*!
	 * \brief It evaluates the differential state of the individual.
	 * 
	 * It evaluates the differential state of the individual.
	 * 
	 * \param input input signal.
	 * \param inputState differential state variables.
	 * \param outputState output vector that stores the differential ecuation results.
	 */
	void FuncVOR(double input, double * inputState, double * outputState);

	/*!
	 * \brief It calculates the output values.
	 * 
	 * It calculates the output values.
	 * 
	 * \param entrySignal entry signal.
	 * \param Integrator integrator method.
	 * 
	 * \return output values.
	 */
	void CalculateOutputVOR(EntrySignalVOR * entrySignalVOR, RK4_VOR * Integrator, double * inputCereb,double * OutputVOR);


};

#endif