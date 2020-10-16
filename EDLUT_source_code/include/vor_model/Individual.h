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

#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

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
class EntrySignal;

class Individual{

public:

	 /*!
   	 * 
   	 */
	float K;

	/*!
   	 * 
   	 */
	float TC1;

	/*!
   	 * 
   	 */
	float TC2;

	/*!
   	 * Number of diffential variables.
   	 */
	static const int NStates=2;

	/*!
   	 * Differential variables (initial values {0,0}).
   	 */
	float States[NStates]; 

	/*!
   	 * Auxiliar parameters (a={1/(TC1*TC2), (TC1+TC2)/(TC1*TC2)}).
   	 */
	float A[NStates];  //a=[1/(TC1s*TC2s) (TC1s+TC2s)/(TC1s*TC2s)];
	
	/*!
   	 * Auxiliar parameters (b={0, K*TC1/(TC1*TC2)}).
   	 */
	float B[NStates];


	/*!
	 * \brief Constructor with parameters.
	 * 
	 * It creates and initializes a new individual.
	 * 
	 * \param new_K parameter K.
	 * \param new_TC1 parameter TC1.
	 * \param new_TC2 parameter TC2.
	 */
	Individual(float new_K, float new_TC1, float new_TC2);

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
	void SetParameters(float new_K, float new_TC1, float new_TC2);

	/*!
	 * \brief It evaluates the differential state of the individual.
	 * 
	 * It evaluates the differential state of the individual.
	 * 
	 * \param input input signal.
	 * \param inputState differential state variables.
	 * \param outputState output vector that stores the differential ecuation results.
	 */
	void FuncVOR(float input, float * inputState, float * outputState);

	/*!
	 * \brief It calculates the fitness function.
	 * 
	 * It calculates the fitness function.
	 * 
	 * \param entrySignal entry signal.
	 * \param Integrator integrator method.
	 * 
	 * \return fitness result.
	 */
	float FitnessVOR(EntrySignal * entrySignal, RK4_VOR * Integrator);


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
	float * CalculateOutputVOR(EntrySignal * entrySignal, RK4_VOR * Integrator);


};

#endif