/***************************************************************************
 *                           FixedStepSRM.h                                *
 *                           -------------------                           *
 * copyright            : (C) 2013 by Francisco Naveros                    *
 * email                : fnaveros@ugr.es                                  *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef FIXEDSTEPSRM_H_
#define FIXEDSTEPSRM_H_

/*!
 * \file IntegrationMethod.h
 *
 * \author Francisco Naveros
 * \date May 2013
 *
 * This file declares a class which implement a fixed step integration method for SRM neuron model in CPU.
 * This class only store the value of the integration step size.
 */

#include "./IntegrationMethod.h"

class TimeDrivenNeuronModel;



/*!
 * \class FixedStepSRM
 *
 * \brief Fixed step integration methods in CPU for SRM neuron model. This class only store the value of the
 * integration step size.
 *
 * \author Francisco Naveros
 * \date May 2013
 */
class FixedStepSRM : public IntegrationMethod {

	public:


		/*!
		 * \brief Default constructor.
		 *
		 * It generates a new FixedStepSRM object.
		 *
		 */
		FixedStepSRM();

		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		~FixedStepSRM();

		
		/*!
		 * \brief It calculate the new neural state variables for a defined elapsed_time.
		 *
		 * It calculate the new neural state variables for a defined elapsed_time.
		 *
		 * \param index for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 * \param NeuronState neuron state variables of one neuron.
		 * \return Retrun if the neuron spike
		 */
		void NextDifferentialEquationValues(int index, float * NeuronState) {
		}

		/*!
		* \brief It calculate the new neural state variables for a defined elapsed_time and all the neurons.
		*
		* It calculate the new neural state variables for a defined elapsed_time and all the neurons.
		*
		* \return Retrun if the neuron spike
		*/
		void NextDifferentialEquationValues(){
		}

		/*!
		* \brief It calculate the new neural state variables for a defined elapsed_time and all the neurons that require integration.
		*
		* It calculate the new neural state variables for a defined elapsed_time and all the neurons that requre integration.
		*
		* \param integration_required array that sets if a neuron must be integrated (for lethargic neuron models)
		* \return Retrun if the neuron spike
		*/
		void NextDifferentialEquationValues(bool * integration_required, double CurrentTime){
		}

		/*!
		 * \brief It prints the integration method info.
		 *
		 * It prints the current integration method characteristics.
		 *
		 * \param out The stream where it prints the information.
		 *
		 * \return The stream after the printer.
		 */
		 ostream & PrintInfo(ostream & out);


		/*!
		 * \brief It initialize the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * It initialize the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * \param N_neuron number of neurons in the neuron model.
		 * \param inicialization vector with initial values.
		 */
		 void InitializeStates(int N_neurons, float * inicialization){}


		/*!
		 * \brief It reset the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * It reset the state of the integration method for method with memory (e.g. BDF1ad, BDF2, BDF3, etc.).
		 *
		 * \param index indicate which neuron must be reseted.
		 */
		void resetState(int index){}


		/*!
		 * \brief It loads the integration method parameters.
		 *
		 * It loads the integration method parameters from the file that define the parameter of the neuron model.
		 *
		 * \param model time driven neuron model associated to this integration method.
		 * \param Pointer to a neuron description file (*.cfg). At the end of this file must be included 
		 *  the integration method type and its parameters.
		 * \param Currentline line inside the neuron description file where start the description of the integration method parameter. 
		 * \param fileName file name.
		 *
		 * \throw EDLUTFileException If something wrong has happened in the file load.
		 */
		void loadParameter(TimeDrivenNeuronModel * model, FILE *fh, long * Currentline, string fileName) throw (EDLUTFileException);


		/*!
		 * \brief It sets the required parameter in the adaptative integration methods (Bifixed_Euler, Bifixed_RK2, Bifixed_RK4, Bifixed_BDF1 and Bifixed_BDF2).
		 *
		 * It sets the required parameter in the adaptative integration methods (Bifixed_Euler, Bifixed_RK2, Bifixed_RK4, Bifixed_BDF1 and Bifixed_BDF2).
		 *
		 * \param startVoltageThreshold, when the membrane potential reaches this value, the multi-step integration methods change the integration
		 *  step from elapsedTimeInNeuronModelScale to bifixedElapsedTimeInNeuronModelScale.
		 * \param endVoltageThreshold, when the membrane potential reaches this value, the multi-step integration methods change the integration
		 *  step from bifixedElapsedTimeInNeuronModelScale to ElapsedTimeInNeuronModelScale after timeAfterEndVoltageThreshold in seconds.
		 * \param timeAfterEndVoltageThreshold, time in seconds that the multi-step integration methods maintain the bifixedElapsedTimeInNeuronModelScale
		 *  after the endVoltageThreshold
		 */
		void SetBiFixedStepParameters(float startVoltageThreshold, float endVoltageThreshold, float timeAfterEndVoltageThreshold){};

		/*!
		 * \brief It calculates the conductance exponential values for time driven neuron models.
		 *
		 * It calculates the conductance exponential values for time driven neuron models.
		 */
		void Calculate_conductance_exp_values(){};

};

#endif /* FIXEDSTEPSRM_H_ */
