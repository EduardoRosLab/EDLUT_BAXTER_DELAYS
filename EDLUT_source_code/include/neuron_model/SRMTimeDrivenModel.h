/***************************************************************************
 *                           SRMTimeDrivenModel.h                          *
 *                           -------------------                           *
 * copyright            : (C) 2011 by Jesus Garrido and Francisco Naveros  *
 * email                : jgarrido@atc.ugr.es, fnaveros@ugr.es             *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef SRMTIMEDRIVENMODEL_H_
#define SRMTIMEDRIVENMODEL_H_

/*!
 * \file SRMTimeDrivenModel.h
 *
 * \author Jesus Garrido
 * \author Francisco Naveros
 * \date January 2011
 *
 * This file declares a class which implements a SRM (Spike response model) time-driven neuron
 * model.
 *
 * \note: this class has been modified to use the class VectorSRMState and VectorBufferState
 * instead of SRMState and BufferState.
 */

#include "./TimeDrivenNeuronModel.h"

#include "../spike/EDLUTFileException.h"

class VectorSRMState;
class VectorBufferedState;
class Interconnection;

/*!
 * \class SRMTimeDrivenModel
 *
 * \brief Spike Response time-driven neuron model non-based in look-up tables.
 *
 * This class implements the behavior of a neuron in a spiking neural network.
 * It includes internal model functions which define the behavior of the model
 * (initialization, update of the state, synapses effect...).
 * This behavior is calculated based in a buffer of activity.
 *
 * \author Jesus Garrido
 * \date January 2010
 */
class SRMTimeDrivenModel: public TimeDrivenNeuronModel {

	private:

		/*!
		 * \brief Number of channels in the neuron model
		 */
		int NumberOfChannels;

		/*!
		 * \brief Decay time constant of the EPSP
		 */
		float * tau;

		/*!
		 * \brief Resting potential
		 */
		float vr;

		/*!
		 * \brief Synaptic efficacy
		 */
		float * W;

		/*!
		 * \brief Spontaneous firing rate
		 */
		float r0;

		/*!
		 * \brief Probabilistic threshold potential
		 */
		float v0;

		/*!
		 * \brief Gain factor
		 */
		float vf;

		/*!
		 * \brief Absolute refractory period
		 */
		float tauabs;

		/*!
		 * \brief Relative refractory period
		 */
		float taurel;


	protected:
		/*!
		 * \brief It calculates the potential difference between resting and the potential in the defined time.
		 *
		 * It calculates the potential difference between resting and the potential in the defined time.
		 *
		 * \param index The cell index inside the VectorSRMState.
		 * \param State Cells state vector.
		 *
		 * \return The potential difference between resting and the potential in the defined time.
		 */
		float PotentialIncrement(int index, VectorSRMState * State);

		/*!
		 * \brief It checks if an spike is fired in the defined time.
		 *
		 * It checks if an spike is fired in the defined time.
		 *
		 * \param index The cell index inside the VectorSRMState.
		 * \param State Cell current state.
		 * \param CurrentTime Current simulation time.
		 *
		 * \return True if an spike is fired in the defined time. False in otherwise.
		 */
		bool CheckSpikeAt(int index, VectorSRMState * State, double CurrentTime);

		/*!
		 * \brief It loads the neuron model description.
		 *
		 * It loads the neuron type description from the file .cfg.
		 *
		 * \param ConfigFile Name of the neuron description file (*.cfg).
		 *
		 * \throw EDLUTFileException If something wrong has happened in the file load.
		 */
		void LoadNeuronModel(string ConfigFile) throw (EDLUTFileException);

		/*!
		 * \brief It abstracts the effect of an input spike in the cell.
		 *
		 * It abstracts the effect of an input spike in the cell.
		 *
		 * \param index The cell index inside the VectorSRMState.
		 * \param State Cell current state.
		 * \param InputConnection Input connection from which the input spike has got the cell.
		 */
		void SynapsisEffect(int index, Interconnection * InputConnection);

	public:
		/*!
		 * \brief Default constructor with parameters.
		 *
		 * It generates a new neuron model object.
		 *
		 * \param NeuronTypeID Neuron type identificator
		 * \param NeuronModelID Neuron model identificator.
		 */
		SRMTimeDrivenModel(string NeuronTypeID, string NeuronModelID);

		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		~SRMTimeDrivenModel();

		/*!
		 * \brief It loads the neuron model description and tables (if necessary).
		 *
		 * It loads the neuron model description and tables (if necessary).
		 *
		 * \throw EDLUTFileException If something wrong has happened in the file load.
		 */
		virtual void LoadNeuronModel() throw (EDLUTFileException);

		/*!
		 * \brief It return the Neuron Model VectorNeuronState 
		 *
		 * It return the Neuron Model VectorNeuronState 
		 *
		 */
		virtual VectorNeuronState * InitializeState();


		/*!
		 * \brief It processes a propagated spike (input spike in the cell).
		 *
		 * It processes a propagated spike (input spike in the cell).
		 *
		 * \note This function doesn't generate the next propagated spike. It must be externally done.
		 *
		 * \param inter the interconection which propagate the spike
		 * \param time the time of the spike.
		 *
		 * \return A new internal spike if someone is predicted. 0 if none is predicted.
		 */
		virtual InternalSpike * ProcessInputSpike(Interconnection * inter, double time);


		/*!
		 * \brief Update the neuron state variables.
		 *
		 * It updates the neuron state variables.
		 *
		 * \param index The cell index inside the VectorNeuronState. if index=-1, updating all cell.
		 * \param CurrentTime Current time.
		 *
		 * \return True if an output spike have been fired. False in other case.
		 */
		virtual bool UpdateState(int index, double CurrentTime);


		/*!
		 * \brief It gets the neuron output activity type (spikes or currents).
		 *
		 * It gets the neuron output activity type (spikes or currents).
		 *
		 * \return The neuron output activity type (spikes or currents).
		 */
		enum NeuronModelOutputActivityType GetModelOutputActivityType();

		/*!
		 * \brief It gets the neuron input activity types (spikes and/or currents or none).
		 *
		 * It gets the neuron input activity types (spikes and/or currents or none).
		 *
		 * \return The neuron input activity types (spikes and/or currents or none).
		 */
		enum NeuronModelInputActivityType GetModelInputActivityType();

		/*!
		 * \brief It prints the time-driven model info.
		 *
		 * It prints the current time-driven model characteristics.
		 *
		 * \param out The stream where it prints the information.
		 *
		 * \return The stream after the printer.
		 */
		virtual ostream & PrintInfo(ostream & out);


		/*!
		 * \brief It initialice VectorNeuronState.
		 *
		 * It initialice VectorNeuronState.
		 *
		 * \param N_neurons cell number inside the VectorNeuronState.
		 * \param OpenMPQueueIndex openmp index
		 */
		void InitializeStates(int N_neurons, int OpenMPQueueIndex);


		/*!
		 * \brief It evaluates if a neuron must spike.
		 *
		 * It evaluates if a neuron must spike.
		 *
		 * \param previous_V previous membrane potential
		 * \param NeuronState neuron state variables.
		 * \param index Neuron index inside the neuron model.
		 * \param elapsedTimeInNeuronModelScale integration method step.
		 * \return It returns if a neuron must spike.
		 */
		bool EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
			return false;
		};

		/*!
		 * \brief It evaluates the differential equation in NeuronState and it stores the results in AuxNeuronState.
		 *
		 * It evaluates the differential equation in NeuronState and it stores the results in AuxNeuronState.
		 *
		 * \param NeuronState value of the neuron state variables where differential equations are evaluated.
		 * \param AuxNeuronState results of the differential equations evaluation.
		 * \param index Neuron index inside the VectorNeuronState
		 *
		 * \Note: this function it is not necesary for this neuron model because this one does not use integration method. 
		 */
		void EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){};

		/*!
		 * \brief It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * \param NeuronState value of the neuron state variables where time dependent equations are evaluated.
		 * \param elapsed_time integration time step.
		 * \param elapsed_time_index index inside the conductance_exp_values array.
		 */
		void EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){};



		/*!
		 * \brief It Checks if the neuron model has this connection type.
		 *
		 * It Checks if the neuron model has this connection type.
		 *
		 * \param Type input connection type.
		 *
		 * \return If the neuron model supports this connection type
		 */
		virtual bool CheckSynapseType(Interconnection * connection);


		/*!
		 * \brief It calculates the conductace exponential value for an elapsed time.
		 *
		 * It calculates the conductace exponential value for an elapsed time.
		 *
		 * \param index elapsed time index .
		 * \param elapses_time elapsed time.
		 */
		void Calculate_conductance_exp_values(int index, float elapsed_time){};

		/*!
		* \brief It calculates the number of electrical coupling synapses.
		*
		* It calculates the number for electrical coupling synapses.
		*
		* \param inter synapse that arrive to a neuron.
		*/
		void CalculateElectricalCouplingSynapseNumber(Interconnection * inter){};

		/*!
		* \brief It allocate memory for electrical coupling synapse dependencies.
		*
		* It allocate memory for electrical coupling synapse dependencies.
		*/
		void InitializeElectricalCouplingSynapseDependencies(){};

		/*!
		* \brief It calculates the dependencies for electrical coupling synapses.
		*
		* It calculates the dependencies for electrical coupling synapses.
		*
		* \param inter synapse that arrive to a neuron.
		*/
		void CalculateElectricalCouplingSynapseDependencies(Interconnection * inter){};


		/*!
		* \brief It loads the integration method from the neuron model configuration file
		*
		* It loads the integration method from the neuron model configuration file
		*
		* \param fileName neuron model configuration file name
		* \fh neuron model configuration file
		* \Currentline current line inside the file
		*/
		void loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException);

};

#endif /* SRMTIMEDRIVENMODEL_H_ */
