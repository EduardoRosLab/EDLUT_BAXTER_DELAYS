/***************************************************************************
 *                           HHTimeDrivenModel.h		                   *
 *                           -------------------                           *
 * copyright            : (C) 2015 by Francisco Naveros					   *
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

#ifndef HHTIMEDRIVENMODEL_H_
#define HHTIMEDRIVENMODEL_H_

/*!
 * \file HHTimeDrivenModel.h
 *
 * \author Francisco Naveros
 * \date May 2015
 *
 * This file declares a class which abstracts a Hodgkin and Huxley neuron model with four 
 * differential equation and two time dependent equations (conductances).
 */

#include "./TimeDrivenNeuronModel.h"

#include <string>

using namespace std;

class InputSpike;
class VectorNeuronState;
class Interconnection;



/*!
 * \class _HHTimeDrivenModel
 *
 * \brief Hodgkin and Huxley Time-Driven neuron model with a membrane potential, three ion-channel variables and
 * two conductances.
 *
 * This class abstracts the behavior of a neuron in a time-driven spiking neural network.
 * It includes internal model functions which define the behavior of the model
 * (initialization, update of the state, synapses effect, next firing prediction...).
 * This is only a virtual function (an interface) which defines the functions of the
 * inherited classes.
 *
 * \author Francisco Naveros
 * \date May 2015
 */
class HHTimeDrivenModel : public TimeDrivenNeuronModel {
	protected:

		/*!
		 * \brief Number of variables variables that must be stored in channel_values.
		*/
		static const int N_variables=6;
		
		/*!
		 * \brief Maximum membrane potential evaluated in the channel_values.
		*/
		static const float Max_V;
		
		/*!
		 * \brief Minimum membrane potential evaluated in the channel_values.
		*/
		static const float Min_V;
		
		/*!
		 * \brief Number of point in which the membrane potential is divided.
		*/
		static const int TableSize=(1024*128);

		/*!
		 * \brief Auxiliar variable.
		*/
		static const float aux;


		/*!
		* \brief vector that stores the precalculated channel values for a range of membrane potential.
		*/
		float * channel_values;
		

		/*!
		* \brief Excitatory reversal potential in mV units
		*/
		float eexc;

		/*!
		* \brief Inhibitory reversal potential in mV units
		*/
		float einh;

		/*!
		* \brief Resting potential in mV units
		*/
		float erest;

		/*!
		* \brief Resting conductance in nS units
		*/
		float grest;

		/*!
		* \brief Membrane capacitance in pF units
		*/
		float cm;
		float inv_cm;

		/*!
		* \brief Virtual firing threshold in mV units
		*/
		float vthr;

		/*!
		* \brief AMPA receptor time constant in ms units
		*/
		float texc;
		float inv_texc;

		/*!
		* \brief GABA receptor time constant in ms units
		*/
		float tinh;
		float inv_tinh;

		/*!
		* \brief Maximum value of sodium conductance in nS units
		*/
		float gNa;

		/*!
		* \brief Maximum value of potasium conductance in nS units
		*/
		float gKd;

		/*!
		* \brief Sodium potential in mV units
		*/
		float ENa;

		/*!
		* \brief Potasium potential in mV units
		*/
		float EK;

		/*!
		* \brief Efective threshold potential in mV units
		*/
		float VT;



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


	public:

		/*!
		 * \brief Number of state variables for each cell.
		*/
		static const int N_NeuronStateVariables=6;

		/*!
		 * \brief Number of state variables which are calculate with a differential equation for each cell.
		*/
		static const int N_DifferentialNeuronState=4;

		/*!
		 * \brief Number of state variables which are calculate with a time dependent equation for each cell.
		*/
		static const int N_TimeDependentNeuronState=2;


		/*!
		 * \brief Default constructor with parameters.
		 *
		 * It generates a new neuron model object without being initialized.
		 *
		 * \param NeuronTypeID Neuron model identificator.
		 * \param NeuronModelID Neuron model configuration file.
		 */
		HHTimeDrivenModel(string NeuronTypeID, string NeuronModelID);


		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		virtual ~HHTimeDrivenModel();


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
		virtual void InitializeStates(int N_neurons, int OpenMPQueueIndex);

		/*!
		 * \brief It evaluates if a neuron must spike.
		 *
		 * It evaluates if a neuron must spike.
		 *
		 * \param previous_V previous membrane potential
		 * \param NeuronState neuron state variables.
		 * \param index Neuron index inside the neuron model.
		 * \param elapsedTimeInNeuronModelScale integration method step.
		 */
		void EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale);


		/*!
		 * \brief It evaluates the differential equation in NeuronState and it stores the results in AuxNeuronState.
		 *
		 * It evaluates the differential equation in NeuronState and it stores the results in AuxNeuronState.
		 *
		 * \param NeuronState value of the neuron state variables where differential equations are evaluated.
		 * \param AuxNeuronState results of the differential equations evaluation.
		 * \param index Neuron index inside the VectorNeuronState
		 */
		void EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time);


		/*!
		 * \brief It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * \param NeuronState value of the neuron state variables where time dependent equations are evaluated.
		 * \param elapsed_time integration time step.
		 * \param elapsed_time_index index inside the conductance_exp_values array.
		 */
		void EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index);


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
		 * \brief It Calculates the channel values of different values of the membrane potential.
		 *
		 * It Calculates the channel values of different values of the membrane potential.
		 *
		 * \return The channel values stored in a vector.
		 */
		void Generate_channel_values(){
			channel_values = new float[TableSize*N_variables];

			for(int i=0; i<TableSize; i++){
				float V = Min_V + ((Max_V-Min_V)*i)/(TableSize-1);
				
				//alpha_m
				float alpha_m;
				if((13.0f-V+VT)==0.0){
					alpha_m=1.279999894092334f;
				}else{
					alpha_m=0.32f*(13.0f-V+VT)/(exp((13.0f-V+VT)*0.25f)-1.0f);
				}
				channel_values[i*N_variables] = alpha_m;
				
				//beta_m
				float beta_m;
				if((V-VT-40.0f)==0.0){
					beta_m=1.400030971093304f;
				}else{
					beta_m=0.28f*(V-VT-40.0f)/(exp((V-VT-40.0f)*0.2f)-1.0f);
				}
				channel_values[i*N_variables + 1] = beta_m;
				
				//alpha_h
				float alpha_h=0.128f*exp((17.0f-V+VT)*0.05555555f);
				channel_values[i*N_variables + 2] = alpha_h;
				
				//beta_h
				float beta_h=4.0f/(1.0f+exp((40.0f-V+VT)*0.2f));
				channel_values[i*N_variables + 3] = beta_h;

				//alpha_n
				float alpha_n;
				if((15.0f-V+VT)==0.0){
					alpha_n=0.1599999867615418f;
				}else{
					alpha_n=0.032f*(15.0f-V+VT)/(exp((15.0f-V+VT)*0.2f)-1.0f);
				}
				channel_values[i*N_variables + 4] = alpha_n;
				
				//beta_n
				float beta_n=0.5f*exp((10.0f-V+VT)*0.025f);
				channel_values[i*N_variables + 5] = beta_n;
			}
		}


		/*!
		 * \brief It Returns the channel values for a specific membrane potential.
		 *
		 * It Returns the channel values for a specific membrane potential.
		 *
		 * \param membranePotential.
		 *
		 * \return The channel values for a specific membrane potential.
		 */
		float * Get_channel_values(float membranePotential){
			int position=int((membranePotential-Min_V)*aux);
			if(position<0){
				position=0;
			}else if(position>(TableSize-1)){
				position=TableSize-1;
			}
			return (channel_values + position*N_variables);
		} 


		/*!
		 * \brief It calculates the conductace exponential value for an elapsed time.
		 *
		 * It calculates the conductace exponential value for an elapsed time.
		 *
		 * \param index elapsed time index .
		 * \param elapses_time elapsed time.
		 */
		void Calculate_conductance_exp_values(int index, float elapsed_time);

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

#endif /* HHTIMEDRIVENMODEL_H_ */
