/***************************************************************************
 *                           EdidioGranuleCell_TimeDriven.h                *
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

#ifndef EGIDIOGRANULECELL_TIMEDRIVEN_H_
#define EGIDIOGRANULECELL_TIMEDRIVEN_H_


/*!
 * \file EdidioGranuleCell_TimeDriven.h
 *
 * \author Francisco Naveros
 * \date May 2013
 *
 * This file declares a class which abstracts a Leaky Integrate-And-Fire neuron model for a cerebellar 
 * granule cell. This neuron model has 15 differential equations and 2 time dependent equations (conductances).
 */

#include "./TimeDrivenNeuronModel.h"
#include <cmath>
#include <string>

using namespace std;

class InputSpike;
class VectorNeuronState;
class Interconnection;

 

/*!
 * \class EdidioGranuleCell_TimeDriven
 *
 * \brief Leaky Integrate-And-Fire Time-Driven neuron model for a cerebellar granule cell.
 *
 * This class abstracts the behavior of a neuron in a time-driven spiking neural network.
 * It includes internal model functions which define the behavior of the model
 * (initialization, update of the state, synapses effect, next firing prediction...).
 * This is only a virtual function (an interface) which defines the functions of the
 * inherited classes.
 *
 * \author Francisco Naveros
 * \date May 2013
 */
class EgidioGranuleCell_TimeDriven : public TimeDrivenNeuronModel {
	protected:


		static const float gMAXNa_f; // S/cm^2
		static const float gMAXNa_r; // S/cm^2
		static const float gMAXNa_p; // S/cm^2
		static const float gMAXK_V;  // S/cm^2
		static const float gMAXK_A;  // S/cm^2
		static const float gMAXK_IR; // S/cm^2
		static const float gMAXK_Ca; // S/cm^2
		static const float gMAXCa;   // S/cm^2
		static const float gMAXK_sl; // S/cm^2

		static const float gLkg1;
		static const float gLkg2;
		static const float VNa;
		static const float VK;
		static const float VLkg1;
		static const float VLkg2;
		static const float V0_xK_Ai;
		static const float K_xK_Ai;
		static const float V0_yK_Ai;
		static const float K_yK_Ai;
		static const float V0_xK_sli;
		static const float B_xK_sli;
		static const float F;
		static const float A;
		static const float d;
		static const float betaCa;
		static const float Ca0;
		static const float R;
		static const float cao;
		//Membrane Capacitance = 1uF/cm^2 = 10^-3mF/cm^2;
		static const float Cm;

		static const float temper;

		// gating-kinetic correction for experiment temperature=20ºC;
		static const float Q10_20;
		// gating-kinetic correction for experiment temperature=22ºC;
		static const float Q10_22;
		// experiment temperature=30ºC;
		static const float Q10_30;
		// experiment temperature=6.3ºC;
		static const float Q10_6_3;

		const float I_inj_abs;

		// Injected Current in absolute terms = -10pA;
		// Cell membrane area = pi*2*r*L = pi*9.76*9.76 = 299.26058um^2 = 299.26058*10^-8cm^2;
		// Injected current per area unit = -I_inj_abs/ 299.26058*10^-8cm^2 = I_inj;
		const float I_inj;

		const float eexc;
		const float einh;
		const float texc;
		const float tinh;
		const float vthr;


		/*!
		 * \brief vector that stores the precalculated channel values for a range of membrane potential.
		*/
		static float * channel_values;

		/*!
		 * \brief Number of variables variables that must be stored in channel_values.
		*/
		static const int N_variables=26;
		
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
		static const int TableSize=1024*16;

		/*!
		 * \brief Auxiliar variable.
		*/
		static const float aux;
		

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
		 * \brief 
		 *
		 * 
		 *
		 * \param ci.
		 * \param co.
		 * \param z.
		 * \param temper temperature.
		 */
		static float nernst(float ci, float co, float z, float temper){
			return (1000*(R*(temper + 273.15f)/F)/z*log(abs(co/ci)));
		}


		/*!
		 * \brief 
		 *
		 * 
		 *
		 * \param x.
		 * \param y.
		 */
		static float linoid(float x, float y){
			float f=0.0;
			if (abs(x/y)<1e-06f){
				f=y*(1-x/y/2);
			}else{
				f=x/(exp(x/y)-1);
			}
			return f;
		};


	public:

		/*!
		 * \brief Number of state variables for each cell.
		*/
		static const int N_NeuronStateVariables=17;

		/*!
		 * \brief Number of state variables which are calculate with a differential equation for each cell.
		*/
		static const int N_DifferentialNeuronState=15;

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
		EgidioGranuleCell_TimeDriven(string NeuronTypeID, string NeuronModelID);


		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		virtual ~EgidioGranuleCell_TimeDriven();


		/*!
		 * \brief It loads the neuron model description and tables (if necessary).
		 *
		 * It loads the neuron model description and tables (if necessary).
		 *
		 * \throw EDLUTFileException If something wrong has happened in the file load.
		 */
		virtual void LoadNeuronModel() throw (EDLUTFileException);


		/*!
		 * \brief It initializes the neuron state to defined values.
		 *
		 * It initializes the neuron state to defined values.
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
		static float * Generate_channel_values(){
			float * NewLookUpTable=new float[TableSize*N_variables]();
			for(int i=0; i<TableSize; i++){
				float V = Min_V + ((Max_V-Min_V)*i)/(TableSize-1);
				
				//////////////////////xNa_f//////////////////////////
				//alpha_xNa_f
				float alpha_xNa_f=Q10_20*(-0.3f)*linoid(V+19.0f, -10.0f);
				NewLookUpTable[i*N_variables]=alpha_xNa_f;
				
				//inv_tau_xNa_f
				float beta_xNa_f=Q10_20*12.0f*exp(-(V+44.0f)/18.182f);
				float inv_tau_xNa_f=alpha_xNa_f+beta_xNa_f;
				NewLookUpTable[i*N_variables+1]=inv_tau_xNa_f;


				//////////////////////yNa_f//////////////////////////
				//alpha_yNa_f
				float alpha_yNa_f=Q10_20*0.105f*exp(-(V+44.0f)/3.333f);
				NewLookUpTable[i*N_variables+2]=alpha_yNa_f;
				
				//inv_tau_yNa_f
				float beta_yNa_f=Q10_20*1.5f/(1.0f+exp(-(V+11.0f)/5.0f));
				float inv_tau_yNa_f=alpha_yNa_f+beta_yNa_f;
				NewLookUpTable[i*N_variables+3]=inv_tau_yNa_f;


				//////////////////////xNa_r//////////////////////////
				//alpha_xNa_r
				float alpha_xNa_r=Q10_20*(0.00008f-0.00493f*linoid(V-4.48754f,-6.81881f));
				NewLookUpTable[i*N_variables+4]=alpha_xNa_r;
				
				//inv_tau_xNa_r
				float beta_xNa_r=Q10_20*(0.04752f+0.01558f*linoid(V+43.97494f,0.10818f));
				float inv_tau_xNa_r=alpha_xNa_r+beta_xNa_r;
				NewLookUpTable[i*N_variables+5]=inv_tau_xNa_r;


				//////////////////////yNa_r//////////////////////////
				//alpha_yNa_r
				float alpha_yNa_r=Q10_20*0.31836f*exp(-(V+80.0f)/62.52621f);
				NewLookUpTable[i*N_variables+6]=alpha_yNa_r;
				
				//inv_tau_yNa_r
				float beta_yNa_r=Q10_20*0.01014f*exp((V+83.3332f)/16.05379f);
				float inv_tau_yNa_r=alpha_yNa_r+beta_yNa_r;
				NewLookUpTable[i*N_variables+7]=inv_tau_yNa_r;


				//////////////////////xNa_p//////////////////////////
				//xNa_p_inf
				float xNa_p_inf = 1.0f/(1.0f+exp(-(V+42.0f)/5.0f));	
				NewLookUpTable[i*N_variables+8]=xNa_p_inf;

				//inv_tau_xNa_p
				float alpha_xNa_p = Q10_30*(-0.091f)*linoid(V+42.0f,-5.0f);										
				float beta_xNa_p = Q10_30*0.062f*linoid(V+42.0f,5.0f);											
				float inv_tau_xNa_p = (alpha_xNa_p + beta_xNa_p)*0.2f;
				NewLookUpTable[i*N_variables+9]=inv_tau_xNa_p;


				//////////////////////xK_V//////////////////////////
				//alpha_xK_V
				float alpha_xK_V = Q10_6_3*(-0.01f)*linoid(V+25.0f,-10.0f);	
				NewLookUpTable[i*N_variables+10]=alpha_xK_V;
				
				//inv_tau_xK_V
				float beta_xK_V   = Q10_6_3*0.125f*exp(-0.0125f*(V+35.0f));			
				float inv_tau_xK_V     = (alpha_xK_V + beta_xK_V);
				NewLookUpTable[i*N_variables+11]=inv_tau_xK_V;


				//////////////////////xK_A//////////////////////////
				//xK_A_inf
				float xK_A_inf = 1.0f/(1.0f+exp((V-V0_xK_Ai)/K_xK_Ai));
				NewLookUpTable[i*N_variables+12]=xK_A_inf;
				
				//inv_tau_xK_A
				float alpha_xK_A = (Q10_20*4.88826f)/(1+exp(-(V+9.17203f)/23.32708f));	//
				float beta_xK_A = (Q10_20*0.99285f)/exp((V+18.27914f)/19.47175f);		//
				float inv_tau_xK_A = (alpha_xK_A + beta_xK_A);
				NewLookUpTable[i*N_variables+13]=inv_tau_xK_A;


				//////////////////////yK_A//////////////////////////
				//yK_A_inf
				float yK_A_inf    = 1.0f/(1.0f+exp((V-V0_yK_Ai)/K_yK_Ai));					
				NewLookUpTable[i*N_variables+14]=yK_A_inf;

				//inv_tau_yK_A
				float alpha_yK_A = (Q10_20*0.11042f)/(1.0f+exp((V+111.33209f)/12.8433f));	
				float beta_yK_A   = (Q10_20*0.10353f)/(1.0f+exp(-(V+49.9537f)/8.90123f));	
				float inv_tau_yK_A     = (alpha_yK_A + beta_yK_A);
				NewLookUpTable[i*N_variables+15]=inv_tau_yK_A;

				
				//////////////////////xK_IR//////////////////////////
				//alpha_xK_IR
				float alpha_xK_IR = Q10_20*0.13289f*exp(-(V+83.94f)/24.3902f);
				NewLookUpTable[i*N_variables+16]=alpha_xK_IR;

				//inv_tau_xK_IR
				float beta_xK_IR = Q10_20*0.16994f*exp((V+83.94f)/35.714f);			
				float inv_tau_xK_IR = (alpha_xK_IR + beta_xK_IR);
				NewLookUpTable[i*N_variables+17]=inv_tau_xK_IR;

				//////////////////////xK_Ca//////////////////////////
				float aux_xK_Ca=0.0015f*exp(-V/11.765f);
				float inv_aux_xK_Ca=1.0f/(0.00015f*exp(-V/11.765f));
				NewLookUpTable[i*N_variables+18]=aux_xK_Ca;
				NewLookUpTable[i*N_variables+19]=inv_aux_xK_Ca;
				
				//////////////////////xCa//////////////////////////
				//alpha_xCa
				float alpha_xCa  = Q10_20*0.04944f*exp((V+29.06f)/15.87301587302f);	
				NewLookUpTable[i*N_variables+20]=alpha_xCa;

				//inv_tau_xCa
				float beta_xCa   = Q10_20*0.08298f*exp(-(V+18.66f)/25.641f);		
				float inv_tau_xCa     = (alpha_xCa + beta_xCa);
				NewLookUpTable[i*N_variables+21]=inv_tau_xCa;


				//////////////////////yCa//////////////////////////
				//alpha_yCa
				float alpha_yCa = Q10_20*0.0013f*exp(-(V+48.0f)/18.183f);
				NewLookUpTable[i*N_variables+22]=alpha_yCa;

				//inv_tau_yCa
				float beta_yCa   = Q10_20*0.0013f*exp((V+48.0f)/83.33f);	
				float inv_tau_yCa     = (alpha_yCa + beta_yCa);
				NewLookUpTable[i*N_variables+23]=inv_tau_yCa;

				
				//////////////////////xK_sl//////////////////////////
				//xK_sl_inf
				float xK_sl_inf    = 1.0f/(1.0f+exp(-(V-V0_xK_sli)/B_xK_sli));
				NewLookUpTable[i*N_variables+24]=xK_sl_inf;

				//inv_tau_xK_sl
				float alpha_xK_sl = Q10_22*0.0033f*exp((V+30.0f)/40.0f);			
				float beta_xK_sl = Q10_22*0.0033f*(-(V+30.0f)/20.0f);		
				float inv_tau_xK_sl = (alpha_xK_sl + beta_xK_sl);
				NewLookUpTable[i*N_variables+25]=inv_tau_xK_sl;
				
			}
			return NewLookUpTable;
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
		static float * Get_channel_values(float membranePotential){
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

#endif /* EGIDIOGRANULECELL_TIMEDRIVEN_H_ */
