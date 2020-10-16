/***************************************************************************
 *                           TimeDrivenPurkinjeCell_GPU2.h                 *
 *                           -------------------                           *
 * copyright            : (C) 2015 by Richard Carrill, Niceto Luque and    *
						  Francisco Naveros								   *
 * email                : rcarrillo@ugr.es, nluque@ugr.es and			   *
						  fnaveros@ugr.es    							   *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef TIMEDRIVENPURKINJECELL_GPU2_H_
#define TIMEDRIVENPURKINJECELL_GPU2_H_

/*!
 * \file TimeDrivenPurkinjeCell_GPU.h
 *
 * \author Richard Carrillo
 * \author Niceto Luque
 * \author Francisco Naveros
 * \date May 2015
 *
 * This file declares a class which implement a Purkinje cell model. This model is
 * implemented in GPU.
 */


#include "./TimeDrivenNeuronModel_GPU2.h"
#include "../../include/integration_method/IntegrationMethod_GPU2.h"


//Library for CUDA
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

/*!
 * \class TimeDrivenPurkinjeCell_GPU2.h
 *
 * \brief Time driven neuron model with a membrane potential, two current channels and two conductances.
 *
 * \author Richard Carrillo
 * \author Niceto Luque
 * \author Francisco Naveros
 * \date May 2015
 */

class TimeDrivenPurkinjeCell_GPU2 : public TimeDrivenNeuronModel_GPU2 {
	public:

		/*!
		* \brief leak current in mS/cm^2 units
		*/
		const float g_L;

		/*!
		* \brief high-threshold noninactivating calcium conducance in mS/cm^2 units
		*/
		const float g_Ca;

		/*!
		* \brief muscarinic receptor suppressed potassium conductance (or M conductance) in mS/cm^2 units
		*/
		const float g_M;

		/*!
		* \brief Cylinder length of the soma in cm units
		*/
		const float Cylinder_length_of_the_soma;


		/*!
		* \brief Radius of the soma in cm units
		*/
		const float Radius_of_the_soma;

		/*!
		* \brief Cell area in cm^2 units
		*/
		const float Area;
		const float inv_Area;

		/*!
		* \brief Membrane capacitance in uF/cm^2 units
		*/
		const float Membrane_capacitance;
		const float inv_Membrane_capacitance;

		/*!
		* \brief Excitatory reversal potential in mV units
		*/
		const float eexc;

		/*!
		* \brief Inhibitory reversal potential in mV units
		*/
		const float einh;

		/*!
		* \brief Firing threshold in mV units
		*/
		const float vthr;

		/*!
		* \brief Resting potential in mV units
		*/
		const float erest;

		/*!
		* \brief AMPA receptor time constant in ms units
		*/
		const float texc;
		const float inv_texc;

		/*!
		* \brief GABA receptor time constant in ms units
		*/
		const float tinh;
		const float inv_tinh;


		/*!
		* \brief Refractory period in ms units
		*/
		const float tref;
		const float tref_0_5;
		const float inv_tref_0_5;

		/*!
		* \brief Peak amplitude in mV units
		*/
		const float spkpeak;

		/*!
		 * \brief Number of state variables for each cell.
		*/
		static const int N_NeuronStateVariables=5;

		/*!
		 * \brief Number of state variables which are calculate with a differential equation for each cell.
		*/
		static const int N_DifferentialNeuronState=3;

		/*!
		 * \brief Number of state variables which are calculate with a time dependent equation for each cell.
		*/
		static const int N_TimeDependentNeuronState=2;

		
		/*!
		 * \brief constructor with parameters.
		 *
		 * It generates a new neuron model object.
		 *
		 * \param g_L;
		 * \param g_Ca;
		 * \param g_M;
		 * \param Cylinder_length_of_the_soma;
		 * \param Radius_of_the_soma;
		 * \param Area;
		 * \param inv_Area;
		 * \param Membrane_capacitance;
		 * \param inv_Membrane_capacitance;
		 * \param eexc;
		 * \param einh;
		 * \param vthr;
		 * \param erest;
		 * \param texc;
		 * \param inv_texc;
		 * \param tinh;
		 * \param inv_tinh;
		 * \param tref;
		 * \param tref_0_5;
		 * \param inv_tref_0_5;
		 * \param spkpeak;
		 * \param integrationName integration method type.
		 * \param N_neurons number of neurons.
		 * \param Total_N_thread total number of CUDA thread.
		 * \param Buffer_GPU Gpu auxiliar memory.
		 *
		 */
		__device__ TimeDrivenPurkinjeCell_GPU2(float new_g_L, float new_g_Ca, float new_g_M, float new_Cylinder_length_of_the_soma,
		    float new_Radius_of_the_soma, float new_Area,float new_Membrane_capacitance, float new_eexc, float new_einh,
			float new_vthr, float new_erest, float new_texc, float new_tinh, float new_tref, float new_spkpeak, 
			char const* integrationName,	int N_neurons, void ** Buffer_GPU):TimeDrivenNeuronModel_GPU2(MilisecondScale_GPU),
			g_L(new_g_L),g_Ca(new_g_Ca), g_M(new_g_M), Cylinder_length_of_the_soma(new_Cylinder_length_of_the_soma), 
			Radius_of_the_soma(new_Radius_of_the_soma),	Area(new_Area), inv_Area(1.0f/new_Area), 
			Membrane_capacitance(new_Membrane_capacitance), inv_Membrane_capacitance(1.0f/new_Membrane_capacitance),
			eexc(new_eexc), einh(new_einh), vthr(new_vthr), erest(new_erest), texc(new_texc), inv_texc(1.0f/new_texc), 
			tinh(new_tinh),	inv_tinh(1.0f/new_tinh), tref(new_tref), tref_0_5(new_tref*0.5f), inv_tref_0_5(2.0f/new_tref), 
			spkpeak(new_spkpeak)
		{
			loadIntegrationMethod_GPU2(integrationName, Buffer_GPU);

			integrationMethod_GPU2->Calculate_conductance_exp_values();	
		}

		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		__device__ virtual ~TimeDrivenPurkinjeCell_GPU2(){
			delete integrationMethod_GPU2;
		}


		/*!
		 * \brief Update the neuron state variables.
		 *
		 * It updates the neuron state variables.
		 *
		 * \param index The cell index inside the StateGPU. 
		 * \param AuxStateGPU Auxiliary incremental conductance vector.
		 * \param StateGPU Neural state variables.
		 * \param LastUpdateGPU Last update time
		 * \param LastSpikeTimeGPU Last spike time
		 * \param InternalSpikeGPU In this vector is stored if a neuron must generate an output spike.
		 * \param SizeStates Number of neurons
		 * \param CurrentTime Current time.
		 *
		 * \return True if an output spike have been fired. False in other case.
		 */
		//__device__ void UpdateState(double CurrentTime)
		//{
		//	int index = blockIdx.x * blockDim.x + threadIdx.x;
		//	while (index<vectorNeuronState_GPU2->SizeStates){
		//		vectorNeuronState_GPU2->VectorNeuronStates_GPU[N_DifferentialNeuronState*vectorNeuronState_GPU2->SizeStates + index]+=vectorNeuronState_GPU2->AuxStateGPU[index];
		//		vectorNeuronState_GPU2->VectorNeuronStates_GPU[(N_DifferentialNeuronState+1)*vectorNeuronState_GPU2->SizeStates + index]+=vectorNeuronState_GPU2->AuxStateGPU[vectorNeuronState_GPU2->SizeStates + index];

		//		this->integrationMethod_GPU2->NextDifferentialEquationValues(index, vectorNeuronState_GPU2->SizeStates, vectorNeuronState_GPU2->VectorNeuronStates_GPU);

		//		this->CheckValidIntegration(index);

		//		index+=blockDim.x*gridDim.x;
		//	}
		//} 

		__device__ void UpdateState(double CurrentTime)
		{
			int index = blockIdx.x * blockDim.x + threadIdx.x;
			while (index<vectorNeuronState_GPU2->SizeStates){
				vectorNeuronState_GPU2->VectorNeuronStates_GPU[N_DifferentialNeuronState*vectorNeuronState_GPU2->SizeStates + index] += vectorNeuronState_GPU2->AuxStateGPU[index];
				vectorNeuronState_GPU2->VectorNeuronStates_GPU[(N_DifferentialNeuronState + 1)*vectorNeuronState_GPU2->SizeStates + index] += vectorNeuronState_GPU2->AuxStateGPU[vectorNeuronState_GPU2->SizeStates + index];

				index += blockDim.x*gridDim.x;
			}

			this->integrationMethod_GPU2->NextDifferentialEquationValues(vectorNeuronState_GPU2->SizeStates, vectorNeuronState_GPU2->VectorNeuronStates_GPU);
		}


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
		__device__ bool EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
			if (previous_V<vthr && NeuronState[index] >= vthr){
				vectorNeuronState_GPU2->LastSpikeTimeGPU[index]=0.0;
				vectorNeuronState_GPU2->InternalSpikeGPU[index] = true;
			}

			float last_spike=timeScale*vectorNeuronState_GPU2->LastSpikeTimeGPU[index];
			if(last_spike < tref){
				if(last_spike <= tref_0_5){
					vectorNeuronState_GPU2->VectorNeuronStates_GPU[index]=vthr+(spkpeak-vthr)*(last_spike*inv_tref_0_5);
				}else{
					vectorNeuronState_GPU2->VectorNeuronStates_GPU[index]=spkpeak-(spkpeak-erest)*((last_spike-tref_0_5)*inv_tref_0_5);
				}
			}else if((last_spike - tref)<elapsedTimeInNeuronModelScale){
				vectorNeuronState_GPU2->VectorNeuronStates_GPU[index]=erest;
			}
		}

		/*!
		 * \brief It evaluates the differential equation in NeuronState and it stores the results in AuxNeuronState.
		 *
		 * It evaluates the differential equation in NeuronState and it stores the results in AuxNeuronState.
		 *
		 * \param index index inside the NeuronState vector.
		 * \param SizeStates number of element in NeuronState vector.
		 * \param NeuronState value of the neuron state variables where differential equations are evaluated.
		 * \param AuxNeuronState results of the differential equations evaluation.
		 */
		__device__ void EvaluateDifferentialEquation(int index, int SizeStates, float * NeuronState, float * AuxNeuronState, float elapsed_time){
			float V=NeuronState[SizeStates + index];
			float ca=NeuronState[1*SizeStates + index];
			float M=NeuronState[2*SizeStates + index];
			float g_exc=NeuronState[3*SizeStates + index];
			float g_inh=NeuronState[4*SizeStates + index];
			float last_spike=timeScale*vectorNeuronState_GPU2->LastSpikeTimeGPU[index];

			int offset1=gridDim.x * blockDim.x;
			int offset2=blockDim.x*blockIdx.x + threadIdx.x;

		
			//V
			if(last_spike >= tref){
				AuxNeuronState[0*offset1 + offset2]=(-g_L*(V+70.0f)-g_Ca*ca*ca*(V-125.0f)-g_M*M*(V+95.0f) + (g_exc * (this->eexc - V) + g_inh * (this->einh - V))*inv_Area )*inv_Membrane_capacitance;
			}else if(last_spike <= tref_0_5){
				AuxNeuronState[0*offset1 + offset2]=(spkpeak-vthr)*inv_tref_0_5;
			}else{
				AuxNeuronState[0*offset1 + offset2]=(erest-spkpeak)*inv_tref_0_5;
			}


			//ca
			float alpha_ca=1.6f/(1+__expf(-0.072f*(V-5.0f)));
			float beta_ca=(0.02f*(V+8.9f))/(__expf((V+8.9f)*0.2f)-1.0f);
			float inv_tau_ca=alpha_ca+beta_ca;

			AuxNeuronState[1*offset1 + offset2]=alpha_ca - ca*inv_tau_ca;


			//M
			float alpha_M=0.3f/(1+__expf((-V-2.0f)*0.2f));
			float beta_M=0.001f*__expf((-V-60.0f)*0.055555555555555f);
			float inv_tau_M=alpha_M+beta_M;

			AuxNeuronState[2*offset1 + offset2]=alpha_M - M*inv_tau_M;
		}








		/*!
		 * \brief It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * It evaluates the time depedendent Equation in NeuronState for elapsed_time and it stores the results in NeuronState.
		 *
		 * \param index index inside the NeuronState vector.
		 * \param SizeStates number of element in NeuronState vector.
		 * \param NeuronState value of the neuron state variables where time dependent equations are evaluated.
		 * \param elapsed_time integration time step.
		 * \param elapsed_time_index index inside the conductance_exp_values array.
		 */
		__device__ void EvaluateTimeDependentEquation(int index, int SizeStates, float * NeuronState, float elapsed_time, int elapsed_time_index){
			float limit=1e-15;
			
			float * Conductance_values=this->Get_conductance_exponential_values(elapsed_time_index);

			if(NeuronState[this->N_DifferentialNeuronState*SizeStates + index]<limit){
				NeuronState[this->N_DifferentialNeuronState*SizeStates + index]=0.0f;
			}else{
				NeuronState[this->N_DifferentialNeuronState*SizeStates + index]*=  Conductance_values[0];
			}
			if(NeuronState[(this->N_DifferentialNeuronState+1)*SizeStates + index]<limit){
				NeuronState[(this->N_DifferentialNeuronState+1)*SizeStates + index]=0.0f;
			}else{
				NeuronState[(this->N_DifferentialNeuronState+1)*SizeStates + index]*=  Conductance_values[1];
			}
		}

		/*!
		 * \brief It calculates the conductace exponential value for an elapsed time.
		 *
		 * It calculates the conductace exponential value for an elapsed time.
		 *
		 * \param index elapsed time index .
		 * \param elapses_time elapsed time.
		 */
		__device__ void Calculate_conductance_exp_values(int index, float elapsed_time){
			//excitatory synapse.
			Set_conductance_exp_values(index, 0, __expf(-elapsed_time*this->inv_texc));
			//inhibitory synapse.
			Set_conductance_exp_values(index, 1, __expf(-elapsed_time*this->inv_tinh));
		}



		/*!
		* \brief It loads the integration method from the neuron model configuration file
		*
		* It loads the integration method from the neuron model configuration file
		*
		* \param integrationName integration method name
		* \param Buffer_GPU integration method parameters
		*/
		__device__ void loadIntegrationMethod_GPU2(char const* integrationName, void ** Buffer_GPU){

			//DEFINE HERE NEW INTEGRATION METHOD
			if (cmp4(integrationName, "Euler", 5) == 0){
				this->integrationMethod_GPU2 = (Euler_GPU2<TimeDrivenPurkinjeCell_GPU2> *) new Euler_GPU2<TimeDrivenPurkinjeCell_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "RK2", 3) == 0){
				this->integrationMethod_GPU2 = (RK2_GPU2<TimeDrivenPurkinjeCell_GPU2> *) new RK2_GPU2<TimeDrivenPurkinjeCell_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "RK4", 3) == 0){
				this->integrationMethod_GPU2 = (RK4_GPU2<TimeDrivenPurkinjeCell_GPU2> *) new RK4_GPU2<TimeDrivenPurkinjeCell_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "BDF", 3) == 0 && atoiGPU(integrationName, 3)>0 && atoiGPU(integrationName, 3)<7){
				this->integrationMethod_GPU2 = (BDFn_GPU2<TimeDrivenPurkinjeCell_GPU2> *) new BDFn_GPU2<TimeDrivenPurkinjeCell_GPU2>(this, Buffer_GPU, atoiGPU(integrationName, 3));
			}
			else if (cmp4(integrationName, "Bifixed_Euler", 13) == 0){
				this->integrationMethod_GPU2 = (Bifixed_Euler_GPU2<TimeDrivenPurkinjeCell_GPU2> *) new Bifixed_Euler_GPU2<TimeDrivenPurkinjeCell_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "Bifixed_RK2", 11) == 0){
				this->integrationMethod_GPU2 = (Bifixed_RK2_GPU2<TimeDrivenPurkinjeCell_GPU2> *) new Bifixed_RK2_GPU2<TimeDrivenPurkinjeCell_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "Bifixed_RK4", 11) == 0){
				this->integrationMethod_GPU2 = (Bifixed_RK4_GPU2<TimeDrivenPurkinjeCell_GPU2> *) new Bifixed_RK4_GPU2<TimeDrivenPurkinjeCell_GPU2>(this, Buffer_GPU);
			}
			else{
				printf("There was an error loading the integration methods of the GPU.\n");

			}
		}


};


#endif /* TIMEDRIVENPURKINJECELL_GPU2_H_ */
