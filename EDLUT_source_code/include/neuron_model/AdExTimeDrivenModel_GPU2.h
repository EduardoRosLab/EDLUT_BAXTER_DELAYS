/***************************************************************************
 *                          IzhikevichTimeDrivenModel_GPU2.h                 *
 *                           -------------------                           *
 * copyright            : (C) 2012 by Francisco Naveros                    *
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

#ifndef ADEXTIMEDRIVENMODEL_GPU2_H_
#define ADEXTIMEDRIVENMODEL_GPU2_H_

/*!
 * \file AdEximeDrivenModel_GPU2.h
 *
 * \author Francisco Naveros
 * \date December 2015
 *
 * This file declares a class which abstracts a Adaptative Exponential Integrate and Fire (AdEx) neuron model with one 
 * differential equation and two time dependent equations (conductances). This model is
 * implemented in GPU.
 */

#include "./TimeDrivenNeuronModel_GPU2.h"
#include "../../include/integration_method/IntegrationMethod_GPU2.h"


//Library for CUDA
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

/*!
 * \class AdExTimeDrivenModel_1_2_GPU
 *
 *
 * \brief Adaptative Exponential Integrate and Fire (AdEx) Time-Driven neuron model with a membrane potential and
 * two conductances. This model is implemented in GPU.
 *
 * This class abstracts the behavior of a neuron in a time-driven spiking neural network.
 * It includes internal model functions which define the behavior of the model
 * (initialization, update of the state, synapses effect, next firing prediction...).
 * This is only a virtual function (an interface) which defines the functions of the
 * inherited classes.
 *
 * \author Francisco Naveros
 * \date December 2015
 */

class AdExTimeDrivenModel_GPU2 : public TimeDrivenNeuronModel_GPU2 {
	public:

		/*!
		* \brief Conductance in nS units
		*/
		const float a;

		/*!
		* \brief Spike trigger adaptation in pA units
		*/
		const float b;

		/*!
		* \brief Threshold slope factor in mV units
		*/
		const float TSF;
		const float inv_TSF;

		/*!
		* \brief Effective threshold potential in mV units
		*/
		const float VT;

		/*!
		* \brief Time constant in ms units
		*/
		const float tauw;
		const float inv_tauw;

		/*!
		* \brief Excitatory reversal potential in mV units
		*/
		const float eexc;

		/*!
		* \brief Inhibitory reversal potential in mV units
		*/
		const float einh;

		/*!
		* \brief Resting potential in mV units
		*/
		const float Ereset;

		/*!
		* \brief Effective rest potential in mV units
		*/
		const float Eleak;

		/*!
		* \brief Total leak conductance in nS units
		*/
		const float gleak;

		/*!
		* \brief Membrane capacitance in pF units
		*/
		const float cm;
		const float inv_cm;

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
		 * \brief Number of state variables for each cell.
		*/
		static const int N_NeuronStateVariables=4;

		/*!
		 * \brief Number of state variables which are calculate with a differential equation for each cell.
		*/
		static const int N_DifferentialNeuronState=2;

		/*!
		 * \brief Number of state variables which are calculate with a time dependent equation for each cell.
		*/
		static const int N_TimeDependentNeuronState=2;

		
		/*!
		 * \brief constructor with parameters.
		 *
		 * It generates a new neuron model object.
		 *
		 * \param Eexc eexc.
		 * \param Einh einh.
		 * \param Erest erest.
		 * \param Vthr vthr.
		 * \param Cm cm.
		 * \param Texc texc.
		 * \param Tinh tinh.
		 * \param Tref tref.
		 * \param Grest grest.
		 * \param integrationName integration method type.
		 * \param N_neurons number of neurons.
		 * \param Total_N_thread total number of CUDA thread.
		 * \param Buffer_GPU Gpu auxiliar memory.
		 *
		 */
		__device__ AdExTimeDrivenModel_GPU2(float new_a, float new_b, float new_TSF, float new_VT, float new_tauw, float new_eexc, float new_einh, float new_Ereset, float new_Eleak,
				float new_gleak, float new_cm, float new_texc, float new_tinh, char const* integrationName, int N_neurons, void ** Buffer_GPU):
					TimeDrivenNeuronModel_GPU2(MilisecondScale_GPU), a(new_a), b(new_b), TSF(new_TSF), VT(new_VT), tauw(new_tauw), eexc(new_eexc), einh(new_einh),
					Ereset(new_Ereset), Eleak(new_Eleak), gleak(new_gleak), cm(new_cm), texc(new_texc), tinh(new_tinh), inv_TSF(1.0f/new_TSF), inv_tauw(1.0f/new_tauw), inv_cm(1.0f/new_cm), 
					inv_texc(1.0f/new_texc), inv_tinh(1.0f/new_tinh) {
			loadIntegrationMethod_GPU2(integrationName, Buffer_GPU);
		
			integrationMethod_GPU2->Calculate_conductance_exp_values();			
		}


		/*!
		 * \brief Class destructor.
		 *
		 * It destroys an object of this class.
		 */
		__device__ virtual ~AdExTimeDrivenModel_GPU2(){
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
		//		
		//		vectorNeuronState_GPU2->VectorNeuronStates_GPU[(this->N_DifferentialNeuronState)*vectorNeuronState_GPU2->SizeStates + index]+=vectorNeuronState_GPU2->AuxStateGPU[index];
		//		vectorNeuronState_GPU2->VectorNeuronStates_GPU[(this->N_DifferentialNeuronState+1)*vectorNeuronState_GPU2->SizeStates + index]+=vectorNeuronState_GPU2->AuxStateGPU[vectorNeuronState_GPU2->SizeStates + index];

		//		this->integrationMethod_GPU2->NextDifferentialEquationValues(index, vectorNeuronState_GPU2->SizeStates, vectorNeuronState_GPU2->VectorNeuronStates_GPU);

		//		this->CheckValidIntegration(index);

		//		index+=blockDim.x*gridDim.x;
		//	}
		//} 

		__device__ void UpdateState(double CurrentTime)
		{
			int index = blockIdx.x * blockDim.x + threadIdx.x;
			while (index < vectorNeuronState_GPU2->SizeStates){
				vectorNeuronState_GPU2->VectorNeuronStates_GPU[(this->N_DifferentialNeuronState)*vectorNeuronState_GPU2->SizeStates + index] += vectorNeuronState_GPU2->AuxStateGPU[index];
				vectorNeuronState_GPU2->VectorNeuronStates_GPU[(this->N_DifferentialNeuronState + 1)*vectorNeuronState_GPU2->SizeStates + index] += vectorNeuronState_GPU2->AuxStateGPU[vectorNeuronState_GPU2->SizeStates + index];

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
		__device__ void EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
			if (NeuronState[index] > 0.0f){
				//V
				NeuronState[index] = this->Ereset;
				//w
				NeuronState[vectorNeuronState_GPU2->SizeStates + index] += this->b;
				vectorNeuronState_GPU2->LastSpikeTimeGPU[index]=0.0;
				this->integrationMethod_GPU2->resetState(index);
				vectorNeuronState_GPU2->InternalSpikeGPU[index] = true;
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
			float V=NeuronState[index];
			float w=NeuronState[SizeStates + index];
			float gexc=NeuronState[2*SizeStates + index];
			float ginh=NeuronState[3*SizeStates + index];

			
			if(V<=-20.0f){
				//V
				AuxNeuronState[blockDim.x*blockIdx.x + threadIdx.x]=( - gleak*(V-Eleak) + gleak*TSF*__expf((V-VT)*inv_TSF) - w - gexc * (V - this->eexc) - ginh * (V - this->einh))*this->inv_cm;
				//w
				AuxNeuronState[blockDim.x*gridDim.x + blockDim.x*blockIdx.x + threadIdx.x]=(a*(V - Eleak) - w)*this->inv_tauw;
			}else if(V<=0.0f){
				float Vm=-20.0f;
				//V
				AuxNeuronState[blockDim.x*blockIdx.x + threadIdx.x]=( - gleak*(Vm-Eleak) + gleak*TSF*__expf((Vm-VT)*inv_TSF) - w - gexc * (Vm - this->eexc) - ginh * (Vm - this->einh))*this->inv_cm;
				//w
				AuxNeuronState[blockDim.x*gridDim.x + blockDim.x*blockIdx.x + threadIdx.x]=(a*(V - Eleak) - w)*this->inv_tauw;
			}else{
				//V
				AuxNeuronState[blockDim.x*blockIdx.x + threadIdx.x]=0;
				//w
				AuxNeuronState[blockDim.x*gridDim.x + blockDim.x*blockIdx.x + threadIdx.x]=0;
			}
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
			float limit=1e-9;

			float * Conductance_values=this->Get_conductance_exponential_values(elapsed_time_index);
			
			if(NeuronState[this->N_DifferentialNeuronState*SizeStates + index]<limit){
				NeuronState[this->N_DifferentialNeuronState*SizeStates + index]=0.0f;
			}else{
				NeuronState[this->N_DifferentialNeuronState*SizeStates + index]*= Conductance_values[0];
			}
			if(NeuronState[(this->N_DifferentialNeuronState+1)*SizeStates + index]<limit){
				NeuronState[(this->N_DifferentialNeuronState+1)*SizeStates + index]=0.0f;
			}else{
				NeuronState[(this->N_DifferentialNeuronState+1)*SizeStates + index]*= Conductance_values[1];
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
			IntegrationMethod_GPU2 * integ;

			//DEFINE HERE NEW INTEGRATION METHOD
			if (cmp4(integrationName, "Euler", 5) == 0){
				this->integrationMethod_GPU2 = (Euler_GPU2<AdExTimeDrivenModel_GPU2> *) new Euler_GPU2<AdExTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "RK2", 3) == 0){
				this->integrationMethod_GPU2 = (RK2_GPU2<AdExTimeDrivenModel_GPU2> *) new RK2_GPU2<AdExTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "RK4", 3) == 0){
				this->integrationMethod_GPU2 = (RK4_GPU2<AdExTimeDrivenModel_GPU2> *) new RK4_GPU2<AdExTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "BDF", 3) == 0 && atoiGPU(integrationName, 3)>0 && atoiGPU(integrationName, 3)<7){
				this->integrationMethod_GPU2 = (BDFn_GPU2<AdExTimeDrivenModel_GPU2> *) new BDFn_GPU2<AdExTimeDrivenModel_GPU2>(this, Buffer_GPU, atoiGPU(integrationName, 3));
			}
			else if (cmp4(integrationName, "Bifixed_Euler", 13) == 0){
				this->integrationMethod_GPU2 = (Bifixed_Euler_GPU2<AdExTimeDrivenModel_GPU2> *) new Bifixed_Euler_GPU2<AdExTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "Bifixed_RK2", 11) == 0){
				this->integrationMethod_GPU2 = (Bifixed_RK2_GPU2<AdExTimeDrivenModel_GPU2> *) new Bifixed_RK2_GPU2<AdExTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else if (cmp4(integrationName, "Bifixed_RK4", 11) == 0){
				this->integrationMethod_GPU2 = (Bifixed_RK4_GPU2<AdExTimeDrivenModel_GPU2> *) new Bifixed_RK4_GPU2<AdExTimeDrivenModel_GPU2>(this, Buffer_GPU);
			}
			else{
				printf("There was an error loading the integration methods of the GPU.\n");

			}
		}
};


#endif /* ADEXTIMEDRIVENMODEL_1_2_GPU2_H_ */
