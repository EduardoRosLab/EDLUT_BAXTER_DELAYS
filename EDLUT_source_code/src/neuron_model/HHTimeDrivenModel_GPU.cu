/***************************************************************************
 *                           LIFTimeDrivenModel_1_2_GPU.cu                 *
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

#include "../../include/neuron_model/HHTimeDrivenModel_GPU.h"
#include "../../include/neuron_model/HHTimeDrivenModel_GPU2.h"
#include "../../include/neuron_model/VectorNeuronState.h"
#include "../../include/neuron_model/VectorNeuronState_GPU.h"

#include <iostream>
#include <cmath>
#include <string>

#include "../../include/spike/EDLUTFileException.h"
#include "../../include/spike/Neuron.h"
#include "../../include/spike/InternalSpike.h"
#include "../../include/spike/PropagatedSpike.h"
#include "../../include/spike/Interconnection.h"

#include "../../include/simulation/Utils.h"

#include "../../include/openmp/openmp.h"

#include "../../include/cudaError.h"
//Library for CUDA
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

void HHTimeDrivenModel_GPU::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
	FILE *fh;
	long Currentline = 0L;
	fh=fopen(ConfigFile.c_str(),"rt");
	if(fh){
		Currentline = 1L;
		skip_comments(fh, Currentline);
		if (fscanf(fh, "%f", &this->eexc) == 1){
			skip_comments(fh, Currentline);
			if (fscanf(fh, "%f", &this->einh) == 1){
				skip_comments(fh, Currentline);
				if (fscanf(fh, "%f", &this->erest) == 1){
					skip_comments(fh, Currentline);
					if (fscanf(fh, "%f", &this->grest) == 1 && this->grest > 0.0f){
						skip_comments(fh, Currentline);
						if (fscanf(fh, "%f", &this->cm) == 1 && this->cm > 0.0f){
							skip_comments(fh, Currentline);
							if (fscanf(fh, "%f", &this->vthr) == 1){
								skip_comments(fh, Currentline);
								if (fscanf(fh, "%f", &this->texc) == 1 && this->texc > 0.0f){
									skip_comments(fh, Currentline);
									if (fscanf(fh, "%f", &this->tinh) == 1 && this->tinh > 0.0f){
										skip_comments(fh, Currentline);
										if (fscanf(fh, "%f", &this->gNa) == 1 && this->gNa > 0.0f){
											skip_comments(fh, Currentline);
											if (fscanf(fh, "%f", &this->gKd) == 1 && this->gKd > 0.0f){
												skip_comments(fh, Currentline);
												if (fscanf(fh, "%f", &this->ENa) == 1){
													skip_comments(fh, Currentline);
													if (fscanf(fh, "%f", &this->EK) == 1){
														skip_comments(fh, Currentline);
														if (fscanf(fh, "%f", &this->VT) == 1){

															this->State = (VectorNeuronState_GPU *) new VectorNeuronState_GPU(N_NeuronStateVariables);

														}else {
															throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_VT, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
														}
													}else {
														throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_EK, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
													}
												}else {
													throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_ENA, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
												}
											}else {
												throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_GKD, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
											}
										}else {
											throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_GNA, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
										}
									}else {
										throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_TINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
									}
								}else {
									throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_TEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
								}
							}else {
								throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_VTHR, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
							}
						}else {
							throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_CM, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
						}
					}else {
						throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_GREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
					}
				}else {
					throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_EREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
				}
			}else {
				throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_EINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
			}
		}else {
			throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_EEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
		}

  		//INTEGRATION METHOD
		this->integrationMethod_GPU = LoadIntegrationMethod_GPU::loadIntegrationMethod_GPU((TimeDrivenNeuronModel_GPU *)this, this->GetModelID(), fh, &Currentline, N_NeuronStateVariables, N_DifferentialNeuronState, N_TimeDependentNeuronState);

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod_GPU->elapsedTimeInSeconds);
	}
	else{
		throw EDLUTFileException(TASK_HH_TIME_DRIVEN_MODEL_GPU_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

HHTimeDrivenModel_GPU::HHTimeDrivenModel_GPU(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel_GPU(NeuronTypeID, NeuronModelID, MilisecondScale){
	//eexc=0.0f; //mV
	//einh=-80.0f; //mV
	//erest=-65.0f; //mV
	//vthr=-30.0f; //mV
	//cm=120.0f; //pF   
	//texc=5.0f;//ms
	//tinh=10.0f;//ms
	//grest=10.0f;//nS
	//gNa=20000.0;//nS
	//gKd=6000.0f;//nS
	//ENa=50.0f;//mV
	//EK=-90.0f;//mV
	//VT=-52.0f;//mV

}

HHTimeDrivenModel_GPU::~HHTimeDrivenModel_GPU(void){
	DeleteClassGPU2();
}

void HHTimeDrivenModel_GPU::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * HHTimeDrivenModel_GPU::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * HHTimeDrivenModel_GPU::ProcessInputSpike(Interconnection * inter, double time){
	this->State_GPU->AuxStateCPU[inter->GetType()*State_GPU->GetSizeState() + inter->GetTargetNeuronModelIndex()] += inter->GetWeight();

	return 0;
}


__global__ void HHTimeDrivenModel_GPU_UpdateState(HHTimeDrivenModel_GPU2 ** NeuronModel_GPU2, double CurrentTime){
	(*NeuronModel_GPU2)->UpdateState(CurrentTime);
}

		
bool HHTimeDrivenModel_GPU::UpdateState(int index, double CurrentTime){
	VectorNeuronState_GPU *state = (VectorNeuronState_GPU *) State;

	//----------------------------------------------
	if(prop.canMapHostMemory){
		HHTimeDrivenModel_GPU_UpdateState<<<N_block,N_thread>>>(NeuronModel_GPU2, CurrentTime);
	}else{
		HANDLE_ERROR(cudaMemcpy(state->AuxStateGPU,state->AuxStateCPU,this->N_TimeDependentNeuronState*state->SizeStates*sizeof(float),cudaMemcpyHostToDevice));
		HHTimeDrivenModel_GPU_UpdateState<<<N_block,N_thread>>>(NeuronModel_GPU2, CurrentTime);
		HANDLE_ERROR(cudaMemcpy(state->InternalSpikeCPU,state->InternalSpikeGPU,state->SizeStates*sizeof(bool),cudaMemcpyDeviceToHost));
	}


	if(this->GetVectorNeuronState()->Get_Is_Monitored()){
		HANDLE_ERROR(cudaMemcpy(state->VectorNeuronStates,state->VectorNeuronStates_GPU,state->GetNumberOfVariables()*state->SizeStates*sizeof(float),cudaMemcpyDeviceToHost));
		HANDLE_ERROR(cudaMemcpy(state->LastUpdate,state->LastUpdateGPU,state->SizeStates*sizeof(double),cudaMemcpyDeviceToHost));
		HANDLE_ERROR(cudaMemcpy(state->LastSpikeTime,state->LastSpikeTimeGPU,state->SizeStates*sizeof(double),cudaMemcpyDeviceToHost));
	}
 

	HANDLE_ERROR(cudaEventRecord(stop, 0)); 
	HANDLE_ERROR(cudaEventSynchronize(stop));


	memset(state->AuxStateCPU,0,N_TimeDependentNeuronState*state->SizeStates*sizeof(float));

	return false;

}


enum NeuronModelOutputActivityType HHTimeDrivenModel_GPU::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType HHTimeDrivenModel_GPU::GetModelInputActivityType(){
	return INPUT_SPIKE;
}


ostream & HHTimeDrivenModel_GPU::PrintInfo(ostream & out){
	return out;
}	


void HHTimeDrivenModel_GPU::InitializeStates(int N_neurons, int OpenMPQueueIndex){

	//Select the correnpondent device. 
	HANDLE_ERROR(cudaSetDevice(GPUsIndex[OpenMPQueueIndex % NumberOfGPUs]));  
	HANDLE_ERROR(cudaEventCreate(&stop));
	HANDLE_ERROR(cudaGetDeviceProperties( &prop, GPUsIndex[OpenMPQueueIndex % NumberOfGPUs]));


	this->State_GPU = (VectorNeuronState_GPU *) this->State;
	
	//Initialize neural state variables.
	//m
	float alpha_m=0.32f*(13.0f-erest+VT)/(exp((13.0f-erest+VT)/4.0f)-1.0f);
	float beta_m=0.28f*(erest-VT-40.0f)/(exp((erest-VT-40.0f)/5.0f)-1.0f);
	float m_inf=alpha_m/(alpha_m+beta_m);

	//h
	float alpha_h=0.128f*exp((17.0f-erest+VT)/18.0f);
	float beta_h=4.0f/(1.0f+exp((40.0f-erest+VT)/5.0f));
	float h_inf=alpha_h/(alpha_h+beta_h);


	//n
	float alpha_n=0.032f*(15.0f-erest+VT)/(exp((15.0f-erest+VT)/5.0f)-1.0f);
	float beta_n=0.5f*exp((10.0f-erest+VT)/40.0f);
	float n_inf=alpha_n/(alpha_n+beta_n);

	float initialization[] = {erest,m_inf, h_inf, n_inf, 0.0f, 0.0f};

	State_GPU->InitializeStatesGPU(N_neurons, initialization, N_TimeDependentNeuronState, prop);

	//INITIALIZE CLASS IN GPU
	this->InitializeClassGPU2(N_neurons);


	InitializeVectorNeuronState_GPU2();
}




__global__ void HHTimeDrivenModel_GPU_InitializeClassGPU2(HHTimeDrivenModel_GPU2 ** NeuronModel_GPU2, 
		float new_eexc, float new_einh, float new_erest, float new_grest, float new_cm, float new_vthr, float new_texc, 
		float new_tinh, float new_gNa, float new_gKd, float new_ENa, float new_EK, float new_VT,
		char const* integrationName, int N_neurons, void ** Buffer_GPU){
	if(blockIdx.x==0 && threadIdx.x==0){
		(*NeuronModel_GPU2)=new HHTimeDrivenModel_GPU2(new_eexc, new_einh, new_erest, new_grest, new_cm, new_vthr, new_texc, 
				new_tinh, new_gNa, new_gKd, new_ENa, new_EK, new_VT, integrationName, N_neurons, Buffer_GPU);
	}
}

void HHTimeDrivenModel_GPU::InitializeClassGPU2(int N_neurons){
	cudaMalloc(&NeuronModel_GPU2, sizeof(HHTimeDrivenModel_GPU **));
	
	char * integrationNameGPU;
	cudaMalloc((void **)&integrationNameGPU,32*4);
	HANDLE_ERROR(cudaMemcpy(integrationNameGPU,integrationMethod_GPU->GetType(),32*4,cudaMemcpyHostToDevice));

	this->N_thread = 128;
	this->N_block=prop.multiProcessorCount*16;
	if((N_neurons+N_thread-1)/N_thread < N_block){
		N_block = (N_neurons+N_thread-1)/N_thread;
	}
	int Total_N_thread=N_thread*N_block;

	integrationMethod_GPU->InitializeMemoryGPU(N_neurons, Total_N_thread);

	HHTimeDrivenModel_GPU_InitializeClassGPU2<<<1,1>>>(NeuronModel_GPU2, eexc, einh, erest, grest, cm, vthr, texc, 
		tinh, gNa, gKd, ENa, EK, VT, integrationNameGPU, N_neurons, integrationMethod_GPU->Buffer_GPU);

	cudaFree(integrationNameGPU);
}



__global__ void initializeVectorNeuronState_GPU2(HHTimeDrivenModel_GPU2 ** NeuronModel_GPU2, int NumberOfVariables, float * InitialStateGPU, float * AuxStateGPU, float * StateGPU, double * LastUpdateGPU, double * LastSpikeTimeGPU, bool * InternalSpikeGPU, int SizeStates){
	if(blockIdx.x==0 && threadIdx.x==0){
		(*NeuronModel_GPU2)->InitializeVectorNeuronState_GPU2(NumberOfVariables, InitialStateGPU, AuxStateGPU, StateGPU, LastUpdateGPU, LastSpikeTimeGPU, InternalSpikeGPU, SizeStates);
	}
}

void HHTimeDrivenModel_GPU::InitializeVectorNeuronState_GPU2(){
	VectorNeuronState_GPU *state = (VectorNeuronState_GPU *) State;
	initializeVectorNeuronState_GPU2<<<1,1>>>(NeuronModel_GPU2, state->NumberOfVariables, state->InitialStateGPU, state->AuxStateGPU, state->VectorNeuronStates_GPU, state->LastUpdateGPU, state->LastSpikeTimeGPU, state->InternalSpikeGPU, state->SizeStates);
}


__global__ void DeleteClass_GPU2(HHTimeDrivenModel_GPU2 ** NeuronModel_GPU2){
	if(blockIdx.x==0 && threadIdx.x==0){
		delete (*NeuronModel_GPU2); 
	}
}


void HHTimeDrivenModel_GPU::DeleteClassGPU2(){
    DeleteClass_GPU2<<<1,1>>>(NeuronModel_GPU2);
    cudaFree(NeuronModel_GPU2);
}


bool HHTimeDrivenModel_GPU::CheckSynapseType(Interconnection * connection){
	int Type = connection->GetType();
	if (Type<N_TimeDependentNeuronState && Type >= 0){
		NeuronModel * model = connection->GetSource()->GetNeuronModel();
		//Synapse types that process input spikes 
		if (Type < N_TimeDependentNeuronState && model->GetModelOutputActivityType() == OUTPUT_SPIKE)
			return true;
		else{
			cout << "Synapses type " << Type << " of neuron model " << this->GetTypeID() << ", " << this->GetModelID() << " must receive spikes. The source model generates currents." << endl;
			return false;
		}
		//Synapse types that process input current 
	}
	else{
		cout << "Neuron model " << this->GetTypeID() << ", " << this->GetModelID() << " does not support input synapses of type " << Type << ". Just defined " << N_TimeDependentNeuronState << " synapses types." << endl;
		return false;
	}
}