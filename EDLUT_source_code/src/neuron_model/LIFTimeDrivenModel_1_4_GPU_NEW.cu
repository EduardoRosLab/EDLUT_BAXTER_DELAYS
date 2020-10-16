/***************************************************************************
 *                           LIFTimeDrivenModel_1_4_GPU_NEW.cu             *
 *                           -------------------                           *
 * copyright            : (C) 2016 by Francisco Naveros                    *
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

#include "../../include/neuron_model/LIFTimeDrivenModel_1_4_GPU_NEW.h"
#include "../../include/neuron_model/LIFTimeDrivenModel_1_4_GPU2_NEW.h"
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
//#include <helper_cuda.h>
#include "cuda_runtime.h"
#include "device_launch_parameters.h"

void LIFTimeDrivenModel_1_4_GPU_NEW::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
	FILE *fh;
	long Currentline = 0L;
	fh=fopen(ConfigFile.c_str(),"rt");
	if(fh){
		Currentline=1L;
		skip_comments(fh,Currentline);
		if(fscanf(fh,"%f",&this->eexc)==1){
			skip_comments(fh,Currentline);

			if (fscanf(fh,"%f",&this->einh)==1){
				skip_comments(fh,Currentline);

				if(fscanf(fh,"%f",&this->erest)==1){
					skip_comments(fh,Currentline);

					if(fscanf(fh,"%f",&this->vthr)==1){
						skip_comments(fh,Currentline);

						if(fscanf(fh,"%f",&this->cm)==1 && this->cm > 0.0f){
							skip_comments(fh,Currentline);

							if(fscanf(fh,"%f",&this->tampa)==1 && this->tampa > 0.0f){
								skip_comments(fh,Currentline);

								if(fscanf(fh,"%f",&this->tnmda)==1 && this->tnmda > 0.0f){
									skip_comments(fh,Currentline);
									
									if(fscanf(fh,"%f",&this->tinh)==1 && this->tinh > 0.0f){
										skip_comments(fh,Currentline);

										if(fscanf(fh,"%f",&this->tgj)==1 && this->tgj > 0.0f){
											skip_comments(fh,Currentline);
											if(fscanf(fh,"%f",&this->tref)==1 && this->tref > 0.0f){
												skip_comments(fh,Currentline);

												if(fscanf(fh,"%f",&this->grest)==1 && this->grest > 0.0f){
													skip_comments(fh,Currentline);

													if(fscanf(fh,"%f",&this->fgj)==1){
														skip_comments(fh,Currentline);


														this->State = (VectorNeuronState_GPU *) new VectorNeuronState_GPU(N_NeuronStateVariables);

													}else {
														throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_FGJ, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
													}
												}else {
													throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_GREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
												}
											}else {
												throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_TREF, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
											}
										}else {
											throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_GAP, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
										}
									}else {
										throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_TINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
									}
								}else {
									throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_TNMDA, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
								}
							}else {
								throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_TAMPA, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
							}
						}else {
							throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_CM, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
						}
					}else {
						throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_VTHR, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
					}
				}else {
					throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_EREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
				}
			}else {
				throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_EINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
			}
		}else {
			throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_4_NEW_EEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
		}

  		//INTEGRATION METHOD
		this->integrationMethod_GPU = LoadIntegrationMethod_GPU::loadIntegrationMethod_GPU((TimeDrivenNeuronModel_GPU *)this, this->GetModelID(), fh, &Currentline, N_NeuronStateVariables, N_DifferentialNeuronState, N_TimeDependentNeuronState);

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod_GPU->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_4_GPU_NEW_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

LIFTimeDrivenModel_1_4_GPU_NEW::LIFTimeDrivenModel_1_4_GPU_NEW(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel_GPU(NeuronTypeID, NeuronModelID, MilisecondScale), eexc(0), einh(0), erest(0), vthr(0), cm(0), tampa(0), tnmda(0), tinh(0), tgj(0),
		tref(0), grest(0){
}

LIFTimeDrivenModel_1_4_GPU_NEW::~LIFTimeDrivenModel_1_4_GPU_NEW(void){
	DeleteClassGPU2();
}

void LIFTimeDrivenModel_1_4_GPU_NEW::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * LIFTimeDrivenModel_1_4_GPU_NEW::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * LIFTimeDrivenModel_1_4_GPU_NEW::ProcessInputSpike(Interconnection * inter, double time){
	this->State_GPU->AuxStateCPU[inter->GetType()*State_GPU->GetSizeState() + inter->GetTargetNeuronModelIndex()] += inter->GetWeight();

	return 0;
}


__global__ void LIFTimeDrivenModel_1_4_GPU_NEW_UpdateState(LIFTimeDrivenModel_1_4_GPU2_NEW ** NeuronModel_GPU2, double CurrentTime){
	(*NeuronModel_GPU2)->UpdateState(CurrentTime);
}
		
bool LIFTimeDrivenModel_1_4_GPU_NEW::UpdateState(int index, double CurrentTime){
	
	VectorNeuronState_GPU *state = (VectorNeuronState_GPU *) State;

	//----------------------------------------------
	if(prop.canMapHostMemory){
		LIFTimeDrivenModel_1_4_GPU_NEW_UpdateState<<<N_block,N_thread>>>(NeuronModel_GPU2, CurrentTime);
	}else{
		HANDLE_ERROR(cudaMemcpy(state->AuxStateGPU,state->AuxStateCPU,this->N_TimeDependentNeuronState*state->SizeStates*sizeof(float),cudaMemcpyHostToDevice));
		LIFTimeDrivenModel_1_4_GPU_NEW_UpdateState<<<N_block,N_thread>>>(NeuronModel_GPU2, CurrentTime);
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


enum NeuronModelOutputActivityType LIFTimeDrivenModel_1_4_GPU_NEW::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType LIFTimeDrivenModel_1_4_GPU_NEW::GetModelInputActivityType(){
	return INPUT_SPIKE;
}



ostream & LIFTimeDrivenModel_1_4_GPU_NEW::PrintInfo(ostream & out){
	out << "- Leaky Time-Driven Model: " << this->GetModelID() << endl;

	out << "\tExc. Reversal Potential: " << this->eexc << "mV\tInh. Reversal Potential: " << this->einh << "mV\tResting potential: " << this->erest << "mV" << endl;

	out << "\tFiring threshold: " << this->vthr << "mV\tMembrane capacitance: " << this->cm << "pF\tAMPA Time Constant: " << this->tampa << "ms\tNMDA Time Constant: " << this->tnmda << "ms"<< endl;

	out << "\tInhibitory time constant: " << this->tinh << "ms\tGap junction time constant: " << this->tgj << "ms\tRefractory Period: " << this->tref << "ms\tResting Conductance: " << this->grest << "nS" << endl;

	out << "\tGap junction factor: " << this->fgj << "mV/nS" << endl;

	return out;
}	


void LIFTimeDrivenModel_1_4_GPU_NEW::InitializeStates(int N_neurons, int OpenMPQueueIndex){

	//Select the correnpondent device. 
	HANDLE_ERROR(cudaSetDevice(GPUsIndex[OpenMPQueueIndex % NumberOfGPUs]));  
	HANDLE_ERROR(cudaEventCreate(&stop));
	HANDLE_ERROR(cudaGetDeviceProperties( &prop, GPUsIndex[OpenMPQueueIndex % NumberOfGPUs]));

	VectorNeuronState_GPU * state = (VectorNeuronState_GPU *) this->State;
	
	float initialization[] = {erest,0.0,0.0,0.0,0.0};
	state->InitializeStatesGPU(N_neurons, initialization, N_TimeDependentNeuronState, prop);

	//INITIALIZE CLASS IN GPU
	this->InitializeClassGPU2(N_neurons);


	InitializeVectorNeuronState_GPU2();
}




__global__ void LIFTimeDrivenModel_1_4_GPU_NEW_InitializeClassGPU2(LIFTimeDrivenModel_1_4_GPU2_NEW ** NeuronModel_GPU2, 
		float eexc,float einh,float erest,float vthr,float cm,float tampa,float tnmda,float tinh,float tgj,float tref,
		float grest,float fgj, char const* integrationName, int N_neurons, void ** Buffer_GPU){
	if(blockIdx.x==0 && threadIdx.x==0){
		(*NeuronModel_GPU2) = new LIFTimeDrivenModel_1_4_GPU2_NEW(eexc,einh,erest,vthr,cm,
        tampa,tnmda,tinh,tgj,tref,grest,fgj,integrationName, N_neurons, Buffer_GPU);
	}
}
void LIFTimeDrivenModel_1_4_GPU_NEW::InitializeClassGPU2(int N_neurons){
	cudaMalloc(&NeuronModel_GPU2, sizeof(LIFTimeDrivenModel_1_4_GPU2_NEW **));
	
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

	LIFTimeDrivenModel_1_4_GPU_NEW_InitializeClassGPU2<<<1,1>>>(NeuronModel_GPU2, eexc,einh,erest,vthr,cm,tampa,
		tnmda,tinh,tgj,tref,grest,fgj,integrationNameGPU, N_neurons, integrationMethod_GPU->Buffer_GPU);

	cudaFree(integrationNameGPU);
}



__global__ void initializeVectorNeuronState_GPU2(LIFTimeDrivenModel_1_4_GPU2_NEW ** NeuronModel_GPU2, int NumberOfVariables, float * InitialStateGPU, float * AuxStateGPU, float * StateGPU, double * LastUpdateGPU, double * LastSpikeTimeGPU, bool * InternalSpikeGPU, int SizeStates){
	if(blockIdx.x==0 && threadIdx.x==0){
		(*NeuronModel_GPU2)->InitializeVectorNeuronState_GPU2(NumberOfVariables, InitialStateGPU, AuxStateGPU, StateGPU, LastUpdateGPU, LastSpikeTimeGPU, InternalSpikeGPU, SizeStates);
	}
}

void LIFTimeDrivenModel_1_4_GPU_NEW::InitializeVectorNeuronState_GPU2(){
	VectorNeuronState_GPU *state = (VectorNeuronState_GPU *) State;
	initializeVectorNeuronState_GPU2<<<1,1>>>(NeuronModel_GPU2, state->NumberOfVariables, state->InitialStateGPU, state->AuxStateGPU, state->VectorNeuronStates_GPU, state->LastUpdateGPU, state->LastSpikeTimeGPU, state->InternalSpikeGPU, state->SizeStates);
}


__global__ void DeleteClass_GPU2(LIFTimeDrivenModel_1_4_GPU2_NEW ** NeuronModel_GPU2){
	if(blockIdx.x==0 && threadIdx.x==0){
		delete (*NeuronModel_GPU2); 
	}
}


void LIFTimeDrivenModel_1_4_GPU_NEW::DeleteClassGPU2(){
    DeleteClass_GPU2<<<1,1>>>(NeuronModel_GPU2);
    cudaFree(NeuronModel_GPU2);
}


bool LIFTimeDrivenModel_1_4_GPU_NEW::CheckSynapseType(Interconnection * connection){
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



