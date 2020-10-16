/***************************************************************************
 *                           OriginalHHTimeDrivenModel.cpp		                   *
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

#include "../../include/neuron_model/OriginalHHTimeDrivenModel.h"
#include "../../include/neuron_model/VectorNeuronState.h"

#include <iostream>
#include <cmath>
#include <string>

#include "../../include/openmp/openmp.h"

#include "../../include/spike/EDLUTFileException.h"
#include "../../include/spike/Neuron.h"
#include "../../include/spike/InternalSpike.h"
#include "../../include/spike/PropagatedSpike.h"
#include "../../include/spike/Interconnection.h"

#include "../../include/simulation/Utils.h"

#include "../../include/openmp/openmp.h"




void OriginalHHTimeDrivenModel::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
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
							this->inv_cm = 1.0f / this->cm;
							skip_comments(fh, Currentline);
							if (fscanf(fh, "%f", &this->vthr) == 1){
								skip_comments(fh, Currentline);
								if (fscanf(fh, "%f", &this->texc) == 1 && this->texc > 0.0f){
									this->inv_texc = 1.0f / this->texc;
									skip_comments(fh, Currentline);
									if (fscanf(fh, "%f", &this->tinh) == 1 && this->tinh > 0.0f){
										this->inv_tinh = 1.0f / this->tinh;
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
		
															this->State = (VectorNeuronState *) new VectorNeuronState(N_NeuronStateVariables, true);

														}else {
															throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_VT, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
														}
													}else {
														throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_EK, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
													}
												}else {
													throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_ENA, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
												}
											}else {
												throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_GKD, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
											}
										}else {
											throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_GNA, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
										}
									}else {
										throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_TINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
									}
								}else {
									throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_TEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
								}
							}else {
								throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_VTHR, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
							}
						}else {
							throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_CM, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
						}
					}else {
						throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_GREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
					}
				}else {
					throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_EREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
				}
			}else {
				throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_EINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
			}
		}else {
			throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_HH_TIME_DRIVEN_MODEL_EEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
		}

		//INTEGRATION METHOD
		loadIntegrationMethod(this->GetModelID(), fh, &Currentline);

		this->integrationMethod->SetBiFixedStepParameters(VT-5, VT-5, 2.0f);

		this->integrationMethod->Calculate_conductance_exp_values();

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_ORIGINAL_HH_TIME_DRIVEN_MODEL_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

OriginalHHTimeDrivenModel::OriginalHHTimeDrivenModel(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel(NeuronTypeID, NeuronModelID, MilisecondScale){
	//eexc=0.0f; //mV
	//einh=-80.0f; //mV
	//erest=-65.0f; //mV
	//vthr=-30.0f; //mV
	//cm=90.0f; //pF   
	//inv_cm=1.0f/cm;
	//texc=5.0f;//ms
	//inv_texc=1.0f/texc;
	//tinh=10.0f;//ms
	//inv_tinh=1.0f/tinh;
	//grest=10.0f;//nS
	//gNa=20000.0;//nS
	//gKd=6000.0f;//nS
	//ENa=50.0f;//mV
	//EK=-90.0f;//mV
	//VT=-51.0f;//mV
}

OriginalHHTimeDrivenModel::~OriginalHHTimeDrivenModel(void)
{
}

void OriginalHHTimeDrivenModel::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * OriginalHHTimeDrivenModel::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * OriginalHHTimeDrivenModel::ProcessInputSpike(Interconnection * inter, double time){
	// Add the effect of the input spike
	this->GetVectorNeuronState()->IncrementStateVariableAtCPU(inter->GetTargetNeuronModelIndex(), N_DifferentialNeuronState + inter->GetType(), inter->GetWeight());

	return 0;
}


bool OriginalHHTimeDrivenModel::UpdateState(int index, double CurrentTime){
	//NeuronState[0] --> vm 
	//NeuronState[1] --> m 
	//NeuronState[2] --> h 
	//NeuronState[3] --> n 
	//NeuronState[4] --> gexc 
	//NeuronState[5] --> ginh 

	//Reset the number of internal spikes in this update period
	this->State->NInternalSpikeIndexs = 0;

	this->integrationMethod->NextDifferentialEquationValues();

	this->CheckValidIntegration(CurrentTime, this->integrationMethod->GetValidIntegrationVariable());

	return false;
}

enum NeuronModelOutputActivityType OriginalHHTimeDrivenModel::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType OriginalHHTimeDrivenModel::GetModelInputActivityType(){
	return INPUT_SPIKE;
}


ostream & OriginalHHTimeDrivenModel::PrintInfo(ostream & out){
	//out << "- Leaky Time-Driven Model: " << this->GetModelID() << endl;

	//out << "\tExc. Reversal Potential: " << this->eexc << "V\tInh. Reversal Potential: " << this->einh << "V\tResting potential: " << this->erest << "V" << endl;

	//out << "\tFiring threshold: " << this->vthr << "V\tMembrane capacitance: " << this->cm << "nS\tExcitatory Time Constant: " << this->texc << "s" << endl;

	//out << "\tInhibitory time constant: " << this->tinh << "s\tRefractory Period: " << this->tref << "s\tResting Conductance: " << this->grest << "nS" << endl;

	return out;
}	



void OriginalHHTimeDrivenModel::InitializeStates(int N_neurons, int OpenMPQueueIndex){
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
	State->InitializeStates(N_neurons, initialization);

	//Initialize integration method state variables.
	this->integrationMethod->InitializeStates(N_neurons, initialization);
}


void OriginalHHTimeDrivenModel::EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elpasedTimeInNeuronModelScale){

	if (NeuronState[0] > this->vthr && previous_V < this->vthr){
		State->NewFiredSpike(index);
		this->State->InternalSpikeIndexs[this->State->NInternalSpikeIndexs] = index;
		this->State->NInternalSpikeIndexs++;
	}
}


void OriginalHHTimeDrivenModel::EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){
	float V=NeuronState[0];
	float m=NeuronState[1];
	float h=NeuronState[2];
	float n=NeuronState[3];
	float gexc=NeuronState[4];
	float ginh=NeuronState[5];

	if(m<0){
		m=0.0f;
	}

	//V
	AuxNeuronState[0]=(gexc * (this->eexc - V) + ginh * (this->einh - V) + grest * (this->erest - V) + gNa*m*m*m*h*(ENa - V) + gKd*n*n*n*n*(EK - V))*this->inv_cm;


	//m
	float alpha_m, beta_m;
	if((13.0f-V+VT)!=0.0){
		alpha_m=0.32f*(13.0f-V+VT)/(exp((13.0f-V+VT)/4.0f)-1.0f);
	}else{
		alpha_m=1.279999894092334f;
	}

	if((V-VT-40.0f)!=0.0){
		beta_m=0.28f*(V-VT-40.0f)/(exp((V-VT-40.0f)/5.0f)-1.0f);
	}else{
		beta_m=1.400030971093304f;
	}
	AuxNeuronState[1]=(alpha_m*(1.0f-m)-beta_m*m);

	//h
	float alpha_h=0.128f*exp((17.0f-V+VT)/18.0f);
	float beta_h=4.0f/(1.0f+exp((40.0f-V+VT)/5.0f));
	AuxNeuronState[2]=(alpha_h*(1-h)-beta_h*h);

	
	//n
	float alpha_n;
	if((15.0f-V+VT)!=0.0){
		alpha_n=0.032f*(15.0f-V+VT)/(exp((15.0f-V+VT)/5.0f)-1.0f);
	}else{
		alpha_n=0.1599999867615418f;
	}
	float beta_n=0.5f*exp((10.0f-V+VT)/40.0f);
	AuxNeuronState[3]=(alpha_n*(1-n)-beta_n*n);
}

void OriginalHHTimeDrivenModel::EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){
	float limit=1e-9;
	float * Conductance_values=this->Get_conductance_exponential_values(elapsed_time_index);
	
	if(NeuronState[N_DifferentialNeuronState]<limit){
		NeuronState[N_DifferentialNeuronState]=0.0f;
	}else{
		NeuronState[N_DifferentialNeuronState]*= Conductance_values[0];
	}
	if(NeuronState[N_DifferentialNeuronState+1]<limit){
		NeuronState[N_DifferentialNeuronState+1]=0.0f;
	}else{
		NeuronState[N_DifferentialNeuronState+1]*= Conductance_values[1];
	}
}

void OriginalHHTimeDrivenModel::Calculate_conductance_exp_values(int index, float elapsed_time){
	//excitatory synapse.
	Set_conductance_exp_values(index, 0, exp(-elapsed_time*this->inv_texc));
	//inhibitory synapse.
	Set_conductance_exp_values(index, 1, exp(-elapsed_time*this->inv_tinh));
}


bool OriginalHHTimeDrivenModel::CheckSynapseType(Interconnection * connection){
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



void OriginalHHTimeDrivenModel::loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException){
	char ident_type[MAXIDSIZE + 1];

	//We load the integration method type.
	skip_comments(fh, *Currentline);
	if (fscanf(fh, "%s", ident_type) == 1){
		skip_comments(fh, *Currentline);
		//DEFINE HERE NEW INTEGRATION METHOD
		if (strncmp(ident_type, "Euler", 5) == 0){
			integrationMethod = (Euler<OriginalHHTimeDrivenModel> *) new Euler<OriginalHHTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "RK2", 3) == 0){
			integrationMethod = (RK2<OriginalHHTimeDrivenModel> *) new RK2<OriginalHHTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "RK4", 3) == 0){
			integrationMethod = (RK4<OriginalHHTimeDrivenModel> *) new RK4<OriginalHHTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "BDF", 3) == 0 && atoi(&ident_type[3])>0 && atoi(&ident_type[3])<7){
			integrationMethod = (BDFn<OriginalHHTimeDrivenModel> *) new BDFn<OriginalHHTimeDrivenModel>(this, atoi(&ident_type[3]));
		}
		else if (strncmp(ident_type, "Bifixed_Euler", 13) == 0){
			integrationMethod = (Bifixed_Euler<OriginalHHTimeDrivenModel> *) new Bifixed_Euler<OriginalHHTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK2", 11) == 0){
			integrationMethod = (Bifixed_RK2<OriginalHHTimeDrivenModel> *) new Bifixed_RK2<OriginalHHTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK4", 11) == 0){
			integrationMethod = (Bifixed_RK4<OriginalHHTimeDrivenModel> *) new Bifixed_RK4<OriginalHHTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "Bifixed_BDF", 11) == 0 && atoi(&ident_type[11]) == 2){
			integrationMethod = (Bifixed_BDFn<OriginalHHTimeDrivenModel> *) new Bifixed_BDFn<OriginalHHTimeDrivenModel>(this, atoi(&ident_type[11]));
		}
		else{
			throw EDLUTFileException(TASK_INTEGRATION_METHOD_TYPE, ERROR_INTEGRATION_METHOD_TYPE, REPAIR_INTEGRATION_METHOD_TYPE, *Currentline, fileName.c_str(), true);
		}
	}
	else{
		throw EDLUTFileException(TASK_INTEGRATION_METHOD_TYPE, ERROR_INTEGRATION_METHOD_READ, REPAIR_INTEGRATION_METHOD_READ, *Currentline, fileName.c_str(), true);
	}

	//We load the integration method parameter.
	integrationMethod->loadParameter((TimeDrivenNeuronModel *)this, fh, Currentline, fileName.c_str());
}
