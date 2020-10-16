/***************************************************************************
 *                           AdExTimeDrivenModel.cpp                       *
 *                           -------------------                           *
 * copyright            : (C) 2015 by Francisco Naveros                    *
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

#include "../../include/neuron_model/AdExTimeDrivenModel.h"
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


void AdExTimeDrivenModel::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
	FILE *fh;
	long Currentline = 0L;
	fh=fopen(ConfigFile.c_str(),"rt");
	if(fh){
		Currentline = 1L;
		skip_comments(fh, Currentline);
		if (fscanf(fh, "%f", &this->a) == 1){
			skip_comments(fh, Currentline);	
			if (fscanf(fh, "%f", &this->b) == 1){
				skip_comments(fh, Currentline);
				if (fscanf(fh, "%f", &this->TSF) == 1 && this->TSF > 0.0f){
					this->inv_TSF = 1.0f / this->TSF;
					skip_comments(fh, Currentline);
					if (fscanf(fh, "%f", &this->VT) == 1){
						skip_comments(fh, Currentline);
						if (fscanf(fh, "%f", &this->tauw) == 1 && this->tauw > 0.0f){
							this->inv_tauw = 1.0f / this->tauw;
							skip_comments(fh, Currentline);
							if (fscanf(fh, "%f", &this->eexc) == 1){
								skip_comments(fh, Currentline);
								if (fscanf(fh, "%f", &this->einh) == 1){
									skip_comments(fh, Currentline);
									if (fscanf(fh, "%f", &this->Ereset) == 1){
										skip_comments(fh, Currentline);
										if (fscanf(fh, "%f", &this->Eleak) == 1){
											skip_comments(fh, Currentline);
											if (fscanf(fh, "%f", &this->gleak) == 1 && this->gleak > 0.0f){
												skip_comments(fh, Currentline);
												if (fscanf(fh, "%f", &this->cm) == 1 && this->cm > 0.0f){
													this->inv_cm = 1.0f / this->cm;
													skip_comments(fh, Currentline);
													if (fscanf(fh, "%f", &this->texc) == 1 && this->texc > 0.0f){
														this->inv_texc = 1.0f / this->texc;
														skip_comments(fh, Currentline);
														if (fscanf(fh, "%f", &this->tinh) == 1 && this->tinh > 0.0f){
															this->inv_tinh = 1.0f / this->tinh;
															skip_comments(fh, Currentline);

															this->State = (VectorNeuronState *) new VectorNeuronState(N_NeuronStateVariables, true);

														}else {
															throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_TINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
														}
													}else {
														throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_TEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
													}
												}else {
													throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_CM, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
												}
											}else {
												throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_GLEAK, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
											}
										}else {
											throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_ELEAK, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
										}
									}else {
										throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_ERESET, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
									}
								}else {
									throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_EINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
								}
							}else {
								throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_EEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
							}
						}else {
							throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_TAUW, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
						}
					}else {
						throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_VT, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
					}
				}else {
					throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_TSF, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
				}
			}else {
				throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_B, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
			}
		}else {
			throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_ADEX_TIME_DRIVEN_MODEL_A, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
		}

		//INTEGRATION METHOD
		loadIntegrationMethod(this->GetModelID(), fh, &Currentline);

		this->integrationMethod->SetBiFixedStepParameters(VT,VT,2.0f);

		this->integrationMethod->Calculate_conductance_exp_values();

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_ADEX_TIME_DRIVEN_MODEL_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

//this neuron model is implemented in a milisecond scale.
AdExTimeDrivenModel::AdExTimeDrivenModel(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel(NeuronTypeID, NeuronModelID, MilisecondScale){
		//a=1.0f;  //nS
		//b=9.0f;  //pA
		//TSF=2.0f;  //mV
		//inv_TSF=1.0f/TSF;
		//VT=-50.0f;//mV
		//tauw=50.0f ; //ms   
		//inv_tauw=1.0f/tauw;
		//eexc=0.0f;//mV
		//einh=-80.0f;//mV
		//Ereset=-80.0f;//mV
		//Eleak=-65.0f;//mV
		//gleak=10;//nS
		//cm=110.0f;//pF
		//inv_cm=1.0f/cm;
		//texc=5.0f;//ms
		//inv_texc=1.0f/texc;
		//tinh=10.0f;//ms
		//inv_tinh=1.0f/tinh;
}

AdExTimeDrivenModel::~AdExTimeDrivenModel(void)
{
}

void AdExTimeDrivenModel::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * AdExTimeDrivenModel::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * AdExTimeDrivenModel::ProcessInputSpike(Interconnection * inter, double time){
	// Add the effect of the input spike
	this->GetVectorNeuronState()->IncrementStateVariableAtCPU(inter->GetTargetNeuronModelIndex(), N_DifferentialNeuronState + inter->GetType(), inter->GetWeight());

	return 0;
}


bool AdExTimeDrivenModel::UpdateState(int index, double CurrentTime){
	//	//NeuronState[0] --> V 
	//	//NeuronState[1] --> w 
	//	//NeuronState[2] --> gexc 
	//	//NeuronState[3] --> ginh 

	//Reset the number of internal spikes in this update period
	this->State->NInternalSpikeIndexs = 0;

	this->integrationMethod->NextDifferentialEquationValues();

	this->CheckValidIntegration(CurrentTime, this->integrationMethod->GetValidIntegrationVariable());

	return false;
}


enum NeuronModelOutputActivityType AdExTimeDrivenModel::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType AdExTimeDrivenModel::GetModelInputActivityType(){
	return INPUT_SPIKE;
}



ostream & AdExTimeDrivenModel::PrintInfo(ostream & out){
	return out;
}	



void AdExTimeDrivenModel::InitializeStates(int N_neurons, int OpenMPQueueIndex){
	//Initialize neural state variables.
	float initialization[] = {this->Eleak,0.0f,0.0f,0.0f};
	State->InitializeStates(N_neurons, initialization);

	//Initialize integration method state variables.
	this->integrationMethod->InitializeStates(N_neurons, initialization);
}

void AdExTimeDrivenModel::EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elpasedTimeInNeuronModelScale){
	if (NeuronState[0] > 0.0f){
		NeuronState[0] = this->Ereset;//V
		NeuronState[1] += this->b;//w 
		State->NewFiredSpike(index);
		this->integrationMethod->resetState(index);
		this->State->InternalSpikeIndexs[this->State->NInternalSpikeIndexs] = index;
		this->State->NInternalSpikeIndexs++;
	}
}

void AdExTimeDrivenModel::EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){
	float V=NeuronState[0];
	float w=NeuronState[1];
	float gexc=NeuronState[2];
	float ginh=NeuronState[3];

	//V
	if(V<=-20.0f){
		//V
		AuxNeuronState[0]=(-gleak*(V-Eleak) + gleak*TSF*ExponentialTable::GetResult((V-VT)*inv_TSF) - w - gexc * (V - this->eexc) - ginh * (V - this->einh))*this->inv_cm;
		//w
		AuxNeuronState[1]=(a*(V - Eleak) - w)*this->inv_tauw;
	}else if(V<=0.0f){
		float Vm=-20.0f;
		//V
		AuxNeuronState[0]=(-gleak*(Vm-Eleak) + gleak*TSF*ExponentialTable::GetResult((Vm-VT)*inv_TSF) - w - gexc * (Vm - this->eexc) - ginh * (Vm - this->einh))*this->inv_cm;
		//w
		AuxNeuronState[1]=(a*(V - Eleak) - w)*this->inv_tauw;
	}else{
		//V
		AuxNeuronState[0]=0;
		//w
		AuxNeuronState[1]=0;
	}
}


void AdExTimeDrivenModel::EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){
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


void AdExTimeDrivenModel::Calculate_conductance_exp_values(int index, float elapsed_time){
	//excitatory synapse.
	Set_conductance_exp_values(index, 0, exp(-elapsed_time*this->inv_texc));
	//inhibitory synapse.
	Set_conductance_exp_values(index, 1, exp(-elapsed_time*this->inv_tinh));
}


bool AdExTimeDrivenModel::CheckSynapseType(Interconnection * connection){
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


void AdExTimeDrivenModel::loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException){
	char ident_type[MAXIDSIZE + 1];

	//We load the integration method type.
	skip_comments(fh, *Currentline);
	if (fscanf(fh, "%s", ident_type) == 1){
		skip_comments(fh, *Currentline);
		//DEFINE HERE NEW INTEGRATION METHOD
		if (strncmp(ident_type, "Euler", 5) == 0){
			integrationMethod = (Euler<AdExTimeDrivenModel> *) new Euler<AdExTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "RK2", 3) == 0){
			integrationMethod = (RK2<AdExTimeDrivenModel> *) new RK2<AdExTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "RK4", 3) == 0){
			integrationMethod = (RK4<AdExTimeDrivenModel> *) new RK4<AdExTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "BDF", 3) == 0 && atoi(&ident_type[3])>0 && atoi(&ident_type[3])<7){
			integrationMethod = (BDFn<AdExTimeDrivenModel> *) new BDFn<AdExTimeDrivenModel>(this, atoi(&ident_type[3]));
		}
		else if (strncmp(ident_type, "Bifixed_Euler", 13) == 0){
			integrationMethod = (Bifixed_Euler<AdExTimeDrivenModel> *) new Bifixed_Euler<AdExTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK2", 11) == 0){
			integrationMethod = (Bifixed_RK2<AdExTimeDrivenModel> *) new Bifixed_RK2<AdExTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK4", 11) == 0){
			integrationMethod = (Bifixed_RK4<AdExTimeDrivenModel> *) new Bifixed_RK4<AdExTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "Bifixed_BDF", 11) == 0 && atoi(&ident_type[11]) == 2){
			integrationMethod = (Bifixed_BDFn<AdExTimeDrivenModel> *) new Bifixed_BDFn<AdExTimeDrivenModel>(this, atoi(&ident_type[11]));
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




