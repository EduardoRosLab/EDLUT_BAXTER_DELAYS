/***************************************************************************
 *                           IzhikevichTimeDrivenModel.cpp                 *
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

#include "../../include/neuron_model/IzhikevichTimeDrivenModel.h"
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


void IzhikevichTimeDrivenModel::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
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
				if (fscanf(fh, "%f", &this->c) == 1){
					skip_comments(fh, Currentline);
					if (fscanf(fh, "%f", &this->d) == 1){
						skip_comments(fh, Currentline);
						if (fscanf(fh, "%f", &this->eexc) == 1){
							skip_comments(fh, Currentline);
							if (fscanf(fh, "%f", &this->einh) == 1){
								skip_comments(fh, Currentline);
								if (fscanf(fh, "%f", &this->cm) == 1 && this->cm > 0.0f){
									inv_cm = 1.0f / cm;
									skip_comments(fh, Currentline);
									if (fscanf(fh, "%f", &this->texc) == 1 && this->texc > 0.0f){
										inv_texc = 1.0f / texc;
										skip_comments(fh, Currentline);
										if (fscanf(fh, "%f", &this->tinh) == 1 && this->tinh > 0.0f){
											inv_tinh = 1.0f / tinh;
											skip_comments(fh, Currentline);

											this->State = (VectorNeuronState *) new VectorNeuronState(N_NeuronStateVariables, true);

										}else {
											throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_IZHIKEVICH_TIME_DRIVEN_MODEL_TINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
										}
									}else {
										throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_IZHIKEVICH_TIME_DRIVEN_MODEL_TEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
									}
								}else {
									throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_IZHIKEVICH_TIME_DRIVEN_MODEL_CM, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
								}
							}else {
								throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_IZHIKEVICH_TIME_DRIVEN_MODEL_EINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
							}
						}else {
							throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_IZHIKEVICH_TIME_DRIVEN_MODEL_EEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
						}
					}else {
						throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_IZHIKEVICH_TIME_DRIVEN_MODEL_D, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
					}
				}else {
					throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_IZHIKEVICH_TIME_DRIVEN_MODEL_C, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
				}
			}else {
				throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_IZHIKEVICH_TIME_DRIVEN_MODEL_B, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
			}
		}else {
			throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_IZHIKEVICH_TIME_DRIVEN_MODEL_A, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
		}

		//INTEGRATION METHOD
		loadIntegrationMethod(this->GetModelID(), fh, &Currentline);

		this->integrationMethod->SetBiFixedStepParameters(-40.0f,c+5.0f,2.0f);

		this->integrationMethod->Calculate_conductance_exp_values();

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_IZHIKEVICH_TIME_DRIVEN_MODEL_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

//this neuron model is implemented in a milisecond scale.
IzhikevichTimeDrivenModel::IzhikevichTimeDrivenModel(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel(NeuronTypeID, NeuronModelID, MilisecondScale){
		//a=0.1f;
		//b=0.23f;
		//c=-65.0f;
		//d=0.2;
		//eexc=0.0f;//mV
		//einh=-80.0f;//mV
		//cm=100;//pF
		//inv_cm=1.0f/cm;
		//texc=5.0f;//ms
		//inv_texc=1.0f/texc;
		//tinh=10.0f;//ms
		//inv_tinh=1.0f/tinh;

}

IzhikevichTimeDrivenModel::~IzhikevichTimeDrivenModel(void)
{
}

void IzhikevichTimeDrivenModel::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * IzhikevichTimeDrivenModel::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * IzhikevichTimeDrivenModel::ProcessInputSpike(Interconnection * inter, double time){
	// Add the effect of the input spike
	this->GetVectorNeuronState()->IncrementStateVariableAtCPU(inter->GetTargetNeuronModelIndex(), N_DifferentialNeuronState + inter->GetType(), inter->GetWeight());

	return 0;
}


bool IzhikevichTimeDrivenModel::UpdateState(int index, double CurrentTime){
	//NeuronState[0] --> v 
	//NeuronState[1] --> u 
	//NeuronState[2] --> gexc 
	//NeuronState[3] --> ginh 

	//Reset the number of internal spikes in this update period
	this->State->NInternalSpikeIndexs = 0;

	this->integrationMethod->NextDifferentialEquationValues();

	this->CheckValidIntegration(CurrentTime, this->integrationMethod->GetValidIntegrationVariable());

	return false;
}


enum NeuronModelOutputActivityType IzhikevichTimeDrivenModel::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType IzhikevichTimeDrivenModel::GetModelInputActivityType(){
	return INPUT_SPIKE;
}


ostream & IzhikevichTimeDrivenModel::PrintInfo(ostream & out){
	//out << "- Leaky Time-Driven Model: " << this->GetModelID() << endl;

	//out << "\tExc. Reversal Potential: " << this->eexc << "V\tInh. Reversal Potential: " << this->einh << "V\tResting potential: " << this->erest << "V" << endl;

	//out << "\tFiring threshold: " << this->vthr << "V\tMembrane capacitance: " << this->cm << "nS\tExcitatory Time Constant: " << this->texc << "s" << endl;

	//out << "\tInhibitory time constant: " << this->tinh << "s\tRefractory Period: " << this->tref << "s\tResting Conductance: " << this->grest << "nS" << endl;

	return out;
}	



void IzhikevichTimeDrivenModel::InitializeStates(int N_neurons, int OpenMPQueueIndex){
	//Initialize neural state variables.
	float Veq=(((b-5.0f)-sqrt((5.0f-b)*(5.0f-b)-22.4f))/0.08f);
	float Ueq=Veq*b;
	float initialization[] = {Veq, Ueq,0.0f,0.0f};
	State->InitializeStates(N_neurons, initialization);

	//Initialize integration method state variables.
	this->integrationMethod->InitializeStates(N_neurons, initialization);
}

void IzhikevichTimeDrivenModel::EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elpasedTimeInNeuronModelScale){
	if (NeuronState[0] > 30.0f){
		NeuronState[0] = this->c;//v
		NeuronState[1] += this->d;//u 
		State->NewFiredSpike(index);
		this->integrationMethod->resetState(index);
		this->State->InternalSpikeIndexs[this->State->NInternalSpikeIndexs] = index;
		this->State->NInternalSpikeIndexs++;
	}
}

void IzhikevichTimeDrivenModel::EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){
	float v=NeuronState[0];
	float u=NeuronState[1];
	float gexc=NeuronState[2];
	float ginh=NeuronState[3];
	
	if (v <= 30.0f){
		//V
		AuxNeuronState[0]=0.04f*v*v + 5*v + 140 - u + (gexc * (this->eexc - v) + ginh * (this->einh - v))*this->inv_cm;
		//u
		AuxNeuronState[1]=a*(b*v - u);
	}else{
		//V
		AuxNeuronState[0]=0;
		//u
		AuxNeuronState[1]=0;
	}
}

void IzhikevichTimeDrivenModel::EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){
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

void IzhikevichTimeDrivenModel::Calculate_conductance_exp_values(int index, float elapsed_time){
	//excitatory synapse.
	Set_conductance_exp_values(index, 0, exp(-elapsed_time*this->inv_texc));
	//inhibitory synapse.
	Set_conductance_exp_values(index, 1, exp(-elapsed_time*this->inv_tinh));
}


bool IzhikevichTimeDrivenModel::CheckSynapseType(Interconnection * connection){
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


void IzhikevichTimeDrivenModel::loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException){
	char ident_type[MAXIDSIZE + 1];

	//We load the integration method type.
	skip_comments(fh, *Currentline);
	if (fscanf(fh, "%s", ident_type) == 1){
		skip_comments(fh, *Currentline);
		//DEFINE HERE NEW INTEGRATION METHOD
		if (strncmp(ident_type, "Euler", 5) == 0){
			integrationMethod = (Euler<IzhikevichTimeDrivenModel> *) new Euler<IzhikevichTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "RK2", 3) == 0){
			integrationMethod = (RK2<IzhikevichTimeDrivenModel> *) new RK2<IzhikevichTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "RK4", 3) == 0){
			integrationMethod = (RK4<IzhikevichTimeDrivenModel> *) new RK4<IzhikevichTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "BDF", 3) == 0 && atoi(&ident_type[3])>0 && atoi(&ident_type[3])<7){
			integrationMethod = (BDFn<IzhikevichTimeDrivenModel> *) new BDFn<IzhikevichTimeDrivenModel>(this, atoi(&ident_type[3]));
		}
		else if (strncmp(ident_type, "Bifixed_Euler", 13) == 0){
			integrationMethod = (Bifixed_Euler<IzhikevichTimeDrivenModel> *) new Bifixed_Euler<IzhikevichTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK2", 11) == 0){
			integrationMethod = (Bifixed_RK2<IzhikevichTimeDrivenModel> *) new Bifixed_RK2<IzhikevichTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK4", 11) == 0){
			integrationMethod = (Bifixed_RK4<IzhikevichTimeDrivenModel> *) new Bifixed_RK4<IzhikevichTimeDrivenModel>(this);
		}
		else if (strncmp(ident_type, "Bifixed_BDF", 11) == 0 && atoi(&ident_type[11]) == 2){
			integrationMethod = (Bifixed_BDFn<IzhikevichTimeDrivenModel> *) new Bifixed_BDFn<IzhikevichTimeDrivenModel>(this, atoi(&ident_type[11]));
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



