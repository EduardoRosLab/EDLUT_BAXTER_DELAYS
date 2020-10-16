/***************************************************************************
 *                           LIFTimeDrivenModel_1_3_NEW.cpp                *
 *                           -------------------                           *
 * copyright            : (C) 2018 by Francisco Naveros                    *
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

#include "../../include/neuron_model/LIFTimeDrivenModel_1_3_NEW.h"
#include "../../include/neuron_model/VectorNeuronState.h"
#include "../../include/neuron_model/CurrentSynapseModel.h"


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


void LIFTimeDrivenModel_1_3_NEW::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
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

						if (fscanf(fh, "%f", &this->cm) == 1 && this->cm > 0.0f){
							inv_cm=1.0f/cm;
							skip_comments(fh,Currentline);

							if (fscanf(fh, "%f", &this->texc) == 1 && this->texc > 0.0f){
								inv_texc=1.0f/texc;
								skip_comments(fh,Currentline);

								if(fscanf(fh,"%f",&this->tinh)==1 && this->tinh > 0.0f){
									inv_tinh=1.0f/tinh;
									skip_comments(fh,Currentline);

									if (fscanf(fh, "%f", &this->tref) == 1 && this->tref >= 0.0f){
										skip_comments(fh,Currentline);

										if(fscanf(fh,"%f",&this->grest)==1 && this->grest > 0.0f){
											skip_comments(fh,Currentline);

											this->State = (VectorNeuronState *) new VectorNeuronState(N_NeuronStateVariables, true);
										} else {
											throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_NEW_GREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
										}
									} else {
										throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_NEW_TREF, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
									}
								} else {
									throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_NEW_TINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
								}
							} else {
								throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_NEW_TEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
							}
						} else {
							throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_NEW_CM, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
						}
					} else {
						throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_NEW_VTHR, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
					}
				} else {
					throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_NEW_EREST, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
				}
			} else {
				throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_NEW_EINH, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
			}
		} else {
			throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_LIF_TIME_DRIVEN_MODEL_1_2_NEW_EEXC, REPAIR_NEURON_MODEL_VALUES, Currentline, ConfigFile.c_str(), true);
		}
	
		//INTEGRATION METHOD
		loadIntegrationMethod(this->GetModelID(), fh, &Currentline);

		this->integrationMethod->SetBiFixedStepParameters((erest+vthr)/2,(erest+vthr)/2,0);
		this->integrationMethod->Calculate_conductance_exp_values();

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_LIF_TIME_DRIVEN_MODEL_1_3_NEW_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

//this neuron model is implemented in a second scale.
LIFTimeDrivenModel_1_3_NEW::LIFTimeDrivenModel_1_3_NEW(string NeuronTypeID, string NeuronModelID) : TimeDrivenNeuronModel(NeuronTypeID, NeuronModelID, MilisecondScale), eexc(0), einh(0), erest(0), vthr(0), cm(0), texc(0), tinh(0),
		tref(0), grest(0){
}

LIFTimeDrivenModel_1_3_NEW::~LIFTimeDrivenModel_1_3_NEW(void)
{
}

void LIFTimeDrivenModel_1_3_NEW::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * LIFTimeDrivenModel_1_3_NEW::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * LIFTimeDrivenModel_1_3_NEW::ProcessInputSpike(Interconnection * inter, double time){
	// Add the effect of the input spike
	this->GetVectorNeuronState()->IncrementStateVariableAtCPU(inter->GetTargetNeuronModelIndex(), N_DifferentialNeuronState + inter->GetType(), inter->GetWeight());

	return 0;
}


bool LIFTimeDrivenModel_1_3_NEW::UpdateState(int index, double CurrentTime){
	//NeuronState[0] --> v 
	//NeuronState[1] --> gexc 
	//NeuronState[2] --> ginh 
	//NeuronState[3] --> current

	//Reset the number of internal spikes in this update period
	this->State->NInternalSpikeIndexs = 0;

	for (int i = 0; i < State->GetSizeState(); i++){
		float * NeuronState = State->GetStateVariableAt(i);
		//update input current value;
		NeuronState[N_DifferentialNeuronState + N_TimeDependentNeuronState - 1] = this->CurrentSynapses->GetTotalCurrent(i);
	}

	this->integrationMethod->NextDifferentialEquationValues();

	this->CheckValidIntegration(CurrentTime, this->integrationMethod->GetValidIntegrationVariable());

	return false;
}


enum NeuronModelOutputActivityType LIFTimeDrivenModel_1_3_NEW::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType LIFTimeDrivenModel_1_3_NEW::GetModelInputActivityType(){
	return INPUT_SPIKE_AND_CURRENT;
}


ostream & LIFTimeDrivenModel_1_3_NEW::PrintInfo(ostream & out){
	out << "- Leaky Time-Driven Model: " << this->GetModelID() << endl;

	out << "\tExc. Reversal Potential: " << this->eexc << "mV\tInh. Reversal Potential: " << this->einh << "mV\tResting potential: " << this->erest << "mV" << endl;

	out << "\tFiring threshold: " << this->vthr << "mV\tMembrane capacitance: " << this->cm << "pF\tExcitatory Time Constant: " << this->texc << "ms" << endl;

	out << "\tInhibitory time constant: " << this->tinh << "ms\tRefractory Period: " << this->tref << "ms\tResting Conductance: " << this->grest << "nS" << endl;

	return out;
}	



void LIFTimeDrivenModel_1_3_NEW::InitializeStates(int N_neurons, int OpenMPQueueIndex){
	//Initialize neural state variables.
	float initialization[] = {erest,0.0f,0.0f,0.0f};
	State->InitializeStates(N_neurons, initialization);

	//Initialize integration method state variables.
	this->integrationMethod->InitializeStates(N_neurons, initialization);

	//Initialize the array that stores the number of input current synapses for each neuron in the model
	this->CurrentSynapses = new CurrentSynapseModel(N_neurons);	
}


void LIFTimeDrivenModel_1_3_NEW::EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
	if (NeuronState[0] > this->vthr){
		NeuronState[0] = this->erest;
		State->NewFiredSpike(index);
		this->integrationMethod->resetState(index);
		this->State->InternalSpikeIndexs[this->State->NInternalSpikeIndexs] = index;
		this->State->NInternalSpikeIndexs++;
	}
}


void LIFTimeDrivenModel_1_3_NEW::EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){
	if(this->GetVectorNeuronState()->GetLastSpikeTime(index) * this->GetTimeScale() >this->tref){
		AuxNeuronState[0]=(NeuronState[1] * (this->eexc - NeuronState[0]) + NeuronState[2] * (this->einh - NeuronState[0]) + grest * (this->erest - NeuronState[0]) + NeuronState[3])*this->inv_cm;
	}else if ((this->GetVectorNeuronState()->GetLastSpikeTime(index) * this->GetTimeScale() + elapsed_time)>this->tref){
		float fraction = (this->GetVectorNeuronState()->GetLastSpikeTime(index) * this->GetTimeScale() + elapsed_time - this->tref) / elapsed_time;
		AuxNeuronState[0]=fraction*(NeuronState[1] * (this->eexc - NeuronState[0]) + NeuronState[2] * (this->einh - NeuronState[0]) + grest * (this->erest - NeuronState[0]) + NeuronState[3])*this->inv_cm;
	}else{
		AuxNeuronState[0]=0;
	}
}

void LIFTimeDrivenModel_1_3_NEW::EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){
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


void LIFTimeDrivenModel_1_3_NEW::Calculate_conductance_exp_values(int index, float elapsed_time){
	//excitatory synapse.
	Set_conductance_exp_values(index, 0, expf(-elapsed_time*this->inv_texc));
	//inhibitory synapse.
	Set_conductance_exp_values(index, 1, expf(-elapsed_time*this->inv_tinh));
}


bool LIFTimeDrivenModel_1_3_NEW::CheckSynapseType(Interconnection * connection){
	int Type = connection->GetType();

	if (Type < N_TimeDependentNeuronState && Type >= 0){
		NeuronModel * model = connection->GetSource()->GetNeuronModel();
		//Synapse types that process input spikes 
		if (Type < N_TimeDependentNeuronState - 1){
			if (model->GetModelOutputActivityType() == OUTPUT_SPIKE){
				return true;
			}
			else{
				cout << "Synapses type " << Type << " of neuron model " << this->GetTypeID() << ", " << this->GetModelID() << " must receive spikes. The source model generates currents." << endl;
				return false;
			}
		}
		//Synapse types that process input current 
		if (Type == N_TimeDependentNeuronState - 1){
			if (model->GetModelOutputActivityType() == OUTPUT_CURRENT){
				connection->SetSubindexType(this->CurrentSynapses->GetNInputCurrentSynapsesPerNeuron(connection->GetTarget()->GetIndex_VectorNeuronState()));
				this->CurrentSynapses->IncrementNInputCurrentSynapsesPerNeuron(connection->GetTarget()->GetIndex_VectorNeuronState());
				return true;
			}
			else{
				cout << "Synapses type " << Type << " of neuron model " << this->GetTypeID() << ", " << this->GetModelID() << " must receive current. The source model generates spikes." << endl;
				return false;
			}
		}
	}
	else{
		cout << "Neuron model " << this->GetTypeID() << ", " << this->GetModelID() << " does not support input synapses of type " << Type << ". Just defined " << N_TimeDependentNeuronState << " synapses types." << endl;
		return false;
	}
}



void LIFTimeDrivenModel_1_3_NEW::loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException){
	char ident_type[MAXIDSIZE + 1];

	//We load the integration method type.
	skip_comments(fh, *Currentline);
	if (fscanf(fh, "%s", ident_type) == 1){
		skip_comments(fh, *Currentline);
		//DEFINE HERE NEW INTEGRATION METHOD
		if (strncmp(ident_type, "Euler", 5) == 0){
			integrationMethod = (Euler<LIFTimeDrivenModel_1_3_NEW> *) new Euler<LIFTimeDrivenModel_1_3_NEW>(this);
		}
		else if (strncmp(ident_type, "RK2", 3) == 0){
			integrationMethod = (RK2<LIFTimeDrivenModel_1_3_NEW> *) new RK2<LIFTimeDrivenModel_1_3_NEW>(this);
		}
		else if (strncmp(ident_type, "RK4", 3) == 0){
			integrationMethod = (RK4<LIFTimeDrivenModel_1_3_NEW> *) new RK4<LIFTimeDrivenModel_1_3_NEW>(this);
		}
		else if (strncmp(ident_type, "BDF", 3) == 0 && atoi(&ident_type[3])>0 && atoi(&ident_type[3])<7){
			integrationMethod = (BDFn<LIFTimeDrivenModel_1_3_NEW> *) new BDFn<LIFTimeDrivenModel_1_3_NEW>(this, atoi(&ident_type[3]));
		}
		else if (strncmp(ident_type, "Bifixed_Euler", 13) == 0){
			integrationMethod = (Bifixed_Euler<LIFTimeDrivenModel_1_3_NEW> *) new Bifixed_Euler<LIFTimeDrivenModel_1_3_NEW>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK2", 11) == 0){
			integrationMethod = (Bifixed_RK2<LIFTimeDrivenModel_1_3_NEW> *) new Bifixed_RK2<LIFTimeDrivenModel_1_3_NEW>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK4", 11) == 0){
			integrationMethod = (Bifixed_RK4<LIFTimeDrivenModel_1_3_NEW> *) new Bifixed_RK4<LIFTimeDrivenModel_1_3_NEW>(this);
		}
		else if (strncmp(ident_type, "Bifixed_BDF", 11) == 0 && atoi(&ident_type[11]) == 2){
			integrationMethod = (Bifixed_BDFn<LIFTimeDrivenModel_1_3_NEW> *) new Bifixed_BDFn<LIFTimeDrivenModel_1_3_NEW>(this, atoi(&ident_type[11]));
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
