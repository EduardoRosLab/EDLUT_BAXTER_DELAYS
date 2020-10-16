/***************************************************************************
 *                           TimeDrivenPurkinjeCell.cpp                    *
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

#include "../../include/neuron_model/TimeDrivenPurkinjeCell.h"
#include "../../include/neuron_model/VectorNeuronState.h"

#include <iostream>
#include <cmath>
#include <string>
#include <cfloat> 

#include "../../include/openmp/openmp.h"

#include "../../include/spike/EDLUTFileException.h"
#include "../../include/spike/Neuron.h"
#include "../../include/spike/InternalSpike.h"
#include "../../include/spike/PropagatedSpike.h"
#include "../../include/spike/Interconnection.h"

#include "../../include/simulation/Utils.h"

#include "../../include/openmp/openmp.h"


const float TimeDrivenPurkinjeCell::Max_V=35.0f;
const float TimeDrivenPurkinjeCell::Min_V=-100.0f;

const float TimeDrivenPurkinjeCell::aux=(TimeDrivenPurkinjeCell::TableSize-1)/( TimeDrivenPurkinjeCell::Max_V- TimeDrivenPurkinjeCell::Min_V);

float * TimeDrivenPurkinjeCell::channel_values=Generate_channel_values();


void TimeDrivenPurkinjeCell::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
	FILE *fh;
	long Currentline = 0L;
	fh=fopen(ConfigFile.c_str(),"rt");
	if(fh){
		this->State = (VectorNeuronState *) new VectorNeuronState(N_NeuronStateVariables, true);

		//INTEGRATION METHOD
		loadIntegrationMethod(this->GetModelID(), fh, &Currentline);

		this->integrationMethod->SetBiFixedStepParameters((erest+3*vthr)/4, (erest+3*vthr)/4, 2.0f);

		this->integrationMethod->Calculate_conductance_exp_values();

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_TIME_DRIVEN_PURKINJE_CELL_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

//this neuron model is implemented in a milisecond scale.
TimeDrivenPurkinjeCell::TimeDrivenPurkinjeCell(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel(NeuronTypeID, NeuronModelID, MilisecondScale), g_L(0.02f),
		g_Ca(0.001f), g_M(0.75f), Cylinder_length_of_the_soma(0.0015f), Radius_of_the_soma(0.0008f), Area(3.141592f*0.0015f*2.0f*0.0008f),
		inv_Area(1.0f/(3.141592f*0.0015f*2.0f*0.0008f)), Membrane_capacitance(0.95f), inv_Membrane_capacitance(1.0f/0.95f)
		{
	eexc=0.0f;
	einh=-80.0f ;
	vthr= -35.0f;
	erest=-70.0f;
	texc=1.0f;
	inv_texc=1.0f/texc;
	tinh=2;
	inv_tinh=1.0f/tinh;
	tref=1.35f;
	tref_0_5=tref*0.5f;
	inv_tref_0_5=1.0f/tref_0_5;
	spkpeak=31.0f;
}

TimeDrivenPurkinjeCell::~TimeDrivenPurkinjeCell(void)
{
	if(this->channel_values){
		delete this->channel_values;
		this->channel_values=0;
	}
}

void TimeDrivenPurkinjeCell::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * TimeDrivenPurkinjeCell::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * TimeDrivenPurkinjeCell::ProcessInputSpike(Interconnection * inter, double time){
	// Add the effect of the input spike
	this->GetVectorNeuronState()->IncrementStateVariableAtCPU(inter->GetTargetNeuronModelIndex(), N_DifferentialNeuronState + inter->GetType(), 1e-6f*inter->GetWeight());

	return 0;
}


bool TimeDrivenPurkinjeCell::UpdateState(int index, double CurrentTime){
	//NeuronState[0] --> V
	//NeuronState[1] --> c
	//NeuronState[2] --> M
	//NeuronState[3] --> gexc 
	//NeuronState[4] --> ginh 

	//Reset the number of internal spikes in this update period
	this->State->NInternalSpikeIndexs = 0;

	this->integrationMethod->NextDifferentialEquationValues();

	this->CheckValidIntegration(CurrentTime, this->integrationMethod->GetValidIntegrationVariable());

	return false;
}


enum NeuronModelOutputActivityType TimeDrivenPurkinjeCell::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType TimeDrivenPurkinjeCell::GetModelInputActivityType(){
	return INPUT_SPIKE;
}


ostream & TimeDrivenPurkinjeCell::PrintInfo(ostream & out){
	//out << "- Leaky Time-Driven Model: " << this->GetModelID() << endl;

	//out << "\tExc. Reversal Potential: " << this->eexc << "V\tInh. Reversal Potential: " << this->einh << "V\tResting potential: " << this->erest << "V" << endl;

	//out << "\tFiring threshold: " << this->vthr << "V\tMembrane capacitance: " << this->cm << "nS\tExcitatory Time Constant: " << this->texc << "s" << endl;

	//out << "\tInhibitory time constant: " << this->tinh << "s\tRefractory Period: " << this->tref << "s\tResting Conductance: " << this->grest << "nS" << endl;

	return out;
}	



void TimeDrivenPurkinjeCell::InitializeStates(int N_neurons, int OpenMPQueueIndex){
	//Initialize neural state variables.
	float * values=Get_channel_values(erest);
	float alpha_ca=values[0];
	float inv_tau_ca=values[1];
	float alpha_M=values[2];
	float inv_tau_M=values[3];

	//c_inf
	float c_inf=alpha_ca/inv_tau_ca;

	//M_inf
	float M_inf=alpha_M/inv_tau_M;

	float initialization[] = {erest,c_inf,M_inf,0.0f,0.0f};
	State->InitializeStates(N_neurons, initialization);

	//Initialize integration method state variables.
	this->integrationMethod->InitializeStates(N_neurons, initialization);
}


void TimeDrivenPurkinjeCell::EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
	bool spike=false;
	if (NeuronState[0] >= this->vthr && previous_V < this->vthr){
		State->NewFiredSpike(index);
		this->State->InternalSpikeIndexs[this->State->NInternalSpikeIndexs] = index;
		this->State->NInternalSpikeIndexs++;
	}

	double last_spike = State->GetLastSpikeTime(index) * this->timeScale;

	if(last_spike < tref){
		if(last_spike <= tref_0_5){
			NeuronState[0]=vthr+(spkpeak-vthr)*(last_spike*inv_tref_0_5);
		}else{
			NeuronState[0]=spkpeak-(spkpeak-erest)*((last_spike-tref_0_5)*inv_tref_0_5);
		}
	}else if((last_spike - tref)<elapsedTimeInNeuronModelScale){
		NeuronState[0]=erest;
	}
}


void TimeDrivenPurkinjeCell::EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){
	float V=NeuronState[0];
	float ca=NeuronState[1];
	float M=NeuronState[2];
	float g_exc=NeuronState[3];
	float g_inh=NeuronState[4];
	float last_spike=this->timeScale*State->GetLastSpikeTime(index);

	//V
	if(last_spike >= tref){
		AuxNeuronState[0]=(-g_L*(V+70.0f)-g_Ca*ca*ca*(V-125.0f)-g_M*M*(V+95.0f) + (g_exc * (this->eexc - V) + g_inh * (this->einh - V))*inv_Area )*inv_Membrane_capacitance;
	}else if(last_spike <= tref_0_5){
		AuxNeuronState[0]=(spkpeak-vthr)*inv_tref_0_5;
	}else{
		AuxNeuronState[0]=(erest-spkpeak)*inv_tref_0_5;
	}

	float * values=Get_channel_values(V);

	//ca
	float alpha_ca=values[0];
	float inv_tau_ca=values[1];
	AuxNeuronState[1]=alpha_ca - ca*inv_tau_ca;
	
	//M	
	float alpha_M=values[2];
	float inv_tau_M=values[3];
	AuxNeuronState[2]=alpha_M - M*inv_tau_M;


}

void TimeDrivenPurkinjeCell::EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){
	float limit=1e-15;
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

void TimeDrivenPurkinjeCell::Calculate_conductance_exp_values(int index, float elapsed_time){
	//excitatory synapse.
	Set_conductance_exp_values(index, 0, exp(-elapsed_time*this->inv_texc));
	//inhibitory synapse.
	Set_conductance_exp_values(index, 1, exp(-elapsed_time*this->inv_tinh));
}


bool TimeDrivenPurkinjeCell::CheckSynapseType(Interconnection * connection){
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




void TimeDrivenPurkinjeCell::loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException){
	char ident_type[MAXIDSIZE + 1];

	//We load the integration method type.
	skip_comments(fh, *Currentline);
	if (fscanf(fh, "%s", ident_type) == 1){
		skip_comments(fh, *Currentline);
		//DEFINE HERE NEW INTEGRATION METHOD
		if (strncmp(ident_type, "Euler", 5) == 0){
			integrationMethod = (Euler<TimeDrivenPurkinjeCell> *) new Euler<TimeDrivenPurkinjeCell>(this);
		}
		else if (strncmp(ident_type, "RK2", 3) == 0){
			integrationMethod = (RK2<TimeDrivenPurkinjeCell> *) new RK2<TimeDrivenPurkinjeCell>(this);
		}
		else if (strncmp(ident_type, "RK4", 3) == 0){
			integrationMethod = (RK4<TimeDrivenPurkinjeCell> *) new RK4<TimeDrivenPurkinjeCell>(this);
		}
		else if (strncmp(ident_type, "BDF", 3) == 0 && atoi(&ident_type[3])>0 && atoi(&ident_type[3])<7){
			integrationMethod = (BDFn<TimeDrivenPurkinjeCell> *) new BDFn<TimeDrivenPurkinjeCell>(this, atoi(&ident_type[3]));
		}
		else if (strncmp(ident_type, "Bifixed_Euler", 13) == 0){
			integrationMethod = (Bifixed_Euler<TimeDrivenPurkinjeCell> *) new Bifixed_Euler<TimeDrivenPurkinjeCell>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK2", 11) == 0){
			integrationMethod = (Bifixed_RK2<TimeDrivenPurkinjeCell> *) new Bifixed_RK2<TimeDrivenPurkinjeCell>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK4", 11) == 0){
			integrationMethod = (Bifixed_RK4<TimeDrivenPurkinjeCell> *) new Bifixed_RK4<TimeDrivenPurkinjeCell>(this);
		}
		else if (strncmp(ident_type, "Bifixed_BDF", 11) == 0 && atoi(&ident_type[11]) == 2){
			integrationMethod = (Bifixed_BDFn<TimeDrivenPurkinjeCell> *) new Bifixed_BDFn<TimeDrivenPurkinjeCell>(this, atoi(&ident_type[11]));
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


