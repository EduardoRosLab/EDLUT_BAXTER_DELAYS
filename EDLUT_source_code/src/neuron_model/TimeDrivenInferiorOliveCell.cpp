/***************************************************************************
 *                           TimeDrivenInferiorOliveCell.cpp               *
 *                           -------------------                           *
 * copyright            : (C) 2016 by Niceto Luque and Francisco Naveros   *
 * email                : nluque@ugr.es and	fnaveros@ugr.es  			   *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "../../include/neuron_model/TimeDrivenInferiorOliveCell.h"
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
#include "../../include/simulation/ExponentialTable.h"

#include "../../include/openmp/openmp.h"



void TimeDrivenInferiorOliveCell::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
	FILE *fh;
	long Currentline = 0L;
	fh=fopen(ConfigFile.c_str(),"rt");
	if(fh){
		this->State = (VectorNeuronState *) new VectorNeuronState(N_NeuronStateVariables, true);

		//INTEGRATION METHOD
		loadIntegrationMethod(this->GetModelID(), fh, &Currentline);

		this->integrationMethod->SetBiFixedStepParameters((erest+vthr)/2,erest+5,0.0);

		this->integrationMethod->Calculate_conductance_exp_values();

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_TIME_DRIVEN_INFERIOR_OLIVE_CELL_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

//this neuron model is implemented in a milisecond scale.
TimeDrivenInferiorOliveCell::TimeDrivenInferiorOliveCell(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel(NeuronTypeID, NeuronModelID, MilisecondScale), g_L(0.015f),
Membrane_capacitance(1.0f), inv_Membrane_capacitance(1.0f / 1.0f), Area(0.00001f), inv_Area(1.0f / 0.00001f)
		{
	eexc=0.0f;
	einh=-80.0f ;
	vthr= -50.0f;
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

TimeDrivenInferiorOliveCell::~TimeDrivenInferiorOliveCell(void)
{
	for (int i = 0; i < this->GetVectorNeuronState()->GetSizeState(); i++){
		if (N_coupling_synapses[i]>0){
			delete Weight_coupling_synapses[i];
			delete Potential_coupling_synapses[i];
		}
	}
	delete N_coupling_synapses;
	delete index_coupling_synapses;
	delete Weight_coupling_synapses;
	delete Potential_coupling_synapses;
}

void TimeDrivenInferiorOliveCell::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}

VectorNeuronState * TimeDrivenInferiorOliveCell::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * TimeDrivenInferiorOliveCell::ProcessInputSpike(Interconnection * inter, double time){
	// Add the effect of the input spike
	if (inter->GetType()<2){
		this->GetVectorNeuronState()->IncrementStateVariableAtCPU(inter->GetTargetNeuronModelIndex(), N_DifferentialNeuronState + inter->GetType(), 1e-6f*inter->GetWeight());
	}

	return 0;
}


bool TimeDrivenInferiorOliveCell::UpdateState(int index, double CurrentTime){
	//NeuronState[0] --> V
	//NeuronState[1] --> gexc 
	//NeuronState[2] --> ginh 
	//NeuronState[3] --> Ic 

	////ELECTRICAL COUPLING CURRENTS.
	#pragma omp barrier
		for (int i = 0; i<this->GetVectorNeuronState()->GetSizeState(); i++){
			float current = 0;
			float * NeuronState = this->GetVectorNeuronState()->GetStateVariableAt(i);
			for (int j = 0; j<this->N_coupling_synapses[i]; j++){
				float source_V = Potential_coupling_synapses[i][j][0];
				float V = NeuronState[0] - source_V;
				float weight = Weight_coupling_synapses[i][j];
				//			current += weight*V*(0.6f*exp(-V*V * 0.0004) + 0.4);                                     //current += weight*V*(0.6f*exp(-V*V / (50.0f*50.0f)) + 0.4);
				current += weight*V*(0.6f*ExponentialTable::GetResult(-V*V * 0.0004) + 0.4);      //current += weight*V*(0.6f*ExponentialTable::GetResult(-V*V / (50.0f*50.0f)) + 0.4);
			}
			NeuronState[3] = 1e-6f*current;
		}
	#pragma omp barrier
	////////////////////////////////////////

	//Reset the number of internal spikes in this update period
	this->State->NInternalSpikeIndexs = 0;

	this->integrationMethod->NextDifferentialEquationValues();

	this->CheckValidIntegration(CurrentTime, this->integrationMethod->GetValidIntegrationVariable());

	return false;
}




enum NeuronModelOutputActivityType TimeDrivenInferiorOliveCell::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType TimeDrivenInferiorOliveCell::GetModelInputActivityType(){
	return INPUT_SPIKE;
}


ostream & TimeDrivenInferiorOliveCell::PrintInfo(ostream & out){
	//out << "- Leaky Time-Driven Model: " << this->GetModelID() << endl;

	//out << "\tExc. Reversal Potential: " << this->eexc << "V\tInh. Reversal Potential: " << this->einh << "V\tResting potential: " << this->erest << "V" << endl;

	//out << "\tFiring threshold: " << this->vthr << "V\tMembrane capacitance: " << this->cm << "nS\tExcitatory Time Constant: " << this->texc << "s" << endl;

	//out << "\tInhibitory time constant: " << this->tinh << "s\tRefractory Period: " << this->tref << "s\tResting Conductance: " << this->grest << "nS" << endl;

	return out;
}	



void TimeDrivenInferiorOliveCell::InitializeStates(int N_neurons, int OpenMPQueueIndex){
	//Initialize neural state variables.
	float initialization[] = {erest,0.0f,0.0f,0.0f};
	State->InitializeStates(N_neurons, initialization);

	//Initialize integration method state variables.
	this->integrationMethod->InitializeStates(N_neurons, initialization);

	N_coupling_synapses=new int[N_neurons]();
	index_coupling_synapses = new int[N_neurons]();
    Weight_coupling_synapses=(float**)new float*[N_neurons];
    Potential_coupling_synapses=(float***)new float**[N_neurons];
}


void TimeDrivenInferiorOliveCell::EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
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


void TimeDrivenInferiorOliveCell::EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){
	float V=NeuronState[0];
	float g_exc=NeuronState[1];
	float g_inh=NeuronState[2];
	float Ic=NeuronState[3];
	float last_spike=this->timeScale*State->GetLastSpikeTime(index);

	//V
	if(last_spike >= tref){
		AuxNeuronState[0]=(-g_L*(V+70.0f) - (g_exc * (V - this->eexc) + g_inh * (V - this->einh) + Ic)*inv_Area )*inv_Membrane_capacitance;
	}else if(last_spike <= tref_0_5){
		AuxNeuronState[0]=(spkpeak-vthr)*inv_tref_0_5;
	}else{
		AuxNeuronState[0]=(erest-spkpeak)*inv_tref_0_5;
	}
}

void TimeDrivenInferiorOliveCell::EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){
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

void TimeDrivenInferiorOliveCell::Calculate_conductance_exp_values(int index, float elapsed_time){
	//excitatory synapse.
	Set_conductance_exp_values(index, 0, exp(-elapsed_time*this->inv_texc));
	//inhibitory synapse.
	Set_conductance_exp_values(index, 1, exp(-elapsed_time*this->inv_tinh));
}


bool TimeDrivenInferiorOliveCell::CheckSynapseType(Interconnection * connection){
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

void TimeDrivenInferiorOliveCell::CalculateElectricalCouplingSynapseNumber(Interconnection * inter){
	if (inter->GetType() == 2){
		int TargetIndex = inter->GetTarget()->GetIndex_VectorNeuronState();
		N_coupling_synapses[TargetIndex]++;
	}
}



void TimeDrivenInferiorOliveCell::InitializeElectricalCouplingSynapseDependencies(){
	for (int i = 0; i<this->GetVectorNeuronState()->GetSizeState(); i++){
		if (N_coupling_synapses[i] > 0){
			Weight_coupling_synapses[i] = new float[N_coupling_synapses[i]];
			Potential_coupling_synapses[i] = (float**)new float*[N_coupling_synapses[i]];
		}
		else{
			Weight_coupling_synapses[i] = 0;
			Potential_coupling_synapses[i] = 0;
		}
	}
}


void TimeDrivenInferiorOliveCell::CalculateElectricalCouplingSynapseDependencies(Interconnection * inter){
	if (inter->GetType() == 2){
		int TargetIndex = inter->GetTarget()->GetIndex_VectorNeuronState();
		Weight_coupling_synapses[TargetIndex][index_coupling_synapses[TargetIndex]] = inter->GetWeight();
		Potential_coupling_synapses[TargetIndex][index_coupling_synapses[TargetIndex]] = inter->GetSource()->GetNeuronModel()->GetVectorNeuronState()->GetStateVariableAt(inter->GetSource()->GetIndex_VectorNeuronState());
		index_coupling_synapses[TargetIndex]++;
	}
}



void TimeDrivenInferiorOliveCell::loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException){
	char ident_type[MAXIDSIZE + 1];

	//We load the integration method type.
	skip_comments(fh, *Currentline);
	if (fscanf(fh, "%s", ident_type) == 1){
		skip_comments(fh, *Currentline);
		//DEFINE HERE NEW INTEGRATION METHOD
		if (strncmp(ident_type, "Euler", 5) == 0){
			integrationMethod = (Euler<TimeDrivenInferiorOliveCell> *) new Euler<TimeDrivenInferiorOliveCell>(this);
		}
		else if (strncmp(ident_type, "RK2", 3) == 0){
			integrationMethod = (RK2<TimeDrivenInferiorOliveCell> *) new RK2<TimeDrivenInferiorOliveCell>(this);
		}
		else if (strncmp(ident_type, "RK4", 3) == 0){
			integrationMethod = (RK4<TimeDrivenInferiorOliveCell> *) new RK4<TimeDrivenInferiorOliveCell>(this);
		}
		else if (strncmp(ident_type, "BDF", 3) == 0 && atoi(&ident_type[3])>0 && atoi(&ident_type[3])<7){
			integrationMethod = (BDFn<TimeDrivenInferiorOliveCell> *) new BDFn<TimeDrivenInferiorOliveCell>(this, atoi(&ident_type[3]));
		}
		else if (strncmp(ident_type, "Bifixed_Euler", 13) == 0){
			integrationMethod = (Bifixed_Euler<TimeDrivenInferiorOliveCell> *) new Bifixed_Euler<TimeDrivenInferiorOliveCell>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK2", 11) == 0){
			integrationMethod = (Bifixed_RK2<TimeDrivenInferiorOliveCell> *) new Bifixed_RK2<TimeDrivenInferiorOliveCell>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK4", 11) == 0){
			integrationMethod = (Bifixed_RK4<TimeDrivenInferiorOliveCell> *) new Bifixed_RK4<TimeDrivenInferiorOliveCell>(this);
		}
		else if (strncmp(ident_type, "Bifixed_BDF", 11) == 0 && atoi(&ident_type[11]) == 2){
			integrationMethod = (Bifixed_BDFn<TimeDrivenInferiorOliveCell> *) new Bifixed_BDFn<TimeDrivenInferiorOliveCell>(this, atoi(&ident_type[11]));
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




