/***************************************************************************
 *                           EdidioGranuleCell_TimeDriven.cpp              *
 *                           -------------------                           *
 * copyright            : (C) 2013 by Francisco Naveros                    *
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

#include "../../include/neuron_model/EgidioGranuleCell_TimeDriven.h"
#include "../../include/neuron_model/VectorNeuronState.h"

#include <iostream>
#include <cmath>
#include <string>

#include "../../include/openmp/openmp.h"

//This neuron model is implemented in milisecond. EDLUT is implemented in second and it is necesary to
//use this constant in order to adapt this model to EDLUT.


#include "../../include/spike/EDLUTFileException.h"
#include "../../include/spike/Neuron.h"
#include "../../include/spike/InternalSpike.h"
#include "../../include/spike/PropagatedSpike.h"
#include "../../include/spike/Interconnection.h"

#include "../../include/simulation/Utils.h"


	const float EgidioGranuleCell_TimeDriven::gMAXNa_f=0.013f;
	const float EgidioGranuleCell_TimeDriven::gMAXNa_r=0.0005f;
	const float EgidioGranuleCell_TimeDriven::gMAXNa_p=0.0002f;
	const float EgidioGranuleCell_TimeDriven::gMAXK_V=0.003f;
	const float EgidioGranuleCell_TimeDriven::gMAXK_A=0.004f;
	const float EgidioGranuleCell_TimeDriven::gMAXK_IR=0.0009f;
	const float EgidioGranuleCell_TimeDriven::gMAXK_Ca=0.004f;
	const float EgidioGranuleCell_TimeDriven::gMAXCa=0.00046f;
	const float EgidioGranuleCell_TimeDriven::gMAXK_sl=0.00035f;
	const float EgidioGranuleCell_TimeDriven::gLkg1=5.68e-5f;
	const float EgidioGranuleCell_TimeDriven::gLkg2=2.17e-5f;
	const float EgidioGranuleCell_TimeDriven::VNa=87.39f;
	const float EgidioGranuleCell_TimeDriven::VK=-84.69f;
	const float EgidioGranuleCell_TimeDriven::VLkg1=-58.0f;
	const float EgidioGranuleCell_TimeDriven::VLkg2=-65.0f;
	const float EgidioGranuleCell_TimeDriven::V0_xK_Ai=-46.7f;
	const float EgidioGranuleCell_TimeDriven::K_xK_Ai=-19.8f;
	const float EgidioGranuleCell_TimeDriven::V0_yK_Ai=-78.8f;
	const float EgidioGranuleCell_TimeDriven::K_yK_Ai=8.4f;
	const float EgidioGranuleCell_TimeDriven::V0_xK_sli=-30.0f;
	const float EgidioGranuleCell_TimeDriven::B_xK_sli=6.0f;
	const float EgidioGranuleCell_TimeDriven::F=96485.309f;
	const float EgidioGranuleCell_TimeDriven::A=1e-04f;
	const float EgidioGranuleCell_TimeDriven::d=0.2f;
	const float EgidioGranuleCell_TimeDriven::betaCa=1.5f;
	const float EgidioGranuleCell_TimeDriven::Ca0=1e-04f;
	const float EgidioGranuleCell_TimeDriven::R=8.3134f;
	const float EgidioGranuleCell_TimeDriven::cao=2.0f;
	const float EgidioGranuleCell_TimeDriven::Cm = 1.0e-3f;
	const float EgidioGranuleCell_TimeDriven::temper=30.0f;
	const float EgidioGranuleCell_TimeDriven::Q10_20 = pow(3.0f,((temper-20.0f)/10.0f));
	const float EgidioGranuleCell_TimeDriven::Q10_22 = pow(3.0f,((temper-22.0f)/10.0f));
	const float EgidioGranuleCell_TimeDriven::Q10_30 = pow(3.0f,((temper-30.0f)/10.0f));
	const float EgidioGranuleCell_TimeDriven::Q10_6_3 = pow(3.0f,((temper-6.3f)/10.0f));

	const float EgidioGranuleCell_TimeDriven::Max_V=50.0f;
	const float EgidioGranuleCell_TimeDriven::Min_V=-100.0f;

	const float EgidioGranuleCell_TimeDriven::aux=(EgidioGranuleCell_TimeDriven::TableSize-1)/( EgidioGranuleCell_TimeDriven::Max_V - EgidioGranuleCell_TimeDriven::Min_V);

	float * EgidioGranuleCell_TimeDriven::channel_values=Generate_channel_values();



void EgidioGranuleCell_TimeDriven::LoadNeuronModel(string ConfigFile) throw (EDLUTFileException){
	FILE *fh;
	long Currentline = 0L;
	fh=fopen(ConfigFile.c_str(),"rt");
	if(fh){
		Currentline=1L;
		this->State = (VectorNeuronState *) new VectorNeuronState(N_NeuronStateVariables, true);

		//INTEGRATION METHOD
		loadIntegrationMethod(this->GetModelID(), fh, &Currentline);

		this->integrationMethod->SetBiFixedStepParameters(-50.0f, -50.0f, 2.0f);

		this->integrationMethod->Calculate_conductance_exp_values();

		//SET TIME-DRIVEN STEP SIZE
		this->SetTimeDrivenStepSize(this->integrationMethod->elapsedTimeInSeconds);
	}else{
		throw EDLUTFileException(TASK_EGIDIO_GRANULE_CELL_TIME_DRIVEN_LOAD, ERROR_NEURON_MODEL_OPEN, REPAIR_NEURON_MODEL_NAME, Currentline, ConfigFile.c_str(), true);
	}
	fclose(fh);
}

//This neuron model is implemented in a milisecond scale.
EgidioGranuleCell_TimeDriven::EgidioGranuleCell_TimeDriven(string NeuronTypeID, string NeuronModelID): TimeDrivenNeuronModel(NeuronTypeID, NeuronModelID, MilisecondScale), 
	//This is a constant current which can be externally injected to the cell.
	I_inj_abs(0.0e-12)/*I_inj_abs(9.0e-12)*/,
	I_inj(-I_inj_abs*1000/299.26058e-8), eexc(0.0), einh(-80), texc(0.5), tinh(10), vthr(0.0)
{
}

EgidioGranuleCell_TimeDriven::~EgidioGranuleCell_TimeDriven(void)
{
	if(this->channel_values){
		delete this->channel_values;
		this->channel_values=0;
	}
}

void EgidioGranuleCell_TimeDriven::LoadNeuronModel() throw (EDLUTFileException){
	this->LoadNeuronModel(this->GetModelID()+".cfg");
}


VectorNeuronState * EgidioGranuleCell_TimeDriven::InitializeState(){
	return this->GetVectorNeuronState();
}


InternalSpike * EgidioGranuleCell_TimeDriven::ProcessInputSpike(Interconnection * inter, double time){
	// Add the effect of the input spike
	this->GetVectorNeuronState()->IncrementStateVariableAtCPU(inter->GetTargetNeuronModelIndex(), N_DifferentialNeuronState + inter->GetType(), 1e-9f*inter->GetWeight());

	return 0;
}


bool EgidioGranuleCell_TimeDriven::UpdateState(int index, double CurrentTime){

	//Reset the number of internal spikes in this update period
	this->State->NInternalSpikeIndexs = 0;

	this->integrationMethod->NextDifferentialEquationValues();

	this->CheckValidIntegration(CurrentTime, this->integrationMethod->GetValidIntegrationVariable());

	return false;
}


enum NeuronModelOutputActivityType EgidioGranuleCell_TimeDriven::GetModelOutputActivityType(){
	return OUTPUT_SPIKE;
}

enum NeuronModelInputActivityType EgidioGranuleCell_TimeDriven::GetModelInputActivityType(){
	return INPUT_SPIKE;
}


ostream & EgidioGranuleCell_TimeDriven::PrintInfo(ostream & out){
	return out;
}	


void EgidioGranuleCell_TimeDriven::InitializeStates(int N_neurons, int OpenMPQueueIndex){
	//Initial State
	float V=-80.0f;
	float xNa_f=0.00047309535f;
	float yNa_f=1.0f;
	float xNa_r=0.00013423511f;
	float yNa_r=0.96227829f;
	float xNa_p=0.00050020111f;
	float xK_V=0.010183001f;
	float xK_A=0.15685486f;
	float yK_A=0.53565367f;
	float xK_IR=0.37337035f;
	float xK_Ca=0.00012384122f;
	float xCa=0.0021951104f;
	float yCa=0.89509747f;
	float xK_sl=0.00024031171f;
	float Ca=Ca0;
	float gexc=0.0f;
	float ginh=0.0f;

	//Initialize neural state variables.
	float initialization[] = {V,xNa_f,yNa_f,xNa_r,yNa_r,xNa_p,xK_V,xK_A,yK_A,xK_IR,xK_Ca,xCa,yCa,xK_sl,Ca,gexc,ginh};
	State->InitializeStates(N_neurons, initialization);

	//Initialize integration method state variables.
	this->integrationMethod->InitializeStates(N_neurons, initialization);
}




void EgidioGranuleCell_TimeDriven::EvaluateSpikeCondition(float previous_V, float * NeuronState, int index, float elapsedTimeInNeuronModelScale){
	if (NeuronState[0]>vthr && previous_V<vthr){
		State->NewFiredSpike(index);
		this->State->InternalSpikeIndexs[this->State->NInternalSpikeIndexs] = index;
		this->State->NInternalSpikeIndexs++;
	}
}


void EgidioGranuleCell_TimeDriven::EvaluateDifferentialEquation(float * NeuronState, float * AuxNeuronState, int index, float elapsed_time){
	float previous_V=NeuronState[0];

	float VCa=nernst(NeuronState[14],cao,2,temper);

	float * values=Get_channel_values(previous_V);
	
	//////////////////////xNa_f//////////////////////////
	float alpha_xNa_f = values[0];
	float inv_tau_xNa_f = values[1];

	//////////////////////yNa_f//////////////////////////
	float alpha_yNa_f = values[2];				
	float inv_tau_yNa_f = values[3];				

	//////////////////////xNa_r//////////////////////////
	float alpha_xNa_r = values[4];
	float inv_tau_xNa_r = values[5];

	//////////////////////yNa_r//////////////////////////
	float alpha_yNa_r = values[6];
	float inv_tau_yNa_r = values[7];

	//////////////////////xNa_p//////////////////////////
	float xNa_p_inf = values[8];						
	float inv_tau_xNa_p = values[9];

	//////////////////////xK_V//////////////////////////
	float alpha_xK_V = values[10];										
	float inv_tau_xK_V = values[11];

	//////////////////////xK_A//////////////////////////
	float xK_A_inf = values[12];
	float inv_tau_xK_A = values[13];

	//////////////////////yK_A//////////////////////////
	float yK_A_inf = values[14];
	float inv_tau_yK_A = values[15];

	//////////////////////xK_IR//////////////////////////
	float alpha_xK_IR = values[16];
	float inv_tau_xK_IR = values[17];

	//////////////////////xK_Ca//////////////////////////
	float aux_xK_Ca = values[18];
	float inv_aux_xK_Ca = values[19];
	float alpha_xK_Ca = (Q10_30*2.5f)/(1.0f + aux_xK_Ca/NeuronState[14]);	//NOOOOOOOOOOOO
	float beta_xK_Ca = (Q10_30*1.5f)/(1.0f + NeuronState[14]*inv_aux_xK_Ca);	//NOOOOOOOOOOOO
	float inv_tau_xK_Ca = (alpha_xK_Ca + beta_xK_Ca);

	//////////////////////xCa//////////////////////////
	float alpha_xCa = values[20];
	float inv_tau_xCa = values[21];

	//////////////////////yCa//////////////////////////
	float alpha_yCa = values[22];
	float inv_tau_yCa = values[23];

	//////////////////////xK_sl//////////////////////////
	float xK_sl_inf = values[24];
	float inv_tau_xK_sl = values[25];


	float gNa_f = gMAXNa_f * NeuronState[1]*NeuronState[1]*NeuronState[1] * NeuronState[2];
	float gNa_r = gMAXNa_r * NeuronState[3] * NeuronState[4];
	float gNa_p= gMAXNa_p * NeuronState[5];
	float gK_V  = gMAXK_V * NeuronState[6]*NeuronState[6]*NeuronState[6]*NeuronState[6];
	float gK_A  = gMAXK_A * NeuronState[7]*NeuronState[7]*NeuronState[7] * NeuronState[8];
	float gK_IR = gMAXK_IR * NeuronState[9];
	float gK_Ca = gMAXK_Ca * NeuronState[10];
	float gCa    = gMAXCa * NeuronState[11]*NeuronState[11] * NeuronState[12];
	float gK_sl  = gMAXK_sl * NeuronState[13];

	 AuxNeuronState[1]=(alpha_xNa_f - NeuronState[1]*inv_tau_xNa_f);
	 AuxNeuronState[2]=(alpha_yNa_f - NeuronState[2]*inv_tau_yNa_f);
	 AuxNeuronState[3]=(alpha_xNa_r - NeuronState[3]*inv_tau_xNa_r);
	 AuxNeuronState[4]=(alpha_yNa_r - NeuronState[4]*inv_tau_yNa_r);
	 AuxNeuronState[5]=(xNa_p_inf - NeuronState[5])*inv_tau_xNa_p;
	 AuxNeuronState[6]=(alpha_xK_V - NeuronState[6]*inv_tau_xK_V);
	 AuxNeuronState[7]=(xK_A_inf  - NeuronState[7])*inv_tau_xK_A;
	 AuxNeuronState[8]=(yK_A_inf  - NeuronState[8])*inv_tau_yK_A;
	 AuxNeuronState[9]=(alpha_xK_IR - NeuronState[9]*inv_tau_xK_IR);
	 AuxNeuronState[10]=(alpha_xK_Ca - NeuronState[10]*inv_tau_xK_Ca);
	 AuxNeuronState[11]=(alpha_xCa - NeuronState[11]*inv_tau_xCa);
	 AuxNeuronState[12]=(alpha_yCa - NeuronState[12]*inv_tau_yCa);
	 AuxNeuronState[13]=(xK_sl_inf-NeuronState[13])*inv_tau_xK_sl;
	 AuxNeuronState[14]=(-gCa*(previous_V-VCa)/(2*F*A*d) - (betaCa*(NeuronState[14] - Ca0)));
	 AuxNeuronState[0]=(-1/Cm)*((NeuronState[15]/299.26058e-8f) * (previous_V - eexc) + (NeuronState[16]/299.26058e-8f) * (previous_V - einh)+gNa_f*(previous_V-VNa)+gNa_r*(previous_V-VNa)+gNa_p*(previous_V-VNa)+gK_V*(previous_V-VK)+gK_A*(previous_V-VK)+gK_IR*(previous_V-VK)+gK_Ca*(previous_V-VK)+gCa*(previous_V-VCa)+gK_sl*(previous_V-VK)+gLkg1*(previous_V-VLkg1)+gLkg2*(previous_V-VLkg2)+I_inj);
}



void EgidioGranuleCell_TimeDriven::EvaluateTimeDependentEquation(float * NeuronState, int index, int elapsed_time_index){
	float limit=1e-18;
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

void EgidioGranuleCell_TimeDriven::Calculate_conductance_exp_values(int index, float elapsed_time){
	//excitatory synapse.
	Set_conductance_exp_values(index, 0, exp(-elapsed_time/this->texc));
	//inhibitory synapse.
	Set_conductance_exp_values(index, 1, exp(-elapsed_time/this->tinh));
}


bool EgidioGranuleCell_TimeDriven::CheckSynapseType(Interconnection * connection){
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


void EgidioGranuleCell_TimeDriven::loadIntegrationMethod(string fileName, FILE *fh, long * Currentline)throw (EDLUTFileException){
	char ident_type[MAXIDSIZE + 1];

	//We load the integration method type.
	skip_comments(fh, *Currentline);
	if (fscanf(fh, "%s", ident_type) == 1){
		skip_comments(fh, *Currentline);
		//DEFINE HERE NEW INTEGRATION METHOD
		if (strncmp(ident_type, "Euler", 5) == 0){
			integrationMethod = (Euler<EgidioGranuleCell_TimeDriven> *) new Euler<EgidioGranuleCell_TimeDriven>(this);
		}
		else if (strncmp(ident_type, "RK2", 3) == 0){
			integrationMethod = (RK2<EgidioGranuleCell_TimeDriven> *) new RK2<EgidioGranuleCell_TimeDriven>(this);
		}
		else if (strncmp(ident_type, "RK4", 3) == 0){
			integrationMethod = (RK4<EgidioGranuleCell_TimeDriven> *) new RK4<EgidioGranuleCell_TimeDriven>(this);
		}
		else if (strncmp(ident_type, "BDF", 3) == 0 && atoi(&ident_type[3])>0 && atoi(&ident_type[3])<7){
			integrationMethod = (BDFn<EgidioGranuleCell_TimeDriven> *) new BDFn<EgidioGranuleCell_TimeDriven>(this, atoi(&ident_type[3]));
		}
		else if (strncmp(ident_type, "Bifixed_Euler", 13) == 0){
			integrationMethod = (Bifixed_Euler<EgidioGranuleCell_TimeDriven> *) new Bifixed_Euler<EgidioGranuleCell_TimeDriven>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK2", 11) == 0){
			integrationMethod = (Bifixed_RK2<EgidioGranuleCell_TimeDriven> *) new Bifixed_RK2<EgidioGranuleCell_TimeDriven>(this);
		}
		else if (strncmp(ident_type, "Bifixed_RK4", 11) == 0){
			integrationMethod = (Bifixed_RK4<EgidioGranuleCell_TimeDriven> *) new Bifixed_RK4<EgidioGranuleCell_TimeDriven>(this);
		}
		else if (strncmp(ident_type, "Bifixed_BDF", 11) == 0 && atoi(&ident_type[11]) == 2){
			integrationMethod = (Bifixed_BDFn<EgidioGranuleCell_TimeDriven> *) new Bifixed_BDFn<EgidioGranuleCell_TimeDriven>(this, atoi(&ident_type[11]));
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







