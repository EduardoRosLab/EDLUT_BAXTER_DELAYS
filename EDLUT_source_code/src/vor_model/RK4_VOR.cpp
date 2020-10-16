/***************************************************************************
 *                           RK4_VOR.cpp                                   *
 *                           -------------------                           *
 * copyright            : (C) 2015 by Niceto Luque and Francisco Naveros   *
 * email                : nluque@ugr.es and fnaveros@ugr.es                *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#define MAX_VARIABLES 2

#include "../../include/vor_model/RK4_VOR.h"

#include "../../include/vor_model/Individual_VOR.h" 

RK4_VOR::RK4_VOR(){
}

RK4_VOR::~RK4_VOR(){
}
	


void RK4_VOR::NextDifferentialEcuationValues(Individual * individual, double StepTime, double input){

	int j;

	double AuxNeuronState[MAX_VARIABLES];
	double AuxNeuronState1[MAX_VARIABLES];
	double AuxNeuronState2[MAX_VARIABLES];
	double AuxNeuronState3[MAX_VARIABLES];
	double AuxNeuronState4[MAX_VARIABLES];
	

	//1st term
	individual->FuncVOR(input,individual->States, AuxNeuronState1);
	
	//2nd term
	for (j=0; j<individual->NStates; j++){
		AuxNeuronState[j]= individual->States[j] + AuxNeuronState1[j]*StepTime*0.5f;
	}

	individual->FuncVOR(input,AuxNeuronState, AuxNeuronState2);


	//3rd term
	for (j=0; j<individual->NStates; j++){
		AuxNeuronState[j]= individual->States[j] + AuxNeuronState2[j]*StepTime*0.5f;
	}

	individual->FuncVOR(input,AuxNeuronState, AuxNeuronState3);

	//4rd term
	for (j=0; j<individual->NStates; j++){
		AuxNeuronState[j]= individual->States[j] + AuxNeuronState3[j]*StepTime;
	}

	individual->FuncVOR(input,AuxNeuronState, AuxNeuronState4);


	for (j=0; j<individual->NStates; j++){
		individual->States[j]+=(AuxNeuronState1[j]+2.0f*(AuxNeuronState2[j]+AuxNeuronState3[j])+AuxNeuronState4[j])*StepTime*0.1666666;
	}
}



