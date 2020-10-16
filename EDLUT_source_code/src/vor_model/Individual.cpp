/***************************************************************************
 *                           Individual.cpp                                *
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

#include "../inc/Individual.h"
#include "../inc/RK4_VOR.h"
#include "../inc/EntrySignal.h"

#include <stdio.h>





Individual::Individual(float new_K, float new_TC1, float new_TC2){
	SetParameters(new_K, new_TC1, new_TC2);
}


Individual::Individual(Individual * new_Individual):K(new_Individual->K), TC1(new_Individual->TC1), TC2(new_Individual->TC2){
		States[0]=new_Individual->States[0];
		States[1]=new_Individual->States[1];
		A[0]=new_Individual->A[0];
		A[1]=new_Individual->A[1];
		B[0]=new_Individual->B[0];
		B[1]=new_Individual->B[1];
}


Individual::~Individual(){
}

void Individual::SetParameters(float new_K, float new_TC1, float new_TC2){
	K=new_K;
	TC1=new_TC1;
	TC2=new_TC2;
	States[0]=0.0f;
	States[1]=0.0f;
	A[0]=1.0f/(TC1*TC2);
	A[1]=(TC1+TC2)/(TC1*TC2);
	B[0]=0;
	B[1]=(K*TC1)/(TC1*TC2);
}

void Individual::FuncVOR(float input, float * inputState, float * outputState){
	outputState[0]=inputState[1];
	outputState[1]=-A[0]*inputState[0]-A[1]*inputState[1]+input;
}

float Individual::FitnessVOR(EntrySignal * entrySignal, RK4_VOR * Integrator){
	float CuadraticError=0.0f;

	for (int i=0; i<entrySignal->GetNElements(); i++){
		//integration.
		Integrator->NextDifferentialEcuationValues(this,entrySignal->StepTime,entrySignal->Signal[i]);

		float Output=B[1]*States[1];
		//Cuadratic error.
		CuadraticError+=(entrySignal->Signal[i]-Output)*(entrySignal->Signal[i]-Output);
	}
	float SSE=CuadraticError/entrySignal->GetNElements();
	return SSE;
}

float * Individual::CalculateOutputVOR(EntrySignal * entrySignal, RK4_VOR * Integrator){
	float * OutputVOR = new float [entrySignal->GetNElements()];
	for (int i=0; i<entrySignal->GetNElements(); i++){
		//integration.
		Integrator->NextDifferentialEcuationValues(this,entrySignal->StepTime,entrySignal->Signal[i]);

		OutputVOR[i]=B[1]*States[1];
	}
	return OutputVOR;
}
