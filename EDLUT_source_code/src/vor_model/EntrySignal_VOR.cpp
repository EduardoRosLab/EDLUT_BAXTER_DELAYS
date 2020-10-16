/***************************************************************************
 *                           EntrySignal.cpp                               *
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

#include "../../include/vor_model/EntrySignal_VOR.h"

#include <stdio.h>
//#include <math.h>
#include <cmath>


EntrySignalVOR::EntrySignalVOR(){
	SignalVOR = 0;
}

EntrySignalVOR::EntrySignalVOR(double NewTimeStep, double Newtsimul,double NewAmplitude,double NewFrequency,int NewType,int NewJoints){

	StepTime=NewTimeStep;
	Tsimul=Newtsimul;
	Type=NewType;
	Amplitude=NewAmplitude;
	Frequency=NewFrequency;
	NJoints=NewJoints;
 	SignalVOR = new double [3*NewJoints];
	int njoint;
	double w = 2 * 3.141592;
	double rho = 0.0;
	double A,B,C;
	B=cos(NewFrequency*w*Newtsimul);
	C=sin(NewFrequency*w*Newtsimul);
	for(njoint=0;njoint<NewJoints;njoint++){			
			switch (Type){
							//Sinusoidal signal for type 0.
							case 0: {
								SignalVOR[njoint]=NewAmplitude*sin(NewFrequency*w*Newtsimul+njoint*3.141592/4.0);
									SignalVOR[njoint+NewJoints]=2.0*NewAmplitude*3.141592*NewFrequency*cos(NewFrequency*w*Newtsimul);
									SignalVOR[njoint+2*NewJoints]=-4.0*NewAmplitude*3.141592*3.141592*NewFrequency*NewFrequency*sin(NewFrequency*w*Newtsimul);
								break;
									}
							//Square signal for type 1.
							case 1: {
								if (sin(NewFrequency*w*Newtsimul+njoint*3.141592/4.0)>=0.0){
										SignalVOR[njoint]=NewAmplitude; 
								}else{
										SignalVOR[njoint]=NewAmplitude*(-1.0);
								}
								
								if(B>=0.0){
										SignalVOR[njoint+NewJoints]=2.0*NewAmplitude*3.141592*NewFrequency;
								}else{
										SignalVOR[njoint+NewJoints]=2.0*NewAmplitude*3.141592*NewFrequency*(-1.0);
								}
								
								if(C>=0.0){
										SignalVOR[njoint+2*NewJoints]=-4.0*NewAmplitude*3.141592*3.141592*NewFrequency*NewFrequency;
								}else{
										SignalVOR[njoint+2*NewJoints]=-4.0*NewAmplitude*3.141592*3.141592*NewFrequency*NewFrequency*(-1.0);
								}
								break;
									}
							default:{
								printf("No input available for that choise");
								break;
									}
			}		
			
			
	}
}




EntrySignalVOR::~EntrySignalVOR(){
	if (SignalVOR != 0){
		delete SignalVOR;
	}
}


double * EntrySignalVOR::GetSignalVOR(){
	return SignalVOR;
}

double EntrySignalVOR::GetSignalVOR(int index){
	return SignalVOR[index];
}

int EntrySignalVOR::GetNElements(){
	return NJoints;
}