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

#include "../inc/EntrySignal.h"

#include <stdio.h>
//#include <math.h>
#include <cmath>


EntrySignal::EntrySignal(float NewStepTime, float NewTInit, float NewTEnd, int NewType, float NewAmplitude,float NewFrequency){

	StepTime=NewStepTime;
	TInit=NewTInit;
	TEnd=NewTEnd;
	Type=NewType;	
	Amplitude=NewAmplitude;
	Frequency=NewFrequency;
 	NElements=(TEnd-TInit)/StepTime;
	Signal = new float [NElements*NUM_JOINT];

	float w = 2 * 3.141592f;
	float rho = 0.0f;
    int njoint;
	float time =TInit;
		for (int i=0; i<NElements; i++){
				switch (Type){
					//Sinusoidal signal for type 0.
					case 0: Signal[i]=Amplitude*sin(Frequency*w*time+rho);
						break;
					//Square signal for type 1.
					case 1: 
						if (sin(Frequency*w*time+rho)>=0.0){
								Signal[i]=Amplitude*1.0; 
						}else{
								Signal[i]=Amplitude*(-1.0);
						}
									 
						break;
					default:
						printf("No input available for that choise");
						break;
					}		time+=StepTime;
			
			
				}	

	////Store the entry file.
	//char InputFile[32]={"EntryWave.txt"};
	//FILE *fentry;
	//fentry=fopen(InputFile,"wt");
	//
	//for (int i=0; i<NElements; i++){
	//		fprintf(fentry,"%f \n",Signal[i]);	
	//}
	//fclose(fentry);
	}


EntrySignal::~EntrySignal(){
	delete Signal;
}


float * EntrySignal::GetSignal(){
	return Signal;
}

int EntrySignal::GetNElements(){
	return NElements;
}
