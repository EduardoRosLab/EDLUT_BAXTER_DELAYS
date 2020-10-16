/***************************************************************************
 *                           SinWeightChange.cpp                           *
 *                           -------------------                           *
 * copyright            : (C) 2009 by Jesus Garrido and Richard Carrillo   *
 * email                : jgarrido@atc.ugr.es                              *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#include "../../include/learning_rules/SinWeightChange.h"

#include "../../include/learning_rules/SinState.h"

#include "../../include/spike/Interconnection.h"

SinWeightChange::SinWeightChange(int NewLearningRuleIndex):AdditiveKernelChange(NewLearningRuleIndex), exponent(0){
}

SinWeightChange::~SinWeightChange(){
}


void SinWeightChange::InitializeConnectionState(unsigned int NumberOfSynapses, unsigned int NumberOfNeurons){
	this->State=(ConnectionState *) new SinState(NumberOfSynapses, this->exponent,this->maxpos);
}

int SinWeightChange::GetNumberOfVar() const{
	return this->exponent+2;
}

int SinWeightChange::GetExponent() const{
	return this->exponent;
}

void SinWeightChange::LoadLearningRule(FILE * fh, long & Currentline, string fileName) throw (EDLUTFileException){
	AdditiveKernelChange::LoadLearningRule(fh,Currentline,fileName);

	if((fscanf(fh,"%i",&this->exponent)==1)){
		if (this->exponent <= 0 || exponent % 2 == 1 || exponent > 20){
			throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_SIN_WEIGHT_CHANGE_EXPONENT, REPAIR_LEARNING_RULE_VALUES, Currentline, fileName.c_str(), true);
		}
	}else{
		throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_LEARNING_RULE_LOAD, REPAIR_SIN_WEIGHT_CHANGE_LOAD, Currentline, fileName.c_str(), true);
	}


}

