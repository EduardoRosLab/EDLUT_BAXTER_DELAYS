/***************************************************************************
 *                           STDPWeightChange.cpp                          *
 *                           -------------------                           *
 * copyright            : (C) 2010 by Jesus Garrido                        *
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

#include "../../include/learning_rules/SimetricCosSinSTDPWeightChange.h"

#include "../../include/learning_rules/SimetricCosSinSTDPState.h"

#include "../../include/spike/Interconnection.h"
#include "../../include/spike/Neuron.h"

#include "../../include/simulation/Utils.h"

#include "../../include/neuron_model/NeuronState.h"
#include "../../include/spike/Neuron.h"


SimetricCosSinSTDPWeightChange::SimetricCosSinSTDPWeightChange(int NewLearningRuleIndex):WithPostSynaptic(NewLearningRuleIndex){
}

SimetricCosSinSTDPWeightChange::~SimetricCosSinSTDPWeightChange(){

}


void SimetricCosSinSTDPWeightChange::InitializeConnectionState(unsigned int NumberOfSynapses, unsigned int NumberOfNeurons){
	this->State=(ConnectionState *) new SimetricCosSinSTDPState(NumberOfSynapses+NumberOfNeurons, this->MaxMinDistance, this->CentralAmplitudeFactor, this->LateralAmplitudeFactor);
}

void SimetricCosSinSTDPWeightChange::ApplyPreSynapticSpike(Interconnection * Connection,double SpikeTime){
	
	int LearningRuleIndex = Connection->GetLearningRuleIndex_withPost();

	// Update the presynaptic activity
	State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);

	// Add the presynaptic spike influence
	State->ApplyPresynapticSpike(LearningRuleIndex);


	//LTD
	int SecondLearningRuleIndex = Connection->GetTarget()->GetIndex();
	
	// Update the presynaptic activity
	State->SetNewUpdateTime(SecondLearningRuleIndex, SpikeTime, false);

	// Update synaptic weight
	Connection->IncrementWeight(State->GetPresynapticActivity(SecondLearningRuleIndex));
}

void SimetricCosSinSTDPWeightChange::ApplyPostSynapticSpike(Interconnection * Connection,double SpikeTime){


	//increment internal spike kernel previous to "future propagate spike"
	int SecondLearningRuleIndex = Connection->GetTarget()->GetIndex();
	if(SpikeTime > State->GetLastUpdateTime(SecondLearningRuleIndex)){
		State->SetNewUpdateTime(SecondLearningRuleIndex, SpikeTime, false);

		State->ApplyPresynapticSpike(SecondLearningRuleIndex);

	}


	Neuron * TargetNeuron=Connection->GetTarget();

	int LearningRuleIndex = Connection->GetLearningRuleIndex_withPost();

	// Update the presynaptic activity
	State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);
			
	// Update synaptic weight
	Connection->IncrementWeight(State->GetPresynapticActivity(LearningRuleIndex));

	return;
}


void SimetricCosSinSTDPWeightChange::ApplyPostSynapticSpike(Neuron * neuron, double SpikeTime){
	int SecondLearningRuleIndex = neuron->GetIndex();
	State->SetNewUpdateTime(SecondLearningRuleIndex, SpikeTime, false);
	State->ApplyPresynapticSpike(SecondLearningRuleIndex);


	for (int i = 0; i < neuron->GetInputNumberWithPostSynapticLearning(); ++i){
		Interconnection * interi = neuron->GetInputConnectionWithPostSynapticLearningAt(i);

		//Aplicate propagate spike kernel to "future internal spike"
		int LearningRuleIndex = neuron->IndexInputLearningConnections[0][i];

		// Update the presynaptic activity
		State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);

		// Update synaptic weight
		interi->IncrementWeight(State->GetPresynapticActivity(LearningRuleIndex));
	}

}


void SimetricCosSinSTDPWeightChange::LoadLearningRule(FILE * fh, long & Currentline, string fileName) throw (EDLUTFileException){
	skip_comments(fh,Currentline);

	if(fscanf(fh,"%f",&this->MaxMinDistance)==1 && fscanf(fh,"%f",&this->CentralAmplitudeFactor)==1 && fscanf(fh,"%f",&this->LateralAmplitudeFactor)==1){
		if (this->MaxMinDistance <= 0){
			throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_COS_SIN_WEIGHT_CHANGE_AMPLITUDE, REPAIR_LEARNING_RULE_VALUES, Currentline, fileName.c_str(), true);
		}
		if (this->CentralAmplitudeFactor * this->LateralAmplitudeFactor > 0.0f){
			throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_COS_SIN_WEIGHT_CHANGE_SIGNS, REPAIR_LEARNING_RULE_VALUES, Currentline, fileName.c_str(), true);
		}
	}else{
		throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_LEARNING_RULE_LOAD, REPAIR_COS_SIN_WEIGHT_CHANGE_LOAD, Currentline, fileName.c_str(), true);
	}
}

ostream & SimetricCosSinSTDPWeightChange::PrintInfo(ostream & out){

//	out << "- SimetricSTDP Learning Rule: LTD " << this->MaxChangeLTD << "\tLTP " << this->MaxChangeLTP << "\tTau" << this->tau << endl;

	return out;
}




