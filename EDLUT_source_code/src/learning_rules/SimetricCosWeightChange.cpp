/***************************************************************************
 *                           CosWeightChange.cpp                           *
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


#include "../../include/learning_rules/SimetricCosWeightChange.h"

#include "../../include/learning_rules/SimetricCosState.h"

#include "../../include/spike/Interconnection.h"
#include "../../include/spike/Neuron.h"

#include "../../include/simulation/Utils.h"

#include "../../include/openmp/openmp.h"

SimetricCosWeightChange::SimetricCosWeightChange(int NewLearningRuleIndex):WithoutPostSynaptic(NewLearningRuleIndex){
}

SimetricCosWeightChange::~SimetricCosWeightChange(){

}


void SimetricCosWeightChange::InitializeConnectionState(unsigned int NumberOfSynapses, unsigned int NumberOfNeurons){
	this->State=(ConnectionState *) new SimetricCosState(NumberOfSynapses, this->tau, this->exponent);
}


void SimetricCosWeightChange::LoadLearningRule(FILE * fh, long & Currentline, string fileName) throw (EDLUTFileException){
	skip_comments(fh,Currentline);

	if(fscanf(fh,"%f",&this->tau)==1 && fscanf(fh,"%f",&this->exponent)==1 && fscanf(fh,"%f",&this->a1pre)==1 && fscanf(fh,"%f",&this->a2prepre)==1){
		if (this->tau <= 0){
			throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_COS_WEIGHT_CHANGE_TAU, REPAIR_LEARNING_RULE_VALUES, Currentline, fileName.c_str(), true);
		}
		if (this->exponent <= 0){
			throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_COS_WEIGHT_CHANGE_EXPONENT, REPAIR_LEARNING_RULE_VALUES, Currentline, fileName.c_str(), true);
		}
	}else{
		throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_LEARNING_RULE_LOAD, REPAIR_COS_WEIGHT_CHANGE_LOAD, Currentline, fileName.c_str(), true);
	}
}



//void SimetricCosWeightChange::ApplyPreSynapticSpike(Interconnection * Connection,double SpikeTime){
//	
//	if(Connection->GetTriggerConnection()==false){
//		int LearningRuleIndex = Connection->GetLearningRuleIndex_withoutPost();
//
//		//LTP
//		// Second case: the weight change is linked to this connection
//		Connection->IncrementWeight(this->a1pre);
//
//		// Update the presynaptic activity
//		State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);
//
//		// Add the presynaptic spike influence
//		State->ApplyPresynapticSpike(LearningRuleIndex);
//
//
//		//LTD
//		int N_TriggerConnection=Connection->GetTarget()->GetN_TriggerConnection();
//		if(N_TriggerConnection>0){
//			Interconnection ** inter=Connection->GetTarget()->GetTriggerConnection();
//			for(int i=0; i<N_TriggerConnection; i++){
//				// Apply sinaptic plasticity driven by teaching signal
//				int LearningRuleIndex = inter[i]->GetLearningRuleIndex_withoutPost();
//
//				// Update the presynaptic activity
//				State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);
//
//				// Update synaptic weight
//				Connection->IncrementWeight(this->a2prepre*State->GetPresynapticActivity(LearningRuleIndex));
//			}
//		}
//	}else{
//		int LearningRuleIndex = Connection->GetLearningRuleIndex_withoutPost();
//
//		// Update the presynaptic activity
//		State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);
//		// Add the presynaptic spike influence
//		State->ApplyPresynapticSpike(LearningRuleIndex);
//
//
//
//		Neuron * TargetNeuron=Connection->GetTarget();
//
//		for (int i = 0; i<TargetNeuron->GetInputNumberWithoutPostSynapticLearning(); ++i){
//
//			Interconnection * interi=TargetNeuron->GetInputConnectionWithoutPostSynapticLearningAt(i);
//
//			if(interi->GetTriggerConnection()==false){
//				// Apply sinaptic plasticity driven by teaching signal
//				int LearningRuleIndex = interi->GetLearningRuleIndex_withoutPost();
//
//				// Update the presynaptic activity
//				State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);
//
//				// Update synaptic weight
//				interi->IncrementWeight(this->a2prepre*State->GetPresynapticActivity(LearningRuleIndex));
//			}
//		}
//	}
//}


void SimetricCosWeightChange::ApplyPreSynapticSpike(Interconnection * Connection, double SpikeTime){

	if (Connection->GetTriggerConnection() == false){
		int LearningRuleIndex = Connection->GetLearningRuleIndex_withoutPost();

		//LTP
		// Second case: the weight change is linked to this connection
		Connection->IncrementWeight(this->a1pre);

		// Update the presynaptic activity
		State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);

		// Add the presynaptic spike influence
		State->ApplyPresynapticSpike(LearningRuleIndex);


		//LTD
		int N_TriggerConnection = Connection->GetTarget()->GetN_TriggerConnection();
		if (N_TriggerConnection>0){
			Interconnection ** inter = Connection->GetTarget()->GetTriggerConnection();
			for (int i = 0; i<N_TriggerConnection; i++){
				// Apply sinaptic plasticity driven by teaching signal
				int LearningRuleIndex = inter[i]->GetLearningRuleIndex_withoutPost();

				// Update the presynaptic activity
				State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);

				// Update synaptic weight
				Connection->IncrementWeight(this->a2prepre*State->GetPresynapticActivity(LearningRuleIndex));
			}
		}
	}
	else{
		int LearningRuleIndex = Connection->GetLearningRuleIndex_withoutPost();

		// Update the presynaptic activity
		State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);
		// Add the presynaptic spike influence
		State->ApplyPresynapticSpike(LearningRuleIndex);



		Neuron * TargetNeuron = Connection->GetTarget();

		for (int i = 0; i<TargetNeuron->GetInputNumberWithoutPostSynapticLearning(); ++i){
			//We implement this weight increment mechanisme in order to improve the cache friendly. Thus we do not need to red a large number of none consecutive synapses from memory.
			//This weight increments will be acumulated in the synaptic weights when a spike will be propagated for this synapses.
			int LearningRuleIndex = TargetNeuron->IndexInputLearningConnections[1][i];

			if (LearningRuleIndex >= 0){
				Interconnection * interi = TargetNeuron->GetInputConnectionWithoutPostSynapticLearningAt(i);

				// Update the presynaptic activity
				State->SetNewUpdateTime(LearningRuleIndex, SpikeTime, false);

				// Update synaptic weight
				interi->IncrementWeight(this->a2prepre*State->GetPresynapticActivity(LearningRuleIndex));
			}
		}
	}
}



ostream & SimetricCosWeightChange::PrintInfo(ostream & out){

	out << "- Simetric Cos Kernel Learning Rule: \t" << this->tau << "\t" << this->exponent<< "\t" << this->a1pre << "\t" << this->a2prepre << endl;


	return out;
}
