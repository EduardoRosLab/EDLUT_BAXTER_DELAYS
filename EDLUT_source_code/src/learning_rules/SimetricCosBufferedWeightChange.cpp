/***************************************************************************
 *                           SimetricCosBufferedWeightChange.cpp           *
 *                           -------------------                           *
 * copyright            : (C) 2016 by Francisco Naveros and Niceto Luque   *
 * email                : fnaveros@ugr.es nluque@ugr.es                    *
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/


#include "../../include/learning_rules/SimetricCosBufferedWeightChange.h"
#include "../../include/learning_rules/BufferedActivityTimes.h"

#include "../../include/spike/Interconnection.h"
#include "../../include/spike/Neuron.h"

#include "../../include/simulation/Utils.h"

#include "../../include/openmp/openmp.h"

SimetricCosBufferedWeightChange::SimetricCosBufferedWeightChange(int NewLearningRuleIndex) :WithoutPostSynaptic(NewLearningRuleIndex),
bufferedActivityTimesTrigger(0), bufferedActivityTimesNoTrigger(0){
}

SimetricCosBufferedWeightChange::~SimetricCosBufferedWeightChange(){
	if(bufferedActivityTimesNoTrigger!=0){
		delete bufferedActivityTimesNoTrigger;
	}
	if(bufferedActivityTimesTrigger!=0){
		delete bufferedActivityTimesTrigger;
	}
}


void SimetricCosBufferedWeightChange::InitializeConnectionState(unsigned int NumberOfSynapses, unsigned int NumberOfNeurons){
	this->maxTimeMeasured = tau * 4;
	this->inv_maxTimeMeasured = 1.0f / this->maxTimeMeasured;
	//Precompute the kernel in the look-up table.
	for (int i = 0; i<N_elements; i++){
		double time = maxTimeMeasured*i / (N_elements*tau);
		kernelLookupTable[i] = exp(-time*exponent)*sin(1.5708f*time)*sin(1.5708f*time);
	}

	//Inicitialize de buffer of activity
	bufferedActivityTimesNoTrigger = new BufferedActivityTimes(NumberOfNeurons);
	bufferedActivityTimesTrigger = new BufferedActivityTimes(NumberOfNeurons);
}


void SimetricCosBufferedWeightChange::LoadLearningRule(FILE * fh, long & Currentline, string fileName) throw (EDLUTFileException){
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


void SimetricCosBufferedWeightChange::ApplyPreSynapticSpike(Interconnection * Connection, double SpikeTime){

	if (Connection->GetTriggerConnection() == false){
		//LTP
		Connection->IncrementWeight(this->a1pre);
		Neuron * TargetNeuron = Connection->GetTarget();
		int neuron_index = TargetNeuron->GetIndex();
		int synapse_index = Connection->LearningRuleIndex_withoutPost_insideTargetNeuron;
		this->bufferedActivityTimesNoTrigger->InsertElement(neuron_index, SpikeTime, SpikeTime - this->maxTimeMeasured, synapse_index);


		//LTD
		int N_elements = bufferedActivityTimesTrigger->ProcessElements(neuron_index, SpikeTime - this->maxTimeMeasured);
		SpikeData * spike_data = bufferedActivityTimesTrigger->GetOutputSpikeData();

		float value = 0;
		for (int i = 0; i < N_elements; i++){
			double ElapsedTime = SpikeTime - spike_data[i].time;
			int tableIndex = ElapsedTime*this->N_elements*this->inv_maxTimeMeasured;
			value += this->kernelLookupTable[tableIndex];
		}
		Connection->IncrementWeight(this->a2prepre*value);
	}
	else{
		Neuron * TargetNeuron = Connection->GetTarget();
		int neuron_index = TargetNeuron->GetIndex();
		this->bufferedActivityTimesTrigger->InsertElement(neuron_index, SpikeTime, SpikeTime - this->maxTimeMeasured, 0);



		//LTD
		int N_elements = bufferedActivityTimesNoTrigger->ProcessElements(neuron_index, SpikeTime - this->maxTimeMeasured);
		SpikeData * spike_data = bufferedActivityTimesNoTrigger->GetOutputSpikeData();

		float value = 0;
		for (int i = 0; i < N_elements; i++){
			Interconnection * interi = TargetNeuron->GetInputConnectionWithoutPostSynapticLearningAt(spike_data[i].synapse_index);


			double ElapsedTime = SpikeTime - spike_data[i].time;
			int tableIndex = ElapsedTime*this->N_elements*this->inv_maxTimeMeasured;
			value = this->kernelLookupTable[tableIndex];

			// Update synaptic weight
			interi->IncrementWeight(this->a2prepre*value);
		}
	}
}



ostream & SimetricCosBufferedWeightChange::PrintInfo(ostream & out){

	out << "- Simetric Cos Kernel Learning Rule: \t" << this->tau << "\t" << this->exponent<< "\t" << this->a1pre << "\t" << this->a2prepre << endl;


	return out;
}
