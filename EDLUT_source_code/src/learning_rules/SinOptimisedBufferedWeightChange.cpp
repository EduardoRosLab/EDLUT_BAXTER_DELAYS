/***************************************************************************
 *                           SinOptimisedBufferedWeightChange.cpp          *
 *                           -------------------                           *
 * copyright            : (C) 2016 by Francisco Naveros                    *
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


#include "../../include/learning_rules/SinOptimisedBufferedWeightChange.h"
#include "../../include/learning_rules/BufferedActivityTimes.h"

#include "../../include/spike/Interconnection.h"
#include "../../include/spike/Neuron.h"

#include "../../include/simulation/Utils.h"

#include "../../include/openmp/openmp.h"

SinOptimisedBufferedWeightChange::SinOptimisedBufferedWeightChange(int NewLearningRuleIndex):AdditiveKernelChange(NewLearningRuleIndex), exponent(0), 
bufferedActivityTimesNoTrigger(0), kernelLookupTable(0){
}

SinOptimisedBufferedWeightChange::~SinOptimisedBufferedWeightChange(){
	if(bufferedActivityTimesNoTrigger!=0){
		delete bufferedActivityTimesNoTrigger;
	}
	if(kernelLookupTable!=0){
		delete kernelLookupTable;
	}
}


void SinOptimisedBufferedWeightChange::InitializeConnectionState(unsigned int NumberOfSynapses, unsigned int NumberOfNeurons){
	float tau = this->maxpos / atan((float)exponent);
	float factor = 1. / (exp(-atan((float)this->exponent))*pow(sin(atan((float)this->exponent)), (int) this->exponent));

	if (tau == 0){
		tau = 1e-6;
	}

	double step_size = 0.0001;

	this->maxTimeMeasured = this->maxpos;
	while (1){
		this->maxTimeMeasured += step_size;
		if ((exp(-this->maxTimeMeasured / tau)*pow(sin(this->maxTimeMeasured / tau), double(exponent))*factor) < 1e-6){
			break;
		}
	}
	if (this->maxTimeMeasured > maxpos * 4){
		this->maxTimeMeasured = maxpos * 4;
	}

	this->N_elements = this->maxTimeMeasured / step_size + 1;
	kernelLookupTable = new float[this->N_elements];


	this->inv_maxTimeMeasured = 1.0f / this->maxTimeMeasured;
	//Precompute the kernel in the look-up table.
	for (int i = 0; i<N_elements; i++){
		double time = maxTimeMeasured*i / (N_elements*tau);
		kernelLookupTable[i] = exp(-time)*pow(sin(time), double(exponent))*factor;
	}

	//Inicitialize de buffer of activity
	bufferedActivityTimesNoTrigger = new BufferedActivityTimes(NumberOfNeurons);
}

int SinOptimisedBufferedWeightChange::GetNumberOfVar() const{
	return this->exponent+2;
}

int SinOptimisedBufferedWeightChange::GetExponent() const{
	return this->exponent;
}

void SinOptimisedBufferedWeightChange::LoadLearningRule(FILE * fh, long & Currentline, string fileName) throw (EDLUTFileException){
	AdditiveKernelChange::LoadLearningRule(fh,Currentline,fileName);

	if((fscanf(fh,"%i",&this->exponent)==1)){
		if (this->exponent <= 0 || exponent % 2 == 1 || exponent > 20){
			throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_SIN_WEIGHT_CHANGE_EXPONENT, REPAIR_LEARNING_RULE_VALUES, Currentline, fileName.c_str(), true);
		}
	}else{
		throw EDLUTFileException(TASK_LEARNING_RULE_LOAD, ERROR_LEARNING_RULE_LOAD, REPAIR_SIN_WEIGHT_CHANGE_LOAD, Currentline, fileName.c_str(), true);
	}
}



void SinOptimisedBufferedWeightChange::ApplyPreSynapticSpike(Interconnection * Connection, double SpikeTime){

	if (Connection->GetTriggerConnection() == false){
		Connection->IncrementWeight(this->a1pre);
		int neuron_index = Connection->GetTarget()->GetIndex();
		int synapse_index = Connection->LearningRuleIndex_withoutPost_insideTargetNeuron;
		this->bufferedActivityTimesNoTrigger->InsertElement(neuron_index, SpikeTime, SpikeTime - this->maxTimeMeasured, synapse_index);

	}
	else{
		Neuron * TargetNeuron = Connection->GetTarget();
		int neuron_index = TargetNeuron->GetIndex();

		int N_elements = bufferedActivityTimesNoTrigger->ProcessElements(neuron_index, SpikeTime - this->maxTimeMeasured);
		SpikeData * spike_data = bufferedActivityTimesNoTrigger->GetOutputSpikeData();


		for (int i = 0; i < N_elements; i++){
			Interconnection * interi = TargetNeuron->GetInputConnectionWithoutPostSynapticLearningAt(spike_data[i].synapse_index);

			double ElapsedTime = SpikeTime - spike_data[i].time;
			int tableIndex = ElapsedTime*this->N_elements*this->inv_maxTimeMeasured;
			float value = this->kernelLookupTable[tableIndex];

			// Update synaptic weight
			interi->IncrementWeight(this->a2prepre*value);
		}
	}
}