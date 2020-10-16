/***************************************************************************
 *                           CommunicationAvatarVOR.cpp                    *
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

#include "../../include/vor_model/CommunicationAvatarVOR.h"
#include "../../include/communication/CdSocket.h"

#include "../../include/interface/C_interface_for_robot_control.h"

#include "../../include/simulation/Simulation.h"

#if (defined (_WIN32) || defined(_WIN64))
#include "windows.h"
#else 
#include <unistd.h>
#endif

#include <iostream>
#include <string>
#include <sstream> 
using namespace std;



CommunicationAvatarVOR::CommunicationAvatarVOR() :ConnectionStarted(false), ConnectionEnded(false), var_log(var_log), socket(0){
}

CommunicationAvatarVOR::~CommunicationAvatarVOR(){
	if (socket != 0){
		delete socket;
	}
}

void CommunicationAvatarVOR::Send(int index, int index1, int index2){
	cerebellum_Avatar_Data.time_index = index;
	if (NUM_JOINTS == 2){
		cerebellum_Avatar_Data.data[0] = var_log->regs[index1].vor_head_vars[0]; //horizontal head position
		cerebellum_Avatar_Data.data[1] = var_log->regs[index1].vor_head_vars[1]; //vertical head position
		cerebellum_Avatar_Data.data[2] = var_log->regs[index1].vor_head_vars[2]; //horizontal head velocity
		cerebellum_Avatar_Data.data[3] = var_log->regs[index1].vor_head_vars[3]; //vertical head velocity
		cerebellum_Avatar_Data.data[4] = var_log->regs[index2].vor_output_vars[0]; //horizontal eye position
		cerebellum_Avatar_Data.data[5] = var_log->regs[index2].vor_output_vars[1]; //vertical eye position
		cerebellum_Avatar_Data.data[6] = var_log->regs[index2].vor_output_vars[2]; //horizontal eye velocity
		cerebellum_Avatar_Data.data[7] = var_log->regs[index2].vor_output_vars[3]; //vertical eye velocity
	}
	else{
		cerebellum_Avatar_Data.data[0] = var_log->regs[index1].vor_head_vars[0]; //horizontal head position
		cerebellum_Avatar_Data.data[1] = 0;			                         //vertical head position
		cerebellum_Avatar_Data.data[2] = var_log->regs[index1].vor_head_vars[1]; //horizontal head velocity
		cerebellum_Avatar_Data.data[3] = 0;                                     //vertical head velocity
		cerebellum_Avatar_Data.data[4] = var_log->regs[index2].vor_output_vars[0]; //horizontal eye position
		cerebellum_Avatar_Data.data[5] = 0;                                       //vertical eye position
		cerebellum_Avatar_Data.data[6] = var_log->regs[index2].vor_output_vars[1]; //horizontal eye velocity
		cerebellum_Avatar_Data.data[7] = 0;                                       //vertical eye velocity
	}

	socket->sendBuffer(((void*)&cerebellum_Avatar_Data), sizeof(Cerebellum_Avatar_Data));
}

void CommunicationAvatarVOR::Receive(){
	int N_bytes = socket->receiveBuffer(((void*)&avatar_Cerebellum_Data), sizeof(Avatar_Cerebellum_Data));

	if (N_bytes != -1){
		float avatar_head_vars[4];
		float avatar_output_vars[4];

		if (NUM_JOINTS == 2){
			avatar_head_vars[0] = avatar_Cerebellum_Data.data[0]; //horizontal heap position
			avatar_head_vars[1] = avatar_Cerebellum_Data.data[1]; //vertical heap position
			avatar_head_vars[2] = avatar_Cerebellum_Data.data[2]; //horizontal heap velocity
			avatar_head_vars[3] = avatar_Cerebellum_Data.data[3]; //vertical heap velocity
			avatar_output_vars[0] = avatar_Cerebellum_Data.data[4]; //horizontal eye position
			avatar_output_vars[1] = avatar_Cerebellum_Data.data[5]; //vertical eye position
			avatar_output_vars[2] = avatar_Cerebellum_Data.data[6]; //horizontal eye velocity
			avatar_output_vars[3] = avatar_Cerebellum_Data.data[7]; //vertical eye velocity
		}
		else{
			avatar_head_vars[0] = avatar_Cerebellum_Data.data[0]; //horizontal heap position
			avatar_head_vars[1] = avatar_Cerebellum_Data.data[2]; //horizontal heap velocity
			avatar_output_vars[0] = avatar_Cerebellum_Data.data[4]; //horizontal eye position
			avatar_output_vars[1] = avatar_Cerebellum_Data.data[6]; //horizontal eye velocity
		}

		//Calculate the index inside the var_log structure where must be stored the avatar positions and velocities according to the real time simulation. This index is updated in realTimeRestriction object by other thread.
		int index = realTimeRestriction->GetRealTimeSimulationStepIndex();
		//This method store in var_log the real positions and velocities for that index. It also interpolates previous elements since the last update using the desired positions and velocities as reference.
		update_outer_loop_log_vars_vor(var_log, index, avatar_head_vars, avatar_output_vars);
	}
	else{
		printf("ERROR: CONNECTION WITH ROBOT INTERFACE CLOSSED\n");
		exit(0);
	}
}

//FOR SIMULATOR
void CommunicationAvatarVOR::Communication(string Avatar_IP, unsigned short tpc_port, RealTimeRestriction * newRealTimeRestriction, struct log_vor * new_var_log, int robot_processing_delay1, int robot_processing_delay2){
	realTimeRestriction = newRealTimeRestriction;
	var_log = new_var_log;
	
	//Create the socket connection and connect with the server.
	socket = new CdSocket(CLIENT, Avatar_IP, tpc_port);
	ConnectionStarted = true;


	
	int last_index = -1;
	//Deploy the communication until the simulation end.
	while (!ConnectionEnded){
		//Calculate the index inside the var_log structure that must be sent to the avatar according to the real time simulation. This index is updated in realTimeRestriction object by other thread.
		int index = realTimeRestriction->GetRealTimeSimulationStepIndex();
		//We add this additional element to compensate the processing delay that the avatar include in the control loop (time since the avatar receives a command until it processes this one).
		int index1=index+robot_processing_delay1;
		int index2=index+robot_processing_delay2;

		//Check if a new element in the structure must be processed.
		if (index > last_index){
			last_index = index;
			//Due to the robot_processing_delay, the first element can't be processed.
			if (var_log->nregs > robot_processing_delay1 && var_log->nregs > robot_processing_delay2){

				//Calculate if the elements that must be sent to the avatar have been already computed by other thread.
				if (index1 >= var_log->nregs){
					//We fix the index to the last element calculated.
					index1 = var_log->nregs-1;
				}
				if (index2 >= var_log->nregs){
					//We fix the index to the last element calculated.
					index2 = var_log->nregs-1;
				}

				//Send the desire position.
				Send(index, index1, index2);
				//Receive the real position.
				Receive();
			}
		}
		else{
			#if (defined (_WIN32) || defined(_WIN64))
				Sleep(1);
			#else
				usleep(1 * 1000);
			#endif
		}
	}

	//We close the connection.
	delete socket;
}


void CommunicationAvatarVOR::StopConnection(){
	ConnectionEnded = true;
}


bool CommunicationAvatarVOR::GetConnectionStarted(){
	return ConnectionStarted;
}
bool CommunicationAvatarVOR::GetConnectionEnded(){
	return ConnectionEnded;
}

