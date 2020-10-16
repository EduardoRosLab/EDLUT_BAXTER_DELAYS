/***************************************************************************
 *                           CommunicationAvatarVOR.h                      *
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

#ifndef COMMUNICATIONAVATARVOR_H_
#define COMMUNICATIONAVATARVOR_H_

/*!
 * \file CommunicationAvatarVOR.h
 *
 * \author Niceto Luque
 * \author Francisco Naveros
 * \date April 2016
 *
 * This file declares a class which implements a communication mechanism between EDLUT and a humanoid avatar for a VOR experiment.
 */

#include <string>

class CdSocket;
class RealTimeRestriction;
struct log_vor;

struct Cerebellum_Avatar_Data {
	/*!
	* Index that set to which simulation step correspond the data.
	*/
	int time_index;
	/*!
	* Buffer used to send to the avatar the desired positions and velocities (horizontal and vertical) of the head and eyes.
	*/
	float data[8];
};

struct Avatar_Cerebellum_Data {
	/*!
	* Buffer used to receive from the avatar the real positions and velocities (horizontal and vertical) of the head and eyes.
	*/
	float data[8];
};

using namespace std;



class CommunicationAvatarVOR{
	public:

		/*!
		* Socket for TCP IP communications.
		*/
		CdSocket * socket;

		/*!
		* This variable shows when the socket is initialized.
		*/
		volatile bool ConnectionStarted;

		/*!
		* This variable shows when the socket is clossed.
		*/
		volatile bool ConnectionEnded;

		/*!
		* Buffer used to send to the avatar the desired positions and velocities (horizontal and vertical) of the head and eyes. It includes also an additional index to measure the time.
		*/
		Cerebellum_Avatar_Data cerebellum_Avatar_Data;

		/*!
		* Buffer used to receive from the avatar the real positions and velocities (horizontal and vertical) of the head and eyes.
		*/
		Avatar_Cerebellum_Data avatar_Cerebellum_Data;
		
		/*!
		* Structure where the desired and real positions and velocities are stored. This structure is initialized in other object and must not be deleted.
		*/
		struct log_vor * var_log;

		/*!
		* Real time object. This object is initialized in other object and must not be deleted.
		*/
		RealTimeRestriction * realTimeRestriction;

   		/*!
   		 * \brief Default constructor.
   		 * 
   		 * It creates and initializes a new spike object.
   		 */
		CommunicationAvatarVOR();

   		/*!
   		 * \brief Class destructor.
   		 * 
   		 * It destroies an object of this class.
   		 */
		~CommunicationAvatarVOR();


		/*!
		* \brief It sends the desired positions and velocities to the avatar. It also sends a boolean setting if the communication must be clossed.
		*
		* It sends the desired positions and velocities to the avatar. It also sends a boolean setting if the communication must be clossed.
		*
		* \param index index inside the var_log structure that must be sent to the avatar.
		*/
		void Send(int index, int index1, int index2);

		/*!
		* \brief It receives the real positions and velocities from the avatar and store they in the var_log structure.
		*
		* It receives the real positions and velocities from the avatar and store they in the var_log structure.
		*/
		void Receive();


		/*!
		* \brief It initializes the TCP/IP socket, connect with the server and deploy the communication proccess with the avatar.
		*
		* It initializes the TCP/IP socket, connect with the server and deploy the communication proccess with the avatar.
		*
		* \param Avatar_IP IP direcction of server in Avatar interface.
		* \param tpc_port port of server in Avatar interface.
		* \param newRealTimeRestriction object that controles the relation between the simulation time and the real time for the avatar. 
		* \param new_var_log strucutre where is stored the desired and real positions and velocities of the avatar.
		* \param robot_processing_delay time since the avatar receives a command until it processes this one for the first motor.
		* \param robot_processing_delay time since the avatar receives a command until it processes this one for the second motor.
		*/
		void Communication(string Avatar_IP, unsigned short tpc_port, RealTimeRestriction * newRealTimeRestriction, struct log_vor * new_var_log, int robot_processing_delay1, int robot_processing_delay2);

		/*!
		* \brief It stops the communication. This methods is executed by the main thread that perform the VOR experiment.
		*
		* It stops the communication. This methods is executed by the main thread that perform the VOR experiment.
		*/
		void StopConnection();

		/*!
		* \brief It returns the ConnectionStarted variable.
		*
		* It returns the ConnectionStarted variable.
		*
		* \return ConnectionStarted.
		*/
		bool GetConnectionStarted();

		/*!
		* \brief It returns the ConnectionEnded variable.
		*
		* It returns the ConnectionEnded variable.
		*
		* \return ConnectionEnded.
		*/
		bool GetConnectionEnded();

};



#endif /* COMMUNICATIONAVATARVOR_H_ */
