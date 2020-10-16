#!/usr/bin/env python

##**************************************************************************
 #                           WriteCurrentPositionFile.py               	   *
 #                           -------------------                           *
 # copyright            : (C) 2020 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                                   *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/

# This program reads the desired position and current position topics and stores the data in two files
# To synchronize the data, the data is stored when the second iteration of the trajectory is performed


import struct
import sys
import rospy
import string
from edlut_ros.msg import AnalogCompactDelay


class WriteCurrentPositionFile(object):

	def __init__(self, current_topic, desired_topic, file_name_current, file_name_desired):

		self.file_current = open(file_name_current, "w")
		self.file_desired = open(file_name_desired, "w")


		self.input_current = rospy.Subscriber(current_topic, AnalogCompactDelay, self.CurrentCallback)
		self.input_desired = rospy.Subscriber(desired_topic, AnalogCompactDelay, self.DesiredCallback)


		self.current_queue = []
		self.desired_queue = []
		self.time_stamps_current = []
		self.time_stamps_desired = []

		self.first_iteration = False
		self.second_iteration = False

		self.iteration_start_point = [-0.76252975, -0.03976575, 0.033638525, 0.594384, -0.0174617, 1.014016]

	def DesiredCallback(self, data):
		input_msg_data = []
		joints = 0

		for x in data.data:
			input_msg_data.append(x)
		if (not self.first_iteration or not self.second_iteration):
			for x in range (0, len(input_msg_data)):
				if input_msg_data[x] == self.iteration_start_point[x]:
					joints += 1
			if joints == 5:
				if self.first_iteration:
					self.second_iteration = True
				else:
					self.first_iteration = True

		if self.second_iteration:
			self.desired_queue.append(input_msg_data)
			self.time_stamps_desired.append(data.header.stamp.to_sec())

	def CurrentCallback(self, data):
		input_msg_data = []
		for x in data.data:
			input_msg_data.append(x)

		if self.second_iteration:
			self.current_queue.append(input_msg_data)
			self.time_stamps_current.append(data.header.stamp.to_sec())

	def update_file(self):
		if self.current_queue:
			for x in range(0,len(self.current_queue)):
				self.file_current.write(str(self.current_queue[x][0])+" "+str(self.current_queue[x][1])+" "+str(self.current_queue[x][2])+" "+
				str(self.current_queue[x][3])+" "+str(self.current_queue[x][4])+" "+str(self.current_queue[x][5])+" 0.0 \n")
			self.current_queue = []
			self.time_stamps_current = []
		if self.desired_queue:
			for x in range(0,len(self.desired_queue)):
				self.file_desired.write(str(self.desired_queue[x][0])+" "+str(self.desired_queue[x][1])+" "+str(self.desired_queue[x][2])+" "+
				str(self.desired_queue[x][3])+" "+str(self.desired_queue[x][4])+" "+str(self.desired_queue[x][5])+" 0.0 \n")
			self.desired_queue = []
			self.time_stamps_desired = []

	def close_file(self):
		self.file_current.close()
		self.file_desired.close()


def main():
	print("Initializing node... ")
	rospy.init_node("write_current_position_file", anonymous=True)

	current_topic = rospy.get_param("~current_topic")
	desired_topic = rospy.get_param("~desired_topic")
	sampling_frequency = rospy.get_param("~sampling_frequency")
	file_name_current = rospy.get_param("~file_name_current")
	file_name_desired = rospy.get_param("~file_name_desired")

	write_file = WriteCurrentPositionFile(current_topic, desired_topic, file_name_current, file_name_desired)

	# register shutdown callback
	rospy.on_shutdown(write_file.close_file)
	control_rate = rospy.Rate(sampling_frequency)
	while not rospy.is_shutdown():
		write_file.update_file()
		control_rate.sleep()


if __name__ == '__main__':
	sys.exit(main())
