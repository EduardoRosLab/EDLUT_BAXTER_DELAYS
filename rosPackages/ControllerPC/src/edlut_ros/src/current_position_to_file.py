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

# This program reads a topic a writes it to a file


import struct
import sys
import rospy
import string
from edlut_ros.msg import AnalogCompactDelay


class WriteCurrentPositionFile(object):

	def __init__(self, input_topic, file_name):

		self.file = open(file_name, "w")

		self.input_sub = rospy.Subscriber(input_topic, AnalogCompactDelay, self.EventCallback)

		self.current_queue = []
		self.time_stamps = []


	def EventCallback(self, data):
		input_msg_data = []
		for x in data.data:
			input_msg_data.append(x)
		self.current_queue.append(input_msg_data)
		self.time_stamps.append(data.header.stamp.to_sec())

	def update_file(self):
		for x in range(0,len(self.current_queue)):
			# self.file.write(str(self.time_stamps[x])+" "+str(self.current_queue[x][0])+" "+str(self.current_queue[x][1])+" "+str(self.current_queue[x][2])+" "+
			# str(self.current_queue[x][3])+" "+str(self.current_queue[x][4])+" "+str(self.current_queue[x][5])+" "+str(self.current_queue[x][6])+"\n")
			self.file.write(str(self.time_stamps[x])+" "+str(self.current_queue[x][0])+" "+str(self.current_queue[x][1])+" "+str(self.current_queue[x][2])+" "+
			str(self.current_queue[x][3])+" "+str(self.current_queue[x][4])+" "+str(self.current_queue[x][5])+"\n")
		self.current_queue = []
		self.time_stamps = []

	def close_file(self):
		self.file.close()

def main():
	print("Initializing node... ")
	rospy.init_node("write_current_position_file", anonymous=True)

	input_topic = rospy.get_param("~input_topic")
	sampling_frequency = rospy.get_param("~sampling_frequency")
	file_name = rospy.get_param("~file_name")

	write_file = WriteCurrentPositionFile(input_topic, file_name)

	# register shutdown callback
	rospy.on_shutdown(write_file.close_file)
	control_rate = rospy.Rate(sampling_frequency)
	while not rospy.is_shutdown():
		write_file.update_file()
		control_rate.sleep()


if __name__ == '__main__':
	sys.exit(main())
