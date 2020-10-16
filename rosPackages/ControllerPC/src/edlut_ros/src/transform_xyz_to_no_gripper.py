#!/usr/bin/env python

##**************************************************************************
 #                           filter_data.py                         	   *
 #                           -------------------                           *
 # copyright            : (C) 2018 by Ignacio Abadia                       *
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

# This program takes the joint angles given in a file and returns a file with the
# joint velocities for that trajectory at a specified frequency.


import argparse
import struct
import sys
import numpy as np
import rospy
import math
import string
import os



class GetFile(object):

	def __init__(self, file_name):

		self.file = open(file_name, "r")
		new_file = file_name.replace(".txt", "") +"_noGripper.txt"
		self.new_file = open(new_file, "w")

		self.x=[]
		self.y=[]
		self.z=[]

		self.z_difference = 0.157707377

		self.read_file()


	def read_file(self):
		for line in self.file:
			position = line.split()
			self.x.append(float(position[0]))
			self.y.append(float(position[1]))
			self.z.append(float(position[2]))


	def get_z_no_gripper(self):
		for i in range(0, len(self.z)):
			# self.z[i] = self.z[i] + self.z_difference
			self.z[i] = -0.102

	def write_new_file(self):
		for i in range(0,len(self.x)):
			self.new_file.write(str(self.x[i])+" "+str(self.y[i])+" "+str(self.z[i])+"\n")

	def close_files(self):
		self.file.close()
		self.new_file.close()

def main():
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									 description=main.__doc__)

	parser.add_argument(
		'-f', '--file', help=".txt file with the data to be filtered"
	)

	args = parser.parse_args(rospy.myargv()[1:])

	xyz = GetFile(args.file)
	xyz.get_z_no_gripper()
	xyz.write_new_file()
	xyz.close_files()
	return 0


if __name__ == '__main__':
	sys.exit(main())
