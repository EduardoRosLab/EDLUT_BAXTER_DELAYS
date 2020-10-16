#!/usr/bin/env python

##**************************************************************************
 #                           joint_recorder_file_format.py                         	   *
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

# This script puts a trajectory .txt file written in format:
# J0, J1, J2, J3, J4, J5, J6
# into format:
# J0 J1 J2 J3 J4 J5 J6

import argparse
import struct
import sys
import numpy as np
import rospy
import math
import string
import os
from scipy.signal import savgol_filter

from math import pi, sqrt, cos, sin

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class InputActivity(object):

	def __init__(self, file_name):

		self.file = open(file_name, "r")
		new_file = file_name.replace(".txt", "") + "_formated.txt"
		self.new_file = open(new_file, "w")

		self.joint0=[]
		self.joint1=[]
		self.joint2=[]
		self.joint3=[]
		self.joint4=[]
		self.joint5=[]
		self.joint6=[]


	def read_file(self):
		for line in self.file:
			data = line.split(",")
			self.joint0.append(data[1])
			self.joint1.append(data[2])
			self.joint2.append(data[3])
			self.joint3.append(data[4])
			self.joint4.append(data[5])
			self.joint5.append(data[6])
			self.joint6.append(data[7])


	def write_new_file(self):
		for x in range(0,len(self.joint0)):
			self.new_file.write(str(self.joint0[x])+" "+str(self.joint1[x])+" "+str(self.joint2[x])+" "+str(self.joint3[x])+" "+
			str(self.joint4[x])+" "+str(self.joint5[x])+" "+str(self.joint6[x])+"\n")

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

	new_file = InputActivity(args.file)
	new_file.read_file()
	new_file.write_new_file()
	new_file.close_files()

	return 0


if __name__ == '__main__':
	sys.exit(main())
