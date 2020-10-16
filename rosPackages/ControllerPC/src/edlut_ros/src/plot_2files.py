#!/usr/bin/env python

##**************************************************************************
 #                           plot_2files.py                         	   *
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


import argparse
import struct
import sys
import numpy as np
import rospy
import math
import string



from math import pi, sqrt, cos, sin
import matplotlib.pyplot as plt

class PlotFile(object):

	def __init__(self, file_name, file_name2):

		self.file = open(file_name, "r")
		self.file2 = open(file_name2, "r")



		self.joint0=[]
		self.joint1=[]
		self.joint2=[]
		self.joint3=[]
		self.joint4=[]
		self.joint5=[]
		self.joint6=[]

		self.joint0_2=[]
		self.joint1_2=[]
		self.joint2_2=[]
		self.joint3_2=[]
		self.joint4_2=[]
		self.joint5_2=[]
		self.joint6_2=[]

		self.read_files()
		self.close_files()

		self.samples = len(self.joint0)


	def read_files(self):
		for line in self.file:
			angles = line.split()
			self.joint0.append(float(angles[0]))
			self.joint1.append(float(angles[1]))
			self.joint2.append(float(angles[2]))
			self.joint3.append(float(angles[3]))
			self.joint4.append(float(angles[4]))
			self.joint5.append(float(angles[5]))
			self.joint6.append(float(angles[6]))

		for line in self.file2:
			angles = line.split()
			self.joint0_2.append(float(angles[0]))
			self.joint1_2.append(float(angles[1]))
			self.joint2_2.append(float(angles[2]))
			self.joint3_2.append(float(angles[3]))
			self.joint4_2.append(float(angles[4]))
			self.joint5_2.append(float(angles[5]))
			self.joint6_2.append(float(angles[6]))

	def close_files(self):
		self.file.close()
		self.file2.close()


	def plot(self):
		line0 = plt.plot(self.joint0, label="s0", color='b')
		line1 = plt.plot(self.joint1, label="s1", color='g')
		line2 = plt.plot(self.joint2, label="e0", color='r')
		line3 = plt.plot(self.joint3, label="e1", color='y')
		line4 = plt.plot(self.joint4, label="w0", color='c')
		line5 = plt.plot(self.joint5, label="w1", color='m')
		line6 = plt.plot(self.joint6, label="w2", color='k')

		line0 = plt.plot(self.joint0_2, label="s0", ls="--", color='b')
		line1 = plt.plot(self.joint1_2, label="s1", ls="--", color='g')
		line2 = plt.plot(self.joint2_2, label="e0", ls="--", color='r')
		line3 = plt.plot(self.joint3_2, label="e1", ls="--", color='y')
		line4 = plt.plot(self.joint4_2, label="w0", ls="--", color='c')
		line5 = plt.plot(self.joint5_2, label="w1", ls="--", color='m')
		line6 = plt.plot(self.joint6_2, label="w2", ls="--", color='k')

		plt.legend()
		plt.show()



def main():
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									 description=main.__doc__)

	parser.add_argument(
		'-f', '--file', help=".txt file with the data to be filtered"
	)

	parser.add_argument(
		'-f2', '--file2', help=".txt file with the data to be filtered"
	)

	args = parser.parse_args(rospy.myargv()[1:])

	draw = PlotFile(args.file, args.file2)
	draw.plot()

	return 0


if __name__ == '__main__':
	sys.exit(main())
