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
from scipy.signal import savgol_filter

from math import pi, sqrt, cos, sin

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class GetVelocity(object):

	def __init__(self, file_name, trajectory_frequency):

		self.file = open(file_name, "r")
		new_file = file_name.replace(".txt", "") +"_velocity.txt"
		self.file_velocities = open(new_file, "w")

		self.joint0=[]
		self.joint1=[]
		self.joint2=[]
		self.joint3=[]
		self.joint4=[]
		self.joint5=[]
		self.joint6=[]

		self.joint0_vel=[]
		self.joint1_vel=[]
		self.joint2_vel=[]
		self.joint3_vel=[]
		self.joint4_vel=[]
		self.joint5_vel=[]
		self.joint6_vel=[]

		self.read_file()

		self.trajectory_frequency = trajectory_frequency
		self.samples = len(self.joint0)
		self.dt = 0.002

	def read_file(self):
		for line in self.file:
			angles = line.split()
			self.joint0.append(float(angles[0]))
			self.joint1.append(float(angles[1]))
			self.joint2.append(float(angles[2]))
			self.joint3.append(float(angles[3]))
			self.joint4.append(float(angles[4]))
			self.joint5.append(float(angles[5]))
			self.joint6.append(float(angles[6]))

	def get_velocity(self):
		for i in range(0, self.samples):
			if i == 0 or i==1 or i==(self.samples-2) or i == (self.samples - 1):
				self.joint0_vel.append(0)
				self.joint1_vel.append(0)
				self.joint2_vel.append(0)
				self.joint3_vel.append(0)
				self.joint4_vel.append(0)
				self.joint5_vel.append(0)
				self.joint6_vel.append(0)

			else:
				self.joint0_vel.append((self.joint0[i-2]*(-0.2) + self.joint0[i-1]*(-0.1) + self.joint0[i+1]*0.1 + self.joint0[i+2]*(0.2))/self.dt)
				self.joint1_vel.append((self.joint1[i-2]*(-0.2) + self.joint1[i-1]*(-0.1) + self.joint1[i+1]*0.1 + self.joint1[i+2]*(0.2))/self.dt)
				self.joint2_vel.append((self.joint2[i-2]*(-0.2) + self.joint2[i-1]*(-0.1) + self.joint2[i+1]*0.1 + self.joint2[i+2]*(0.2))/self.dt)
				self.joint3_vel.append((self.joint3[i-2]*(-0.2) + self.joint3[i-1]*(-0.1) + self.joint3[i+1]*0.1 + self.joint3[i+2]*(0.2))/self.dt)
				self.joint4_vel.append((self.joint4[i-2]*(-0.2) + self.joint4[i-1]*(-0.1) + self.joint4[i+1]*0.1 + self.joint4[i+2]*(0.2))/self.dt)
				self.joint5_vel.append((self.joint5[i-2]*(-0.2) + self.joint5[i-1]*(-0.1) + self.joint5[i+1]*0.1 + self.joint5[i+2]*(0.2))/self.dt)
				self.joint6_vel.append((self.joint6[i-2]*(-0.2) + self.joint6[i-1]*(-0.1) + self.joint6[i+1]*0.1 + self.joint6[i+2]*(0.2))/self.dt)

		print "DT", self.dt

	def filter(self):
		self.joint0_vel=savgol_filter(self.joint0_vel, 53, 3)
		self.joint1_vel=savgol_filter(self.joint1_vel, 53, 3)
		self.joint2_vel=savgol_filter(self.joint2_vel, 53, 3)
		self.joint3_vel=savgol_filter(self.joint3_vel, 53, 3)
		self.joint4_vel=savgol_filter(self.joint4_vel, 53, 3)
		self.joint5_vel=savgol_filter(self.joint5_vel, 53, 3)
		self.joint6_vel=savgol_filter(self.joint6_vel, 53, 3)

		self.write_new_file()
		self.close_files()

	def write_new_file(self):
		for x in range(0,len(self.joint0_vel)):
			self.file_velocities.write(str(self.joint0_vel[x])+" "+str(self.joint1_vel[x])+" "+str(self.joint2_vel[x])+" "+
			str(self.joint3_vel[x])+" "+str(self.joint4_vel[x])+" "+str(self.joint5_vel[x])+" "+str(self.joint6_vel[x])+"\n")

	def close_files(self):
		self.file.close()
		self.file_velocities.close()

def main():
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									 description=main.__doc__)

	parser.add_argument(
		'-f', '--file', help=".txt file with the data to be filtered"
	)
	parser.add_argument(
		'-freq', '--frequency', type=float, help="Trajectory frequency"
	)
	args = parser.parse_args(rospy.myargv()[1:])

	vel = GetVelocity(args.file, args.frequency)
	vel.get_velocity()
	vel.write_new_file()
	vel.close_files()
	return 0


if __name__ == '__main__':
	sys.exit(main())
