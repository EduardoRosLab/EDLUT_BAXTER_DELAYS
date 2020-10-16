#!/usr/bin/env python

##**************************************************************************
 #                           filter_data.py                         	   *
 #                           -------------------                           *
 # copyright            : (C) 2018 by Francisco Naveros                    *
 # email                : fnaveros@ugr.es                                  *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/

# This program takes a file (generated with moveit.cpp) with two equal and
# consecutive trajectories each of them performed in a period T. The program
# returns two files: one with the final positions of a single trajectory making
# sure that the start point and the end point are the same. The other file
# contains the velocities for that trajectory and the specified period.


import argparse
import struct
import sys
import numpy as np
import rospy
import math
import string
import os

from math import pi, sqrt, cos, sin

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class SuperFilter(object):

	def __init__(self, file_name, period):

		self.file = open(file_name, "r")
		file_position_name = file_name.replace(".txt", "") +"_position.txt"
		file_velocity_name = file_name.replace(".txt", "") +"_velocity.txt"
		self.file_positions = open(file_position_name, "w")
		self.file_velocities = open(file_velocity_name, "w")

		self.remove = 250
		self.joint0=[]
		self.joint1=[]
		self.joint2=[]
		self.joint3=[]
		self.joint4=[]
		self.joint5=[]
		self.joint6=[]

		self.joint0_pos=[]
		self.joint1_pos=[]
		self.joint2_pos=[]
		self.joint3_pos=[]
		self.joint4_pos=[]
		self.joint5_pos=[]
		self.joint6_pos=[]

		self.joint0_vel=[]
		self.joint1_vel=[]
		self.joint2_vel=[]
		self.joint3_vel=[]
		self.joint4_vel=[]
		self.joint5_vel=[]
		self.joint6_vel=[]

		self.read_file()

		self.num_samples = len(self.joint0)
		self.num_one_trajectory_samples = self.num_samples / 2
		self.num_half_trajectory_samples = self.num_one_trajectory_samples / 2
		self.num_one_trajectory_samples_f = self.num_one_trajectory_samples*1.0
		self.num_half_trajectory_samples_f = self.num_half_trajectory_samples*1.0

		self.dt = period / (self.num_samples / 2)

	def do_super_filter(self):
		self.read_file()
		self.rectification()
		self.get_velocity()
		self.write_files()
		self.close_files()

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

	def rectification(self):

		offset=[]

		offset.append(self.joint0[self.num_one_trajectory_samples + self.remove] - self.joint0[self.remove])
		offset.append(self.joint1[self.num_one_trajectory_samples + self.remove] - self.joint1[self.remove])
		offset.append(self.joint2[self.num_one_trajectory_samples + self.remove] - self.joint2[self.remove])
		offset.append(self.joint3[self.num_one_trajectory_samples + self.remove] - self.joint3[self.remove])
		offset.append(self.joint4[self.num_one_trajectory_samples + self.remove] - self.joint4[self.remove])
		offset.append(self.joint5[self.num_one_trajectory_samples + self.remove] - self.joint5[self.remove])
		offset.append(self.joint6[self.num_one_trajectory_samples + self.remove] - self.joint6[self.remove])

		for i in range(0, self.num_one_trajectory_samples):
			self.joint0_pos.append(self.joint0[i+self.remove] - i*offset[0]/(self.num_one_trajectory_samples))
			self.joint1_pos.append(self.joint1[i+self.remove] - i*offset[1]/(self.num_one_trajectory_samples))
			self.joint2_pos.append(self.joint2[i+self.remove] - i*offset[2]/(self.num_one_trajectory_samples))
			self.joint3_pos.append(self.joint3[i+self.remove] - i*offset[3]/(self.num_one_trajectory_samples))
			self.joint4_pos.append(self.joint4[i+self.remove] - i*offset[4]/(self.num_one_trajectory_samples))
			self.joint5_pos.append(self.joint5[i+self.remove] - i*offset[5]/(self.num_one_trajectory_samples))
			self.joint6_pos.append(self.joint6[i+self.remove] - i*offset[6]/(self.num_one_trajectory_samples))

		offset2=[]
		offset2.append(((self.joint0_pos[0]-self.joint0_pos[self.num_one_trajectory_samples-1]) - (self.joint0_pos[1]-self.joint0_pos[0]))/1.0)
		offset2.append(((self.joint1_pos[0]-self.joint1_pos[self.num_one_trajectory_samples-1]) - (self.joint1_pos[1]-self.joint1_pos[0]))/1.0)
		offset2.append(((self.joint2_pos[0]-self.joint2_pos[self.num_one_trajectory_samples-1]) - (self.joint2_pos[1]-self.joint2_pos[0]))/1.0)
		offset2.append(((self.joint3_pos[0]-self.joint3_pos[self.num_one_trajectory_samples-1]) - (self.joint3_pos[1]-self.joint3_pos[0]))/1.0)
		offset2.append(((self.joint4_pos[0]-self.joint4_pos[self.num_one_trajectory_samples-1]) - (self.joint4_pos[1]-self.joint4_pos[0]))/1.0)
		offset2.append(((self.joint5_pos[0]-self.joint5_pos[self.num_one_trajectory_samples-1]) - (self.joint5_pos[1]-self.joint5_pos[0]))/1.0)
		offset2.append(((self.joint6_pos[0]-self.joint6_pos[self.num_one_trajectory_samples-1]) - (self.joint6_pos[1]-self.joint6_pos[0]))/1.0)

		r=math.sqrt(self.num_half_trajectory_samples_f*self.num_half_trajectory_samples_f + self.num_half_trajectory_samples_f*self.num_half_trajectory_samples_f)/2.0

		for i in range(0, self.num_one_trajectory_samples):
			x=(((i)/self.num_one_trajectory_samples_f))
			if (i >= self.num_one_trajectory_samples/4 and i< 3*self.num_one_trajectory_samples/4 ):
				x*=math.sqrt(r*r - (self.num_half_trajectory_samples_f-i)*(self.num_half_trajectory_samples_f-i))
			else:
				x*=min(i, self.num_one_trajectory_samples-i)

			self.joint0_pos[i]+= x*offset2[0]
			self.joint1_pos[i]+= x*offset2[1]
			self.joint2_pos[i]+= x*offset2[2]
			self.joint3_pos[i]+= x*offset2[3]
			self.joint4_pos[i]+= x*offset2[4]
			self.joint5_pos[i]+= x*offset2[5]
			self.joint6_pos[i]+= x*offset2[6]

		for i in range(0, self.num_one_trajectory_samples):
			self.joint0_pos.append(self.joint0_pos[i])
			self.joint1_pos.append(self.joint1_pos[i])
			self.joint2_pos.append(self.joint2_pos[i])
			self.joint3_pos.append(self.joint3_pos[i])
			self.joint4_pos.append(self.joint4_pos[i])
			self.joint5_pos.append(self.joint5_pos[i])
			self.joint6_pos.append(self.joint6_pos[i])

	def get_velocity(self):
		for i in range(0, self.num_samples):
			if i == 0 or i==1 or i==(self.num_samples-3) or i==(self.num_samples-2) or i == (self.num_samples - 1):
				self.joint0_vel.append(0)
				self.joint1_vel.append(0)
				self.joint2_vel.append(0)
				self.joint3_vel.append(0)
				self.joint4_vel.append(0)
				self.joint5_vel.append(0)
				self.joint6_vel.append(0)

			else:
				self.joint0_vel.append((self.joint0_pos[i-2]*(-0.2) + self.joint0_pos[i-1]*(-0.1) + self.joint0_pos[i+1]*0.1 + self.joint0_pos[i+2]*(0.2))/self.dt)
				self.joint1_vel.append((self.joint1_pos[i-2]*(-0.2) + self.joint1_pos[i-1]*(-0.1) + self.joint1_pos[i+1]*0.1 + self.joint1_pos[i+2]*(0.2))/self.dt)
				self.joint2_vel.append((self.joint2_pos[i-2]*(-0.2) + self.joint2_pos[i-1]*(-0.1) + self.joint2_pos[i+1]*0.1 + self.joint2_pos[i+2]*(0.2))/self.dt)
				self.joint3_vel.append((self.joint3_pos[i-2]*(-0.2) + self.joint3_pos[i-1]*(-0.1) + self.joint3_pos[i+1]*0.1 + self.joint3_pos[i+2]*(0.2))/self.dt)
				self.joint4_vel.append((self.joint4_pos[i-2]*(-0.2) + self.joint4_pos[i-1]*(-0.1) + self.joint4_pos[i+1]*0.1 + self.joint4_pos[i+2]*(0.2))/self.dt)
				self.joint5_vel.append((self.joint5_pos[i-2]*(-0.2) + self.joint5_pos[i-1]*(-0.1) + self.joint5_pos[i+1]*0.1 + self.joint5_pos[i+2]*(0.2))/self.dt)
				self.joint6_vel.append((self.joint6_pos[i-2]*(-0.2) + self.joint6_pos[i-1]*(-0.1) + self.joint6_pos[i+1]*0.1 + self.joint6_pos[i+2]*(0.2))/self.dt)

		print "DT", self.dt


	def write_files(self):
		offset2 = (self.num_one_trajectory_samples / 2) - self.remove
		for x in range(0, self.num_one_trajectory_samples):
			self.file_positions.write(str(self.joint0_pos[x + offset2])+" "+str(self.joint1_pos[x + offset2])+" "+str(self.joint2_pos[x + offset2])+" "+
			str(self.joint3_pos[x + offset2])+" "+str(self.joint4_pos[x + offset2])+" "+str(self.joint5_pos[x + offset2])+" "+str(self.joint6_pos[x + offset2])+"\n")


		for x in range(0, self.num_one_trajectory_samples):
			self.file_velocities.write(str(self.joint0_vel[x + offset2])+" "+str(self.joint1_vel[x + offset2])+" "+str(self.joint2_vel[x + offset2])+" "+
			str(self.joint3_vel[x + offset2])+" "+str(self.joint4_vel[x + offset2])+" "+str(self.joint5_vel[x + offset2])+" "+str(self.joint6_vel[x + offset2])+"\n")


	def close_files(self):
		self.file.close()
		self.file_positions.close()
		self.file_velocities.close()

def main():
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									 description=main.__doc__)

	parser.add_argument(
		'-f', '--file', help=".txt file with the data to be filtered"
	)
	parser.add_argument(
		'-T', '--period', type=float, help="trajectory period"
	)
	args = parser.parse_args(rospy.myargv()[1:])

	filter = SuperFilter(args.file, args.period)
	filter.do_super_filter()

	return 0


if __name__ == '__main__':
	sys.exit(main())
