#!/usr/bin/env python

##**************************************************************************
 #                           plot_3Dcircle.py                             *
 #                           -------------------                           *
 # copyright            : (C) 2018 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es           		                   *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class PlotCircle3D(object):

	def __init__(self):
		self.max_x = -1.0
		self.max_y = -1.0
		self.max_z = -1.0
		self.min_x = 1.0
		self.min_y = 1.0
		self.min_z = 1.0

		#Cartesian coordinates of the desired trajectory
		self.desired_x = []
		self.desired_y = []
		self.desired_z = []

		self.desired_x2 = []
		self.desired_y2 = []
		self.desired_z2 = []

		self.desired_x3 = []
		self.desired_y3 = []
		self.desired_z3 = []

		self.samples = 1000
		self.radius = 0.12
		self.CreateCircle()
		# self.CreateCircle2()
		# self.CreateCircle3()

	def CreateCircle(self):
		dt = 2*math.pi / self.samples
		for i in range (0, self.samples):
			self.desired_x.append(self.radius * math.cos(math.pi + i*dt) * math.cos(5*math.pi/36.0))
			self.desired_y.append(self.radius * math.sin(math.pi + i*dt))
			self.desired_z.append(self.radius * math.cos(math.pi + i*dt) * math.sin(5*math.pi/36.0))
			# self.desired_z.append((1/(4*math.pi) - math.pow(self.desired_x[i] , 2) - math.pow(self.desired_y[i] , 2)))

	def CreateCircle2(self):
		dt = 2*math.pi / self.samples
		for i in range (0, self.samples):
			self.desired_x2.append(self.radius * math.cos(math.pi + i*dt))
			self.desired_y2.append(self.radius * math.sin(math.pi + i*dt))
			self.desired_z2.append(0)

	def CreateCircle3(self):
		dt = 2*math.pi / self.samples
		for i in range (0, self.samples):
			self.desired_x3.append(0)
			self.desired_y3.append(self.radius * math.sin(math.pi + i*dt))
			self.desired_z3.append(self.radius * math.cos(math.pi + i*dt))

	def ReadFile(self):
	 	for line in self.trajectory_file:
	 		coordinates = line.split()
			x = float(coordinates[0])
			y = float(coordinates[1])
			z = float(coordinates[2])
	 		self.desired_x.append(x)
	 		self.desired_y.append(y)
	 		self.desired_z.append(z)
			if x > self.max_x:
				self.max_x = x
			elif x < self.min_x:
				self.min_x = x
			if y > self.max_y:
				self.max_y = y
			elif y < self.min_y:
				self.min_y = y
			if z > self.max_z:
				self.max_z = z
			elif z < self.min_z:
				self.min_z = z

	def Draw(self):
		fig = plt.figure()
		ax=fig.gca(projection='3d')
		ax.plot(self.desired_x, self.desired_y, self.desired_z, color='b')
		ax.plot(self.desired_x2, self.desired_y2, self.desired_z2, color='r')
		ax.plot(self.desired_x3, self.desired_y3, self.desired_z3, color='g')
		plt.show()

def main():
	figure = PlotCircle3D()
	figure.Draw()
	return 0

if __name__ == '__main__':
	sys.exit(main())
