#!/usr/bin/env python

##**************************************************************************
 #                           xyz_publisher.py      		                   *
 #                           -------------------                           *
 # copyright            : (C) 2019 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                        	       *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/

# This node publishes the robot's endpoint position.

import rospy
from std_msgs.msg import Time
from std_msgs.msg import Bool
import numpy as np
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import (
	JointCommand,
)
import edlut_ros.msg


class Endpoint_record(object):

	def __init__(self):
		self.limb_name = rospy.get_param("~limb")
		self.rate = rospy.get_param("~rate")
		self.samples = rospy.get_param("~samples")
		self.output_file_name = rospy.get_param("~output_file")

		self.output_file = open(self.output_file_name, "w")


		# Baxter limb to have access to the robot's position
		self.limb = baxter_interface.Limb(self.limb_name)

		self.x = []
		self.y = []
		self.z = []
		self.data = []
		self.time = 0.0
		self.rate = rospy.Rate(500)

		self.done = False

	def GetEndPoint(self):
		if not self.done:
			coordinates = self.limb.endpoint_pose()
			self.x.append(coordinates["position"][0])
			self.y.append(coordinates["position"][1])
			self.z.append(coordinates["position"][2])

		if len(self.x)>=self.samples:
			self.done = True

		self.rate.sleep()

	def WriteFile(self):
		for x in range(0,len(self.x)):
			self.output_file.write(str(self.x[x])+" "+str(self.y[x])+" "+str(self.z[x])+"\n")

	def DoneWriting(self):
		return self.done

def main():
	rospy.init_node('write_xyz_file', anonymous=True, disable_signals = True)
	xyz = Endpoint_record()
	rospy.Rate(0.2).sleep()
	while not xyz.DoneWriting():
		xyz.GetEndPoint()
	xyz.WriteFile()
	rospy.signal_shutdown("writing done")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
