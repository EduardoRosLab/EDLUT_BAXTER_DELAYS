#!/usr/bin/env python

##**************************************************************************
 #                           system_time_comparer.py      		           *
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

# This node compares the current system time to another system's time.

import rospy
from std_msgs.msg import Time
from std_msgs.msg import Bool
import numpy as np

import edlut_ros.msg

class Time_Comparer(object):

	def __init__(self):
		self.pub = rospy.Publisher("time_difference_raspberry", edlut_ros.msg.Time, queue_size=10, tcp_nodelay=True)
		self.sub = rospy.Subscriber("time1", edlut_ros.msg.Time, self.TimeCallback, queue_size=10, tcp_nodelay=True)
		self.pub_average = rospy.Publisher("average_diff", edlut_ros.msg.Time, queue_size=10, tcp_nodelay=True)
		self.time = 0.0
		self.time_main_PC = 0.0
		self.msg = edlut_ros.msg.Time()
		self.msg_average = edlut_ros.msg.Time()
		self.rate = rospy.Rate(500)
		self.time_array = []

	def TimeCallback(self, data):
		self.time_main_PC = data.data
		self.msg.data = data.data - rospy.get_rostime().to_sec()
		self.msg.header.stamp = rospy.get_rostime()
		self.pub.publish(self.msg)
		self.time_array.append(self.msg.data)
		# self.GetAverage()


	def GetAverage(self):
		sum = 0
		for x in self.time_array:
			sum += x
		self.average = x / len(self.time_array)
		self.msg_average.data = self.average
		self.msg_average.header = rospy.get_rostime()
		self.pub_average.publish(self.msg_average)

def main():
	rospy.init_node('time_comparer', anonymous=True, disable_signals = True)
	time_comp = Time_Comparer()
	while not rospy.is_shutdown():
		time_comp.rate.sleep()
	rospy.signal_shutdown("enabling done")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
