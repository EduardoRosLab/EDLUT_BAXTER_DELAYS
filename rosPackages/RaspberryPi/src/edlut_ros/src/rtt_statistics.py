#!/usr/bin/env python

##**************************************************************************
 #                            Rtt_statistics.py                            *
 #                           -------------------                           *
 # copyright            : (C) 2019 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                             	   *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/


# This script is used to compute and plot the Mean Absolute Error (MAE)
# comparing the desired vs current state available at the full_state topic.

import time
import argparse

import rospy

import sys
import numpy
import matplotlib.pyplot as plt


import edlut_ros.msg


class Rtt_statistics(object):

	# Callback for the rtt data array
	def RttCallback(self, data):
		self.Rtt_msg.data = rospy.get_rostime().to_sec() - data.data[0]
		self.Rtt_msg.header.stamp = rospy.get_rostime()
		self.pub.publish(self.Rtt_msg)

		self.rtt_data.append(self.Rtt_msg.data)

	def GetMean(self):
		mean = numpy.mean(self.rtt_data)
		return mean

	def GetCDF(self):
		# sort the data:
		self.data_sorted = numpy.sort(self.rtt_data)

		# calculate the proportional values of samples
		self.p = 1. * numpy.arange(len(self.rtt_data)) / (len(self.rtt_data) - 1)

	def CDFStatistics(self):
		print "________________________________________________"
		print "ROUND TRIP TIME CUMULATIVE DISTRIBUTION FUNCTION"
		print "________________________________________________"
		print "	5%  --> " , self.data_sorted[(len(self.data_sorted) - 1) * 5 / 100]
		print "	10% --> " , self.data_sorted[(len(self.data_sorted) - 1) * 10 / 100]
		print "	20% --> " , self.data_sorted[(len(self.data_sorted) - 1) * 20 / 100]
		print "	40% --> " , self.data_sorted[(len(self.data_sorted) - 1) * 40 / 100]
		print "	50% --> " , self.data_sorted[(len(self.data_sorted) - 1) * 50 / 100]
		print "	70% --> " , self.data_sorted[(len(self.data_sorted) - 1) * 70 / 100]
		print "	80% --> " , self.data_sorted[(len(self.data_sorted) - 1) * 80 / 100]
		print "	90% --> " , self.data_sorted[(len(self.data_sorted) - 1) * 90 / 100]
		print "	95% --> " , self.data_sorted[(len(self.data_sorted) - 1) * 95 / 100]
		print "	100% --> " , self.data_sorted[len(self.data_sorted) - 1]
		print "________________________________________________"


	def __init__(self):
		rospy.init_node('rtt_stats', anonymous=True, disable_signals = True)

		#Retrieve RosLaunch parameters
		self.comeback_topic = rospy.get_param("~comeback_topic")
		self.rtt_topic = rospy.get_param("~rtt_topic")

		#Subscribing to the topic
		self.sub = rospy.Subscriber(self.comeback_topic, edlut_ros.msg.AnalogCompact, self.RttCallback, queue_size=None)

		# Create the publisher
		self.pub = rospy.Publisher(self.rtt_topic, edlut_ros.msg.Analog, queue_size=10, tcp_nodelay=True)

		self.rtt_data = []
		self.cdf = []

		self.data_sorted = []
		self.p = []

		self.Rtt_msg = edlut_ros.msg.Analog()

		rospy.spin()

	def plot(self):
		line0 = plt.plot(self.data_sorted, self.p, color='b')

		plt.legend()
		plt.show()


def main():
	rtt = Rtt_statistics()

	rtt.GetCDF()
	rtt.CDFStatistics()
	print "Average RTT = " , rtt.GetMean()
	rtt.plot()

	rospy.signal_shutdown("enabling done")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
