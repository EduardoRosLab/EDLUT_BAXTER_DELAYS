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


	# Callback for the sensorial data array
	def SensorialCallback(self, data):
		self.sensorial_delay_data.append(data.data)

	# Callback for the motor data array
	def MotorCallback(self, data):
		self.motor_delay_data.append(data.data)

	def GetMeanSensorial(self):
		mean = numpy.mean(self.sensorial_delay_data)
		return mean

	def GetMeanMotor(self):
		mean = numpy.mean(self.motor_delay_data)
		return mean

	def GetCDFSensorial(self):
		# sort the data:
		self.sensorial_delay_data_sorted = numpy.sort(self.sensorial_delay_data)

		# calculate the proportional values of samples
		self.sensorial_delay_p = 1. * numpy.arange(len(self.sensorial_delay_data)) / (len(self.sensorial_delay_data) - 1)

	def GetCDFMotor(self):
		# sort the data:
		self.motor_delay_data_sorted = numpy.sort(self.motor_delay_data)

		# calculate the proportional values of samples
		self.motor_delay_p = 1. * numpy.arange(len(self.motor_delay_data)) / (len(self.motor_delay_data) - 1)

	def CDFStatistics(self):

		print "________________________________________________"
		print "SENSORIAL DELAY CUMULATIVE DISTRIBUTION FUNCTION"
		print "________________________________________________"
		print "	5%  --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 5 / 100]
		print "	10% --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 10 / 100]
		print "	20% --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 20 / 100]
		print "	40% --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 40 / 100]
		print "	50% --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 50 / 100]
		print "	70% --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 70 / 100]
		print "	80% --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 80 / 100]
		print "	90% --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 90 / 100]
		print "	95% --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 95 / 100]
		print "	99% --> " , self.sensorial_delay_data_sorted[(len(self.sensorial_delay_data_sorted) - 1) * 99 / 100]
		print "	100% --> " , self.sensorial_delay_data_sorted[len(self.sensorial_delay_data_sorted) - 1]

		print "________________________________________________"
		print "MOTOR DELAY CUMULATIVE DISTRIBUTION FUNCTION"
		print "________________________________________________"
		print "	5%  --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 5 / 100]
		print "	10% --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 10 / 100]
		print "	20% --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 20 / 100]
		print "	40% --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 40 / 100]
		print "	50% --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 50 / 100]
		print "	70% --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 70 / 100]
		print "	80% --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 80 / 100]
		print "	90% --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 90 / 100]
		print "	95% --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 95 / 100]
		print "	99% --> " , self.motor_delay_data_sorted[(len(self.motor_delay_data_sorted) - 1) * 99 / 100]
		print "	100% --> " , self.motor_delay_data_sorted[len(self.motor_delay_data_sorted) - 1]


	def __init__(self):
		rospy.init_node('rtt_stats', anonymous=True, disable_signals = True)

		# #Retrieve RosLaunch parameters
		# self.comeback_topic = rospy.get_param("~comeback_topic")
		# self.rtt_topic = rospy.get_param("~rtt_topic")
		#
		# #Subscribing to the topic
		# self.sub = rospy.Subscriber(self.comeback_topic, edlut_ros.msg.AnalogCompact, self.RttCallback, queue_size=None)
		#
		# # Create the publisher
		# self.pub = rospy.Publisher(self.rtt_topic, edlut_ros.msg.Analog, queue_size=10, tcp_nodelay=True)

		self.sensorial_delay_topic = rospy.get_param("~sensorial_delay_topic")
		self.motor_delay_topic = rospy.get_param("~motor_delay_topic")

		#Subscribing to the topic
		self.sensorial_sub = rospy.Subscriber(self.sensorial_delay_topic, edlut_ros.msg.Analog, self.SensorialCallback, queue_size=None)
		self.motor_sub = rospy.Subscriber(self.motor_delay_topic, edlut_ros.msg.Analog, self.MotorCallback, queue_size=None)


		self.sensorial_delay_data = []
		self.sensorial_delay_cdf = []

		self.sensorial_delay_data_sorted = []
		self.sensorial_delay_p = []

		self.motor_delay_data = []
		self.motor_delay_cdf = []

		self.motor_delay_data_sorted = []
		self.motor_delay_p = []

		# self.Rtt_msg = edlut_ros.msg.Analog()

		rospy.spin()

	def plot(self):
		line0 = plt.plot(self.sensorial_delay_data_sorted, self.sensorial_delay_p, color='b')

		plt.legend()
		plt.show()


def main():
	rtt = Rtt_statistics()

	rtt.GetCDFSensorial()
	rtt.GetCDFMotor()
	rtt.CDFStatistics()
	# print "Average RTT = " , rtt.GetMean()
	# rtt.plot()

	rospy.signal_shutdown("enabling done")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
