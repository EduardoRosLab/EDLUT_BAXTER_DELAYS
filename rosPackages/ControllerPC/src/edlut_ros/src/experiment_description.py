#!/usr/bin/env python

##**************************************************************************
 #                           experiment_description.py                     *
 #                           -------------------                           *
 # copyright            : (C) 2019 by Ignacio Abadia                       *
 # email                : iabadia@ugr.es                      		       *
 #**************************************************************************/

##**************************************************************************
 #                                                                         *
 #   This program is free software; you can redistribute it and/or modify  *
 #   it under the terms of the GNU General Public License as published by  *
 #   the Free Software Foundation; either version 3 of the License, or     *
 #   (at your option) any later version.                                   *
 #                                                                         *
 #**************************************************************************/

# This node publishes a description of the current experiment

import rospy
from std_msgs.msg import String


class DescriptionPublisher(object):

	def __init__(self):
		self.outputTopic = rospy.get_param("~outputTopic")
		self.description = rospy.get_param("~description")

		# Publisher to advertise the experiment description
		self.pub = rospy.Publisher(self.outputTopic, String, queue_size=10, latch=True)

	def pubDescription(self):
		listened = False
		r = rospy.Rate(10)
		while not listened:
			if (self.pub.get_num_connections() > 0):
				self.pub.publish(self.description)
				listened = True
			r.sleep()

def main():
	rospy.init_node('description_node', anonymous=True, disable_signals = True)
	description = DescriptionPublisher()
	description.pubDescription()

	rospy.signal_shutdown("enabling done")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
