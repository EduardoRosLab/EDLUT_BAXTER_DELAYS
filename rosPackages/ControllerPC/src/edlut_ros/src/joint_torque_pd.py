#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import argparse

import rospy
import math


from std_msgs.msg import (
    Empty,
)

import baxter_interface

from baxter_core_msgs.msg import JointCommand

from baxter_interface import CHECK_VERSION
from edlut_ros.msg import AnalogCompact
from edlut_ros.msg import AnalogCompactDelay


from dynamic_reconfigure.server import Server
from edlut_ros.cfg import JointsPDConfig

class JointPD(object):

    def __init__(self, limb, joint_list, initial_kp, initial_kd, initial_max_torque,
                desired_position_topic, desired_velocity_topic, current_position_topic,
                current_velocity_topic, output_topic, reconfig_server, node_name,
                reset_values, sampling_frequency):

        # create our limb instance
        self._limb = baxter_interface.Limb(limb)

        self._dyn = reconfig_server
        # control parameters
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout
        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / sampling_frequency) * self._missed_cmds)

        self.node_name = node_name  #Node name, used to set the parameters values in reset_default_values()
        self.reset_values = reset_values #Whether or not to reset the PD constants to their initial values at the end of the experiment


        self.joint_list = joint_list
        self.desired_position_topic = desired_position_topic
        self.desired_velocity_topic = desired_velocity_topic
        self.current_position_topic = current_position_topic
        self.current_velocity_topic = current_velocity_topic
        self.output_topic = output_topic

        # initialize parameters
        self._kp = dict()
        self._kd = dict()
        self.desired_position = dict()
        self.desired_velocity = dict()
        self.current_position = dict()
        self.current_velocity = dict()
        self._max_torque = dict()
        self.initial_kp = dict()
        self.initial_kd = dict()
        self.initial_max_torque = dict()


        self._kp = self._limb.joint_angles()
        self._kd = self._limb.joint_angles()
        self.desired_position = self._limb.joint_angles()
        self.desired_velocity = self._limb.joint_angles()
        self.current_position = self._limb.joint_angles()
        self.current_velocity = self._limb.joint_angles()
        self._max_torque = self._limb.joint_angles()
        self.initial_kp = self._limb.joint_angles()
        self.initial_kd = self._limb.joint_angles()
        self.initial_max_torque = self._limb.joint_angles()


        for joint in (self.initial_kp):
            for j in range(0, len(self.joint_list)):
                if (joint == self.joint_list[j]):
                    self.initial_kp[joint] = initial_kp[j]
                    self._kp[joint] = initial_kp[j]
                    self.initial_kd[joint] = initial_kd[j]
                    self._kd[joint] = initial_kd[j]
                    self.initial_max_torque[joint] = initial_max_torque[j]
                    self._max_torque[joint] = initial_max_torque[j]


        self.init_params()

        self.desired_position_sub = rospy.Subscriber(self.desired_position_topic, AnalogCompactDelay, self.DesiredPositionCallback)
        self.desired_velocity_sub = rospy.Subscriber(self.desired_velocity_topic, AnalogCompactDelay, self.DesiredVelocityCallback)
        self.current_position_sub = rospy.Subscriber(self.current_position_topic, AnalogCompactDelay, self.CurrentPositionCallback)
        self.current_velocity_sub = rospy.Subscriber(self.current_velocity_topic, AnalogCompactDelay, self.CurrentVelocityCallback)
        self.output_pub = rospy.Publisher(self.output_topic, AnalogCompactDelay, queue_size=1)


        self.torque_msg = AnalogCompactDelay()
        self.torque_msg.names = self.joint_list
        self.torque_msg.data = [0] * len(self.joint_list)


        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")


    def init_params(self):
        print self.initial_kp
        print self._kp
        print self.initial_kd
        print self._kd
        print self.initial_max_torque
        print self._max_torque

    def reset_default_values(self):
        for joint in self._limb.joint_names():
            rospy.set_param(self.node_name + "/" + joint.replace("left_", "") + "_kp", self.initial_kp[joint])
            rospy.set_param(self.node_name + "/" + joint.replace("left_", "") + "_kd", self.initial_kd[joint])
            rospy.set_param(self.node_name + "/" + joint.replace("left_", "") + "_max_torque", self.initial_max_torque[joint])


    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._kp[joint] = self._dyn.config[joint[-2:] +'_kp']
            self._kd[joint] = self._dyn.config[joint[-2:] +'_kd']
            self._max_torque[joint] = self._dyn.config[joint[-2:] +'_max_torque']


    def DesiredPositionCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.desired_position.keys():
                if (x == i):
                    self.desired_position[i] = data.data[x_index]
            x_index+=1

    def CurrentPositionCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.current_position.keys():
                if (x == i):
                    self.current_position[i] = data.data[x_index]
            x_index+=1

    def DesiredVelocityCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.desired_velocity.keys():
                if (x == i):
                    self.desired_velocity[i] = data.data[x_index]
            x_index+=1

    def CurrentVelocityCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.current_velocity.keys():
                if (x == i):
                    self.current_velocity[i] = data.data[x_index]
            x_index+=1


    def _update_forces(self):
        """
        Calculates the current angular difference between the desired position
        and the current joint positions applying the joint torque PD
        """
        self._update_parameters()

        # calculate current forces
        for joint in self.desired_position.keys():
            x_index = 0
            for x in self.torque_msg.names:
                if (joint == x):
                    # KP portion
                    self.torque_msg.data[x_index] = self._kp[joint] * (self.desired_position[joint] - self.current_position[joint])
                    # KD portion
                    self.torque_msg.data[x_index] += self._kd[joint] * (self.desired_velocity[joint] - self.current_velocity[joint])

                    if(self.torque_msg.data[x_index]>self._max_torque[joint]):
                        self.torque_msg.data[x_index]=self._max_torque[joint]
                    if(self.torque_msg.data[x_index]<-self._max_torque[joint]):
                        self.torque_msg.data[x_index]=-self._max_torque[joint]

                x_index += 1

        self.torque_msg.delay = rospy.get_rostime().to_sec()
        self.torque_msg.header.stamp = rospy.get_rostime()
        self.output_pub.publish(self.torque_msg)


    def clean_shutdown(self):
        print("\nExiting torque PD control...")
        self._limb.exit_control_mode()
        if (self.reset_values):
            self.reset_default_values()

def main():
        print("Initializing node... ")
        rospy.init_node("torque_control", anonymous=True)

    	#Retrieve RosLaunch parameters
    	limb = rospy.get_param("~limb")
        joint_list = rospy.get_param("~joint_list")
        initial_kp = rospy.get_param("~initial_kp")
        initial_kd = rospy.get_param("~initial_kd")
        initial_max_torque = rospy.get_param("~initial_max_torque")
        reset_values = rospy.get_param("~reset_values")
        desired_position_topic = rospy.get_param("~desired_position_topic")
        desired_velocity_topic = rospy.get_param("~desired_velocity_topic")
        current_position_topic = rospy.get_param("~current_position_topic")
        current_velocity_topic = rospy.get_param("~current_velocity_topic")
        output_topic = rospy.get_param("~output_topic")
        sampling_frequency = rospy.get_param("~sampling_frequency")
        node_name = rospy.get_param("~node_name")

        dynamic_cfg_srv = Server(JointsPDConfig, lambda config, level: config)

        pd = JointPD(limb, joint_list, initial_kp, initial_kd, initial_max_torque,
            desired_position_topic, desired_velocity_topic, current_position_topic, current_velocity_topic,
            output_topic, dynamic_cfg_srv, node_name, reset_values, sampling_frequency)

        # register shutdown callback
        rospy.on_shutdown(pd.clean_shutdown)
        control_rate = rospy.Rate(sampling_frequency)
        while not rospy.is_shutdown():
            pd._update_forces()
            control_rate.sleep()


if __name__ == "__main__":
    main()
