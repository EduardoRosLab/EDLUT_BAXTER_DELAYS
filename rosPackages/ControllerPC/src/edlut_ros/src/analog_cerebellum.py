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
import numpy as np


from std_msgs.msg import (
    Empty,
)

import baxter_interface

from baxter_core_msgs.msg import JointCommand

from baxter_interface import CHECK_VERSION
from edlut_ros.msg import AnalogCompact
from edlut_ros.msg import AnalogCompactDelay


#from dynamic_reconfigure.server import Server      # NO LO NECESITAMOS
#from edlut_ros.cfg import JointsPDConfig           # NO LO NECESITAMOS

class AnalogCerebellum(object):

    def __init__(self, limb, joint_list, desired_position_topic, desired_velocity_topic,
            current_position_topic, current_velocity_topic, output_topic, n_states_per_inputs,
            max_pos_amplitude, min_pos_amplitude, max_vel_amplitude, min_vel_amplitude,
            pos_error_gain, vel_error_gain, output_torque_gain, sampling_frequency, node_name):

        # create our limb instance
        self.limb = baxter_interface.Limb(limb)

        # self._dyn = reconfig_server # NO LO NECESITAMOS
        # control parameters
        self.missed_cmds = 20.0  # Missed cycles before triggering timeout
        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self.limb.set_command_timeout((1.0 / sampling_frequency) * self.missed_cmds)

        self.node_name = node_name  #Node name, used to set the parameters values in reset_default_values()


        self.joint_list = joint_list
        self.desired_position_topic = desired_position_topic
        self.desired_velocity_topic = desired_velocity_topic
        self.current_position_topic = current_position_topic
        self.current_velocity_topic = current_velocity_topic
        self.output_topic = output_topic
        self.n_states_per_inputs = n_states_per_inputs


        # initialize parameters
        self.max_pos_amplitude = dict()
        self.min_pos_amplitude = dict()
        self.max_vel_amplitude = dict()
        self.min_vel_amplitude = dict()
        self.pos_error_gain = dict()
        self.vel_error_gain = dict()
        self.output_torque_gain = dict()
        self.desired_position = dict()
        self.desired_velocity = dict()
        self.current_position = dict()
        self.current_velocity = dict()
        self.desired_position_index = dict()
        self.desired_velocity_index = dict()
        self.current_position_index = dict()
        self.current_velocity_index = dict()
        self.error = dict()


        self.max_pos_amplitude = self.limb.joint_angles()
        self.min_pos_amplitude = self.limb.joint_angles()
        self.max_vel_amplitude = self.limb.joint_angles()
        self.min_vel_amplitude = self.limb.joint_angles()
        self.pos_error_gain = self.limb.joint_angles()
        self.vel_error_gain = self.limb.joint_angles()
        self.output_torque_gain = self.limb.joint_angles()
        self.desired_position = self.limb.joint_angles()
        self.desired_velocity = self.limb.joint_angles()
        self.current_position = self.limb.joint_angles()
        self.current_velocity = self.limb.joint_angles()
        self.desired_position_index = self.limb.joint_angles()
        self.desired_velocity_index = self.limb.joint_angles()
        self.current_position_index = self.limb.joint_angles()
        self.current_velocity_index = self.limb.joint_angles()
        self.error = self.limb.joint_angles()


        for joint in (self.max_pos_amplitude):
            for j in range(0, len(self.joint_list)):
                if (joint == self.joint_list[j]):
                    self.max_pos_amplitude[joint] = max_pos_amplitude[j]
                    self.min_pos_amplitude[joint] = min_pos_amplitude[j]
                    self.max_vel_amplitude[joint] = max_vel_amplitude[j]
                    self.min_vel_amplitude[joint] = min_vel_amplitude[j]
                    self.pos_error_gain[joint] = pos_error_gain[j]
                    self.vel_error_gain[joint] = vel_error_gain[j]
                    self.output_torque_gain[joint] = output_torque_gain[j]
                    self.error[joint] = 0.0


        #subscribers
        self.desired_position_sub = rospy.Subscriber(self.desired_position_topic, AnalogCompactDelay, self.DesiredPositionCallback)
        self.desired_velocity_sub = rospy.Subscriber(self.desired_velocity_topic, AnalogCompactDelay, self.DesiredVelocityCallback)
        self.current_position_sub = rospy.Subscriber(self.current_position_topic, AnalogCompactDelay, self.CurrentPositionCallback)
        self.current_velocity_sub = rospy.Subscriber(self.current_velocity_topic, AnalogCompactDelay, self.CurrentVelocityCallback)
        self.output_pub = rospy.Publisher(self.output_topic, AnalogCompactDelay, queue_size=1)  #NO SE SI DEBERIA SER SIN EL DELAY.


        self.torque_msg = AnalogCompactDelay() #NO SE SI DEBERIA SER SIN EL DELAY.
        self.torque_msg.names = self.joint_list
        self.torque_msg.data = [0] * len(self.joint_list)

        #Array in which the current and desired position and velocity values are stored.
        #self.desired_position = [0] * len(self.joint_list)
        #self.desired_velocity = [0] * len(self.joint_list)
        #self.current_position = [0] * len(self.joint_list)
        #self.current_velocity = [0] * len(self.joint_list)

        #Array in which analog signals are "digitalised" (taking values between 0 and n_states_per_inputs - 1).
        #self.desired_position_index = [0] * len(self.joint_list)
        #self.desired_velocity_index = [0] * len(self.joint_list)
        #self.current_position_index = [0] * len(self.joint_list)
        #self.current_velocity_index = [0] * len(self.joint_list)

        #self.error = [0] * len(self.joint_list)


        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")


        ############CEREBELLAR STRUCTURE######################
        self.N_joints = len(self.joint_list)
        #self.LTP = 0.0010
        #self.LTD = 0.0050
        self.LTP = 0.0000025
        self.LTD = 0.000020

        self.Initial_GrC_weight = 0.20/(self.n_states_per_inputs*4)
        self.Max_GrC_weight = self.Initial_GrC_weight * 1.5

        self.N_MF = 1
        self.N_GrC = self.N_joints * self.n_states_per_inputs * self.n_states_per_inputs * self.n_states_per_inputs * self.n_states_per_inputs
        self.N_PC = self.N_joints * 2 #one agonist and one antagonist for each joint
        self.N_IO = self.N_joints * 2 #one agonist and one antagonist for each joint
        self.N_DCN = self.N_joints * 2 #one agonist and one antagonist for each joint

        self.MF_activity = [1.0] * self.N_MF
        self.GrC_activity_index = [0] * self.N_joints*self.n_states_per_inputs*4
        self.PC_activity = [0.0] * self.N_PC
        self.IO_activity = [0.0] * self.N_IO
        self.DCN_activity = [0.0] * self.N_DCN

        self.MF_DCN_weights = [1.0] * self.N_DCN
        self.GrC_PC_weights = [[self.Initial_GrC_weight for x in range(0, self.N_PC)] for y in range(0, self.N_GrC)]
        self.IO_PC_weights = [1.0] * self.N_IO
        self.IO_DCN_weights = [0.2] * self.N_IO
        self.PC_DCN_weights = [-1.0] * self.N_PC



    def generate_GrC_activity(self):
        index = 0
        for i in range(0, self.N_joints):
            for j in self.desired_position_index.keys():
                if (self.joint_list[i] == j) :
                    self.GrC_activity_index[i] = (((((((i * self.n_states_per_inputs) + self.desired_position_index[j]) * self.n_states_per_inputs) + self.current_position_index[j]) * self.n_states_per_inputs) + self.desired_velocity_index[j]) * self.n_states_per_inputs) + self.current_velocity_index[j]
                    for x in range(self.n_states_per_inputs):
                        self.GrC_activity_index[index] = (((((((i * self.n_states_per_inputs) + x) * self.n_states_per_inputs) + self.current_position_index[j]) * self.n_states_per_inputs) + self.desired_velocity_index[j]) * self.n_states_per_inputs) + self.current_velocity_index[j]
                        index+=1
                    for x in range(self.n_states_per_inputs):
                        self.GrC_activity_index[index] = (((((((i * self.n_states_per_inputs) + self.desired_position_index[j]) * self.n_states_per_inputs) + x) * self.n_states_per_inputs) + self.desired_velocity_index[j]) * self.n_states_per_inputs) + self.current_velocity_index[j]
                        index+=1
                    for x in range(self.n_states_per_inputs):
                        self.GrC_activity_index[index] = (((((((i * self.n_states_per_inputs) + self.desired_position_index[j]) * self.n_states_per_inputs) + self.current_position_index[j]) * self.n_states_per_inputs) + x) * self.n_states_per_inputs) + self.current_velocity_index[j]
                        index+=1
                    for x in range(self.n_states_per_inputs):
                        self.GrC_activity_index[index] = (((((((i * self.n_states_per_inputs) + self.desired_position_index[j]) * self.n_states_per_inputs) + self.current_position_index[j]) * self.n_states_per_inputs) + self.desired_velocity_index[j]) * self.n_states_per_inputs) + x
                        index+=1



    def generate_IO_activity(self):
        for i in range(0, self.N_joints):
            for j in self.desired_position_index.keys():
                if (self.joint_list[i] == j) :
                    self.error[j] = (self.desired_position[j] - self.current_position[j]) * self.pos_error_gain[j] + (self.desired_velocity[j] - self.current_velocity[j]) * self.vel_error_gain[j]

                    if self.error[j] >= 0:
                        #negative error is 0
                        self.IO_activity[2 * i + 1] = 0.0
                        #positive error
                        if self.error[j] > 1:
                            self.IO_activity[2 * i] = 1.0
                        else:
                            self.IO_activity[2 * i] = self.error[j]
                    else:
                        #positive error is 0
                        self.IO_activity[2 * i] = 0.0
                        #negative error
                        if self.error[j] < -1:
                            self.IO_activity[2 * i + 1] = 1.0
                        else:
                            self.IO_activity[2 * i + 1] = -self.error[j]



    def compute_PC_activity_and_PF_learning(self):
        #Reset PC activity
        for i in range(0, self.N_PC):
            self.PC_activity[i] = 0.0

        #Compute PF contribution (GrC - PC connections) and update PF weights
        for j in range(0, self.N_joints):
            for index in self.error.keys():
                if (self.joint_list[j] == index):
                    for i in range(0, self.N_PC):
                        for x in range(self.n_states_per_inputs*4):
                            self.PC_activity[i] += self.GrC_PC_weights[self.GrC_activity_index[j*self.n_states_per_inputs*4+x]][i]
                            self.GrC_PC_weights[self.GrC_activity_index[j*self.n_states_per_inputs*4+x]][i] += (self.LTP * (1 - self.IO_activity[i] ) - self.LTD * self.IO_activity[i]) * abs(self.error[index])
                            if (self.GrC_PC_weights[self.GrC_activity_index[j*self.n_states_per_inputs*4+x]][i] > self.Max_GrC_weight):
                                self.GrC_PC_weights[self.GrC_activity_index[j*self.n_states_per_inputs*4+x]][i] = self.Max_GrC_weight
                            if (self.GrC_PC_weights[self.GrC_activity_index[j*self.n_states_per_inputs*4+x]][i] < 0.0):
                                self.GrC_PC_weights[self.GrC_activity_index[j*self.n_states_per_inputs*4+x]][i] = 0.0

        #Compute CF contribution (IO - PC connections)
        #for i in range(0, self.N_PC):
        #    self.PC_activity[i] += self.IO_activity[i]*self.IO_PC_weights[i]

        #print(self.PC_activity)


    def compute_DCN_activity(self):
        #Compute MF baseline contribution
        for i in range(0, self.N_DCN):
            self.DCN_activity[i] = self.MF_DCN_weights[i]

        #Compute PC contribution (inhibitory connections)
        for i in range(0, self.N_DCN):
            self.DCN_activity[i] += self.PC_DCN_weights[i]*self.PC_activity[i]

        #Compute IO contribution
        for i in range(0, self.N_DCN):
            self.DCN_activity[i] += self.IO_DCN_weights[i]*self.IO_activity[i]

        #Limit DCN activity between 0 and 1
        for i in range(0, self.N_DCN):
            if (self.DCN_activity[i] > 1.0):
                self.DCN_activity[i] = 1.0
            if (self.DCN_activity[i] < 0.0):
                self.DCN_activity[i] = 0.0

        #print(self.DCN_activity)


    def compute_output_torque(self):
        for i in range(0, self.N_joints):
            for j in self.output_torque_gain.keys():
                if (self.joint_list[i] == j) :
                    self.torque_msg.data[i] = (self.DCN_activity[2 * i] - self.DCN_activity[2 * i + 1]) * self.output_torque_gain[j]

        self.torque_msg.delay = rospy.get_rostime().to_sec()
        self.torque_msg.header.stamp = rospy.get_rostime()
        self.output_pub.publish(self.torque_msg)



    def DesiredPositionCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.desired_position.keys():
                if (x == i):
                    self.desired_position[i] = data.data[x_index]
                    for y in self.joint_list:
                        if (y == i):
                            index = int(self.n_states_per_inputs * (data.data[x_index] - self.min_pos_amplitude[i]) / (self.max_pos_amplitude[i] - self.min_pos_amplitude[i]))
                            if (index < 0):
                                index = 0
                            if (index > (self.n_states_per_inputs - 1)):
                                index = self.n_states_per_inputs - 1
                            self.desired_position_index[i] = index
            x_index+=1

    def CurrentPositionCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.current_position.keys():
                if (x == i):
                    self.current_position[i] = data.data[x_index]
                    for y in self.joint_list:
                        if (y == i):
                            index = int(self.n_states_per_inputs * (data.data[x_index] - self.min_pos_amplitude[i]) / (self.max_pos_amplitude[i] - self.min_pos_amplitude[i]))
                            if (index < 0):
                                index = 0
                            if (index > (self.n_states_per_inputs - 1)):
                                index = self.n_states_per_inputs - 1
                            self.current_position_index[i] = index
            x_index+=1

    def DesiredVelocityCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.desired_velocity.keys():
                if (x == i):
                    self.desired_velocity[i] = data.data[x_index]
                    for y in self.joint_list:
                        if (y == i):
                            index = int(self.n_states_per_inputs * (data.data[x_index] - self.min_vel_amplitude[i]) / (self.max_vel_amplitude[i] - self.min_vel_amplitude[i]))
                            if (index < 0):
                                index = 0
                            if (index > (self.n_states_per_inputs - 1)):
                                index = self.n_states_per_inputs - 1
                            self.desired_velocity_index[i] = index
            x_index+=1

    def CurrentVelocityCallback(self, data):
        x_index = 0
        for x in data.names:
            for i in self.current_velocity.keys():
                if (x == i):
                    self.current_velocity[i] = data.data[x_index]
                    for y in self.joint_list:
                        if (y == i):
                            index = int(self.n_states_per_inputs * (data.data[x_index] - self.min_vel_amplitude[i]) / (self.max_vel_amplitude[i] - self.min_vel_amplitude[i]))
                            if (index < 0):
                                index = 0
                            if (index > (self.n_states_per_inputs - 1)):
                                index = self.n_states_per_inputs - 1
                            self.current_velocity_index[i] = index
            x_index+=1


    def update_forces(self):
        self.generate_GrC_activity()
        self.generate_IO_activity()
        self.compute_PC_activity_and_PF_learning()
        self.compute_DCN_activity()
        self.compute_output_torque()


    def clean_shutdown(self):
        print("\nExiting Analog cerebellum module...")
        self.limb.exit_control_mode()


def main():
        print("Initializing node... ")
        rospy.init_node("torque_control", anonymous=True)

    	#Retrieve RosLaunch parameters
    	limb = rospy.get_param("~limb")
        joint_list = rospy.get_param("~joint_list")
        desired_position_topic = rospy.get_param("~desired_position_topic")
        desired_velocity_topic = rospy.get_param("~desired_velocity_topic")
        current_position_topic = rospy.get_param("~current_position_topic")
        current_velocity_topic = rospy.get_param("~current_velocity_topic")
        output_topic = rospy.get_param("~output_topic")
        n_states_per_inputs = rospy.get_param("~n_states_per_inputs")
        max_pos_amplitude = rospy.get_param("~max_pos_amplitude")
        min_pos_amplitude = rospy.get_param("~min_pos_amplitude")
        max_vel_amplitude = rospy.get_param("~max_vel_amplitude")
        min_vel_amplitude = rospy.get_param("~min_vel_amplitude")
        pos_error_gain = rospy.get_param("~pos_error_gain")
        vel_error_gain = rospy.get_param("~vel_error_gain")
        output_torque_gain = rospy.get_param("~output_torque_gain")
        sampling_frequency = rospy.get_param("~sampling_frequency")
        node_name = rospy.get_param("~node_name")

        #dynamic_cfg_srv = Server(JointsPDConfig, lambda config, level: config)       #NO LO NECESITAMOS

        cerebellum = AnalogCerebellum(limb, joint_list, desired_position_topic, desired_velocity_topic,
            current_position_topic, current_velocity_topic, output_topic, n_states_per_inputs,
            max_pos_amplitude, min_pos_amplitude, max_vel_amplitude, min_vel_amplitude, pos_error_gain,
            vel_error_gain, output_torque_gain, sampling_frequency, node_name)

        # register shutdown callback
        rospy.on_shutdown(cerebellum.clean_shutdown)
        control_rate = rospy.Rate(sampling_frequency)
        while not rospy.is_shutdown():
            cerebellum.update_forces()
            control_rate.sleep()


if __name__ == "__main__":
    main()
