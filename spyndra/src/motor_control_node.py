#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$


import rospy
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from spyndra import gaitModule
from spyndra.msg import MotorSignal

# from BNO055 import *
# import Adafruit_PCA9685
from datetime import datetime
import json 
import numpy as np
import time
import csv
import getch
from subprocess import Popen, PIPE


def motor_getsignal(motor_type, chassisOutput1, tibiaOutput1, chassisOutput2, tibiaOutput2,
                chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4):
    
   # get motor signal data in dictionary in the format:

    '''motor_signal_data = {
    'motor_type': 1 or 2
    'motor_output_1': {
        chassisOutput, 
        tibiaOutput, 
        chassisNum, 
        tibiaNum
    }
    'motor_output_2': {
        chassisOutput, 
        tibiaOutput, 
        chassisNum, 
        tibiaNum
    }
    'motor_output_3': {
        chassisOutput, 
        tibiaOutput, 
        chassisNum, 
        tibiaNum
    }
    'motor_output_4': {
        chassisOutput, 
        tibiaOutput, 
        chassisNum, 
        tibiaNum
    }
    }'''    

    motor_signal_data  = {}
    motor_signal_data['motor_type'] = motor_type

    # 4 sequence of the singals (equivalent to output_motor(chassisOutput1, tibiaOutput1, 0, 1)):
    motor_signal_data['motor_output_1'] = {}
    # 4 arguments for the function outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum)
    motor_signal_data['motor_output_1']['chassisOutput'] = chassisOutput1
    motor_signal_data['motor_output_1']['tibiaOutput'] = tibiaOutput1
    motor_signal_data['motor_output_1']['chassisNum'] = 0
    motor_signal_data['motor_output_1']['tibiaNum'] = 1

    motor_signal_data['motor_output_2'] = {}
    # 4 arguments for the function outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum)
    motor_signal_data['motor_output_2']['chassisOutput'] = chassisOutput2
    motor_signal_data['motor_output_2']['tibiaOutput'] = tibiaOutput2
    motor_signal_data['motor_output_2']['chassisNum'] = 2
    motor_signal_data['motor_output_2']['tibiaNum'] = 3

    motor_signal_data['motor_output_3'] = {}
    # 4 arguments for the function outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum)
    motor_signal_data['motor_output_3']['chassisOutput'] = chassisOutput3
    motor_signal_data['motor_output_3']['tibiaOutput'] = tibiaOutput3
    motor_signal_data['motor_output_3']['chassisNum'] = 4
    motor_signal_data['motor_output_3']['tibiaNum'] = 5

    motor_signal_data['motor_output_4'] = {}
    # 4 arguments for the function outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum)
    motor_signal_data['motor_output_4']['chassisOutput'] = chassisOutput4
    motor_signal_data['motor_output_4']['tibiaOutput'] = tibiaOutput4
    motor_signal_data['motor_output_4']['chassisNum'] = 6
    motor_signal_data['motor_output_4']['tibiaNum'] = 7
    return motor_signal_data

def set_leg_counter(phase, chassis):
    # init leg counter
    leg1_counter = ((4.0*phase)/360.0)*len(chassis)
    leg2_counter = ((3.0*phase)/360.0)*len(chassis)
    leg3_counter = ((2.0*phase)/360.0)*len(chassis)
    leg4_counter = ((1.0*phase)/360.0)*len(chassis)

    #In the case of a phase greater than 180, leg1 and leg2 must be corrected back to 0 and 180 degrees
    if(phase >= 180):
        leg2_counter = ((1.0*phase)/360.0)*len(chassis)
        leg1_counter = ((2.0*phase)/360.0)*len(chassis)
    return leg1_counter, leg2_counter, leg3_counter, leg4_counter   


def spline_run(chassis, tibia, phase, motor_type, motor_minmax_values):
    
    leg1_counter, leg2_counter, leg3_counter, leg4_counter\
    = set_leg_counter(phase, chassis)

    # set motor minmax value
    motor0_min = motor_minmax_values[0]
    motor0_max = motor_minmax_values[1]
    motor1_min = motor_minmax_values[0]
    motor1_max = motor_minmax_values[1]
    motor2_min = motor_minmax_values[0]
    motor2_max = motor_minmax_values[1]
    motor3_min = motor_minmax_values[0]
    motor3_max = motor_minmax_values[1]
    motor4_min = motor_minmax_values[0]
    motor4_max = motor_minmax_values[1]
    motor5_min = motor_minmax_values[0]
    motor5_max = motor_minmax_values[1]
    motor6_min = motor_minmax_values[0]
    motor6_max = motor_minmax_values[1]
    motor7_min = motor_minmax_values[0]
    motor7_max = motor_minmax_values[1]

    chassisOutput1, chassisOutput2, chassisOutput3, chassisOutput4 = [], [], [], []
    tibiaOutput1, tibiaOutput2, tibiaOutput3, tibiaOutput4 = [], [], [], []
    # for i in range(3):
    for i in range(len(chassis)):
        if leg1_counter >= len(chassis):
            leg1_counter -= len(chassis)
        if leg2_counter >= len(chassis):
            leg2_counter -= len(chassis)
        if leg3_counter >= len(chassis):
            leg3_counter -= len(chassis)
        if leg4_counter >= len(chassis):
            leg4_counter -= len(chassis)

        
        #run for percentages
        if motor_type == 1 or motor_type == 2:
        
            chassisOutput1 += [chassis[leg1_counter]*(motor0_max-motor0_min) + motor0_min]
            tibiaOutput1 += [tibia[leg1_counter]*(motor1_max-motor1_min)+motor1_min]
            
            chassisOutput2 += [chassis[leg2_counter]*(motor2_max-motor2_min) + motor2_min]
            tibiaOutput2 += [tibia[leg2_counter]*(motor3_max-motor3_min)+motor3_min]

            chassisOutput3 += [chassis[leg3_counter]*(motor4_max-motor4_min) + motor4_min]
            tibiaOutput3 += [tibia[leg3_counter]*(motor5_max-motor5_min)+motor5_min]

            chassisOutput4 += [chassis[leg4_counter]*(motor6_max-motor6_min) + motor6_min]
            tibiaOutput4 += [tibia[leg4_counter]*(motor7_max-motor7_min)+motor7_min]


            #singal1 = motor_getsignal(1, chassisOutput1, tibiaOutput1, chassisOutput2, \
             #   tibiaOutput2, chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4)
    
        #run for motor angles
        elif self.motor.motor_type == 3:
            chassisOutput1 += [chassis[leg1_counter]]
            tibiaOutput1 += [tibia[leg1_counter]]
            
            chassisOutput2 += [chassis[leg2_counter]]
            tibiaOutput2 += [tibia[leg2_counter]]

            chassisOutput3 += [chassis[leg3_counter]]
            tibiaOutput3 += [tibia[leg3_counter]]

            chassisOutput4 += [chassis[leg4_counter]]
            tibiaOutput4 += [tibia[leg4_counter]]
         
        leg1_counter+=1
        leg2_counter+=1
        leg3_counter+=1
        leg4_counter+=1
    
    return chassisOutput1, chassisOutput2, chassisOutput3, chassisOutput4, tibiaOutput1, tibiaOutput2, tibiaOutput3, tibiaOutput4    

def callback(msg):
    #gravity = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z 
    #euler   = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
    
    if msg.data == "cmd_1":
        chassis, tibia = gaitModule.standingGait()
    elif msg.data == "cmd_2":
        chassis, tibia = gaitModule.randomGait()
    
    phase = 0
    motor_minmax_values = 250, 300
    motor_type = 1
    outputs = zip(*spline_run(chassis, tibia, phase, motor_type, motor_minmax_values))

    pub = rospy.Publisher("motor_signal", MotorSignal, queue_size=10)
    for i in range(len(outputs)):
        motor_signal = MotorSignal()
        motor_signal.motor_type = motor_type
        motor_signal.chassis_1, motor_signal.chassis_2, motor_signal.chassis_3, motor_signal.chassis_4, \
        motor_signal.tibia_1,   motor_signal.tibia_2,   motor_signal.tibia_3,   motor_signal.tibia_4 \
                             = outputs[i]
        pub.publish(motor_signal)
        rospy.loginfo(i)

def main():
    rospy.init_node("motor_control_node")
    #rospy.Subscriber("/imu/data", Imu, callback)
    rospy.Subscriber("/user_cmd", String, callback)
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
