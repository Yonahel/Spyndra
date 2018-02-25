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
from spyndra import vectorModule
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



class ControlNode:
    CHASSIS_1_ID = 1
    CHASSIS_2_ID = 2
    CHASSIS_3_ID = 3
    CHASSIS_4_ID = 4

    TIBIA_1_ID = 5
    TIBIA_2_ID = 6
    TIBIA_3_ID = 7
    TIBIA_4_ID = 8

    def __init__(self):
        self.mode = 'none'
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/user_cmd", String, self.user_callback)
       

    def calibrateTibia(self, bno,femur_center):
        #current observed min and max of legs
        tibia_min = 260 #325 according to json file 
        tibia_max = 450 #425
        chassis_min = 225 #275
        chassis_max = 425 #475

        #start tibia at high point to lower 
        startFemur = chassis_min
        startTibia = tibia_max +150
        

        time.sleep(2)
        self.pubish_signal(CHASSIS_1_ID, femur_center[0])
        self.pubish_signal(TIBIA_1_ID, startTibia)
        self.pubish_signal(CHASSIS_2_ID, femur_center[1])
        self.pubish_signal(TIBIA_2_ID, startTibia)
        self.pubish_signal(CHASSIS_3_ID, femur_center[2])
        self.pubish_signal(TIBIA_3_ID, startTibia)
        self.pubish_signal(CHASSIS_4_ID, femur_center[3])
        self.pubish_signal(TIBIA_4_ID, startTibia)
        time.sleep(1)

        #store flat IMU data in zod_gyro and zod_accel
        zod_matrix = flatIMUdata(bno)
        zod_gyro = zod_matrix[0]
        print("Initial flat data (gyro and accel data):")
        print(zod_gyro) #the first vector value is perpendicular to table
        print('\n')
        flat_gyro =zod_gyro[1:3] #only using roll and pitch


        tibiaCalibrated = np.zeros(shape=(4,1))
        nTibia = 4 #we have 4 tibias
        tibiaCalValue = -1 #if any calibrated value is returned as -1, we've hit an error
        delta_threshold = .05
        diff = 0
        delta =0
        change_threshold = 6

        #go through each tibia
        for iTibia in range(0,nTibia):
            previous_diff = 0
            current_diff = 0
            change_counter = 0
            
            tibiaIncrementer = startTibia
            while tibiaIncrementer > tibia_min:
                #slowly move the tibia down
                tib = "TIBIA_"
                iden = "_ID"
                tibia = tib + str(iTibia + 1) + iden
                tibiaIncrementer = tibiaIncrementer-1
                self.pubish_signal(tibia, tibiaIncrementer)
                next_pwm =0
                
                #track whether or not you've hit the ground through
                #change in flat data
                while (change_counter < change_threshold) and (not next_pwm):
                    time.sleep(0.01)
                    currentV = produceVector(bno)
                    print(currentV[0][1:3])

                    previous_diff = current_diff
                    #only using rolls and pitch
                    current_diff = flat_gyro - currentV[0][1:3]

                    if tibiaIncrementer == startTibia-1:
                        previous_diff = current_diff
                        print("we should only see this once per cycle")

            
                    delta = abs(current_diff-previous_diff)
                    print delta
                    #print ("\n")
                    #print(str(np.linalg.norm(delta)))
                    if (np.linalg.norm(delta) > delta_threshold):
                        change_counter = change_counter+1
                        #time.sleep(0.1)
                        print(str(change_counter) + " for the boys back home")
                    else:
                        change_counter =0
                        next_pwm =1
                    if change_counter == change_threshold:
                        next_pwm =1
                        change_counter =0

                        tibiaCalValue = tibiaIncrementer
                        tibiaIncrementer = tibia_min #escape the while loop
                        #print("gate Delta")
                        #print(np.linalg.norm(delta))
                        #print(delta)
                        #print(flat_gyro)
                        #print(currentVector3)
                        #print("calibrated!")
                            

                tibiaCalibrated[iTibia] = tibiaCalValue
                self.pubish_signal(CHASSIS_1_ID, femur_center[0])
                self.pubish_signal(TIBIA_1_ID, startTibia)
                self.pubish_signal(CHASSIS_2_ID, femur_center[1])
                self.pubish_signal(TIBIA_2_ID, startTibia)
                self.pubish_signal(CHASSIS_3_ID, femur_center[2])
                self.pubish_signal(TIBIA_3_ID, startTibia)
                self.pubish_signal(CHASSIS_4_ID, femur_center[3])
                self.pubish_signal(TIBIA_4_ID, startTibia)
                time.sleep(2)

                zod_matrix = flatIMUdata(bno)
                zod_gyro = zod_matrix[0]
                zod_accel = zod_matrix[1]
                print("next initial set of flat data")
                print(zod_gyro) #the first vector value is perpendicular to table
                print('\n')
                #print(zod_accel)
                #again, only using pitch and roll
                flat_gyro =zod_gyro[1:3]
                time.sleep(1)
                    
            print(tibiaCalibrated)
            max_list = []
            min_list = []
            
            for j in range(0,4):
                currentValue = tibiaCalibrated[j]
                currentMax = currentValue-40 #44 #based off diagram from paper
                currentMin = currentValue-155 #159 #based off diagram from paper
                min_list.append(currentMin)
                max_list.append(currentMax)
                

        return max_list,min_list

                    
    def calibrateFemur(self, bno):
        #current observed min and max of legs
        tibia_min = 260 #325 according to json file 
        tibia_max = 450 #425
        chassis_min = 225 #275
        chassis_max = 425 #475

        startFemur = chassis_min
        startTibia = tibia_max + 175
    

        time.sleep(2)
        self.pubish_signal(CHASSIS_1_ID, startFemur)
        self.pubish_signal(TIBIA_1_ID, startTibia)
        self.pubish_signal(CHASSIS_2_ID, startFemur)
        self.pubish_signal(TIBIA_2_ID, startTibia)
        self.pubish_signal(CHASSIS_3_ID, startFemur)
        self.pubish_signal(TIBIA_3_ID, startTibia)
        self.pubish_signal(CHASSIS_4_ID, startFemur)
        self.pubish_signal(TIBIA_4_ID, startTibia)
        time.sleep(1)

        zod_matrix = flatIMUdata(bno)
        zod_gyro = zod_matrix[0]
        print("Initial flat data (gyro data):")
        print(zod_gyro) #the first vector value is perpendicular to table
        print('\n')
        flat_gyro =zod_gyro[1:3] #only using roll and pitch


        femurCalibrated = np.zeros(shape=(4,1))
        nFemur = 4 #we have 4 tibias
        femurCalValue = -1 #if any calibrated value is returned as -1, we've hit an error
        delta_threshold = .05
        diff = 0
        delta =0
        change_threshold = 6

        for iFemur in range(0,nFemur):
            previous_diff = 0
            current_diff = 0
            change_counter = 0
            
            femurIncrementer = startFemur
            while femurIncrementer < chassis_max:
                femurIncrementer = 1 + femurIncrementer
                fem = "CHASSIS_"
                iden = "_ID"
                femur = tib + str(iFemur + 1) + iden
                self.pubish_signal(femur, tibiaIncrementer)
                next_pwm =0

                while (change_counter < change_threshold) and (not next_pwm):
                    time.sleep(0.01)
                    currentV = produceVector(bno)
                    print(currentV[0][1:3])

                    previous_diff = current_diff
                    #only using rolls and pitch
                    current_diff = flat_gyro - currentV[0][1:3]

                    if femurIncrementer == startFemur+1:
                        previous_diff = current_diff
                        print("we should only see this once per cycle")  
                        delta = abs(current_diff-previous_diff)
                        print delta
                        #print ("\n")
                        #print(str(np.linalg.norm(delta)))
                    if (np.linalg.norm(delta) > delta_threshold):
                        change_counter = change_counter+1
                        #time.sleep(0.1)
                        print(str(change_counter) + " for the boys back home")
                    else:
                        change_counter =0
                        next_pwm =1
                    if change_counter == change_threshold:
                        next_pwm =1
                        change_counter =0

                        femurCalValue = femurIncrementer
                        femurIncrementer = chassis_max #escape while loop
                        #print("gate Delta")
                        #print(np.linalg.norm(delta))
                        #print(delta)
                        #print(flat_gyro)
                        #print(currentVector3)
                        #print("calibrated!")
                        

            femurCalibrated[iFemur] = femurCalValue
            self.pubish_signal(CHASSIS_1_ID, startFemur)
            self.pubish_signal(TIBIA_1_ID, startTibia)
            self.pubish_signal(CHASSIS_2_ID, startFemur)
            self.pubish_signal(TIBIA_2_ID, startTibia)
            self.pubish_signal(CHASSIS_3_ID, startFemur)
            self.pubish_signal(TIBIA_3_ID, startTibia)
            self.pubish_signal(CHASSIS_4_ID, startFemur)
            self.pubish_signal(TIBIA_4_ID, startTibia)
            time.sleep(2)

            zod_matrix = flatIMUdata(bno)
            zod_gyro = zod_matrix[0]
            zod_accel = zod_matrix[1]
            print("next initial set of flat data")
            print(zod_gyro) #the first vector value is perpendicular to table
            print('\n')
            #print(zod_accel)
            #again, only using pitch and roll
            flat_gyro =zod_gyro[1:3]
            time.sleep(1)
                
        print(femurCalibrated)

        femurCenter = []
        femurMax = []
        femurMin = []
        for j in range(0,4):
            currentValue = femurCalibrated[j]
            currentCenter = currentValue-40 #48 #observationally based
            currentMax = currentCenter+80 #70 #based off diagram from paper
            currentMin = currentCenter-30 #based off diagram from paper

            femurCenter.append(currentCenter)
            femurMax.append(currentMax)
            femurMin.append(currentMin)

        return femurCenter,femurMax,femurMin

    def pubish_signal(self, motor_id, output, speed=300):
        # motor_id: 1-4 corresponds to chassis 1-4, 5-8 corresponds to tibia 1-4
        # speed range: 0-1023
        # output: a tuple of 8 values for each motor. range: 0-1023

        pub = rospy.Publisher("motor_signal", MotorSignal, queue_size=10)
        motor_signal = MotorSignal()
        motor_signal.speed = speed
        motor_signal.motor_id = motor_id
        motor_signal.signal = output
        pub.publish(motor_signal)        


    def imu_callback(self, imu):
        # when bno055.py node is spinning, imu data is packed in imu variable
        s = 'current mode: ' + str(self.mode)
        rospy.loginfo(s)

        # control node may neet to subscribe IMU data and feed them into RL models for action genetaion
        
        # gravity = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z 
        # euler   = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z        

        # take action based on subscribed msg "/user_cmd"
        # "cmd_1", "cmd_2" can be further customized, these are now under development
        chassis, tibia = [], []
        if self.mode == 'cmd_0':
            # calibration section
            pass
        else: 
            pass


    def user_callback(self, user_input):
        # when user types command to the user.py node, user_callback is run
        self.mode = user_input.data
        s = 'mode set to be:'+ str(user_input)
        rospy.loginfo(s)

        # test command for ax-12
        if self.mode == 'cmd_4':
            # example of move chassis_1 to position 512 at speed 200
            speed = 200
            output = 512
              # set publisher and publish computed motor signals
            self.pubish_signal(ControlNode.CHASSIS_1_ID, output, speed)
            rospy.loginfo(output)

    """
    def motor_output(chassis1, tibia1, chassis2, tibia2, chassis3, tibia3, chassis4, tibia4): 
        #outputs single motor command
        pub = rospy.Publisher("motor_signal", MotorSignal, queue_size=10)
        motor_signal = MotorSignal()
        motor_signal.motor_type = motor_type
        motor_signal.chassis_1, motor_signal.chassis_2, motor_signal.chassis_3, motor_signal.chassis_4, \
        motor_signal.tibia_1,   motor_signal.tibia_2,   motor_signal.tibia_3,   motor_signal.tibia_4 \
                         = chassis1, tibia1, chassis2, tibia2, chassis3, tibia3, chassis4, tibia4
        pub.publish(motor_signal)
    """

def main():
    rospy.init_node("control_node")
    n = ControlNode()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
