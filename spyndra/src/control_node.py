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
    
      #run for motor angles
      elif motor_type == 3:
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

class ControlNode:
    def __init__(self):
        self.mode = 'none'
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/user_cmd", String, self.user_callback)



    def calibrateTibia(bno,femur_center):

		#start tibia at high point to lower 
		startFemur = chassis_min
		startTibia = tibia_max +150

		
		#motor4Femur = 370;

	    time.sleep(2)
	    """
	    outputMotor(femur_center[0], startTibia, 0, 1)
		outputMotor(femur_center[1], startTibia, 2, 3)	
		outputMotor(femur_center[2], startTibia, 4, 5)
		outputMotor(femur_center[3], startTibia, 6, 7)
		"""
		motor_output(startFemur, startTibia, startFemur, startTibia, startFemur, startTibia, startFemur, startTibia)
		time.sleep(1)

	    #store flat IMU data in zod_gyro and zod_accel
	    zod_matrix = flatIMUdata(bno)
	    zod_gyro = zod_matrix[0]
	    zod_accel = zod_matrix[1]
	    print("Initial flat data (gyro and accel data):")
	    print(zod_gyro) #the first vector value is perpendicular to table
	    print('\n')
	    #print(zod_accel)
	    #only using roll and pitch
	    flat_gyro =zod_gyro[1:3]

	    maxChange= .1


	    tibiaCalibrated = np.zeros(shape=(4,1))
	    nTibia = 4 #we have 4 tibias
	    tibiaCalValue = -1 #if any calibrated value is returned as -1, we've hit an error
	    previous_diff = 0
	    current_diff = 0
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
	            tibiaIncrementer = tibiaIncrementer-1
	            motor_output(startFemur, startTibia, startFemur, startTibia, startFemur, startTibia, startFemur, startTibia)
	            outputMotor(femur_center[iTibia],tibiaIncrementer, iTibia*2, iTibia*2+1)
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
	                    tibiaIncrementer = tibia_min
	                            #print("gate Delta")
	                                #femurIncrementer = chassis_max#escape the while loop
	                                #print(np.linalg.norm(delta))
	                                #print(delta)
						#print(flat_gyro)
						#print(currentVector3)
	                                        #print("calibrated!")
							

	            tibiaCalibrated[iTibia] = tibiaCalValue
	            outputMotor(femur_center[0], startTibia, 0, 1)
	        	outputMotor(femur_center[1], startTibia, 2, 3)	
	        	outputMotor(femur_center[2], startTibia, 4, 5)
	        	outputMotor(femur_center[3], startTibia, 6, 7)
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

	                
	def calibrateFemur(bno):
		startFemur = chassis_min
		startTibia = tibia_max + 175
		#motor4Femur = 370;

	    time.sleep(2)

	    outputMotor(startFemur, startTibia, 0, 1)
		outputMotor(startFemur, startTibia, 2, 3)	
		outputMotor(startFemur, startTibia, 4, 5)
		outputMotor(startFemur, startTibia, 6, 7)
		time.sleep(1)

	    zod_matrix = flatIMUdata(bno)
	    zod_gyro = zod_matrix[0]
	    zod_accel = zod_matrix[1]
		print("Initial flat data (gyro and accel data):")
	    print(zod_gyro) #the first vector value is perpendicular to table
	    print('\n')
	    #print(zod_accel)
	    #only using roll and pitch
	    flat_gyro =zod_gyro[1:3]

	    maxChange= .1



	    femurCalibrated = np.zeros(shape=(4,1))
	    nFemur = 4 #we have 4 tibias
	    femurCalValue = -1 #if any calibrated value is returned as -1, we've hit an error
	    previous_diff = 0
	    current_diff = 0
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
	            outputMotor(femurIncrementer, startTibia, iFemur*2, iFemur*2+1)
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
	                    femurIncrementer = chassis_max
	                    #print("gate Delta")
	                    femurIncrementer = chassis_max#escape the while loop
	                    #print(np.linalg.norm(delta))
	                    #print(delta)
	            #print(flat_gyro)
	                #print(currentVector3)
	                                    #print("calibrated!")
	                    

	        femurCalibrated[iFemur] = femurCalValue
	        outputMotor(startFemur, startTibia, 0, 1)
	        outputMotor(startFemur, startTibia, 2, 3)	
	        outputMotor(startFemur, startTibia, 4, 5)
	        outputMotor(startFemur, startTibia, 6, 7)
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
            if self.mode == 'cmd_1':
              chassis, tibia = gaitModule.standingGait()
            elif self.mode == "cmd_2":
              chassis, tibia = gaitModule.randomGait()

              # spline_run prep and running
              # the following code are for the older motor
              phase = 0
              motor_minmax_values = 250, 300
              motor_type = 1
              outputs = zip(*spline_run(chassis, tibia, phase, motor_type, motor_minmax_values))

              # set publisher and publish computed motor signals
              pub = rospy.Publisher("motor_signal", MotorSignal, queue_size=10)
              # from standingGait() or randomGait(), there're totally 400 outputs per gaitModule function call
              for i in range(len(outputs)):
                  motor_signal = MotorSignal()
                  motor_signal.motor_type = motor_type
                  motor_signal.chassis_1, motor_signal.chassis_2, motor_signal.chassis_3, motor_signal.chassis_4, \
                  motor_signal.tibia_1,   motor_signal.tibia_2,   motor_signal.tibia_3,   motor_signal.tibia_4 \
                                     = outputs[i]
                  pub.publish(motor_signal)
                  rospy.loginfo(i)

    def user_callback(self, user_input):
        # when user types command to the user.py node, user_callback is run
        self.mode = user_input.data
        s = 'mode set to be:'+ str(user_input)
        rospy.loginfo(s)

        # test command for ax-12
        if self.mode == 'cmd_4':
            phase = 0
            motor_minmax_values = 250, 300
            motor_type = 1
            output = (0,512,512,512,512,512,512,512)
              # set publisher and publish computed motor signals
            pub = rospy.Publisher("motor_signal", MotorSignal, queue_size=10)
            motor_signal = MotorSignal()
            motor_signal.motor_type = motor_type
            motor_signal.chassis_1, motor_signal.chassis_2, motor_signal.chassis_3, motor_signal.chassis_4, \
            motor_signal.tibia_1,   motor_signal.tibia_2,   motor_signal.tibia_3,   motor_signal.tibia_4 \
                             = output
            pub.publish(motor_signal)

    def motor_output(chassis1, tibia1, chassis2, tibia2, chassis3, tibia3, chassis4, tibia4): 
        #outputs single motor command
        pub = rospy.Publisher("motor_signal", MotorSignal, queue_size=10)
        motor_signal = MotorSignal()
        motor_signal.motor_type = motor_type
        motor_signal.chassis_1, motor_signal.chassis_2, motor_signal.chassis_3, motor_signal.chassis_4, \
        motor_signal.tibia_1,   motor_signal.tibia_2,   motor_signal.tibia_3,   motor_signal.tibia_4 \
                         = chassis1, tibia1, chassis2, tibia2, chassis3, tibia3, chassis4, tibia4
        pub.publish(motor_signal)


def main():
    rospy.init_node("control_node")
    n = ControlNode()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
