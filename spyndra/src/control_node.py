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



class ControlNode:
    def __init__(self):
        self.mode = 'none'
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/user_cmd", String, self.user_callback)

    def pubish_signal(self, output, speed=300):
        # speed range: 0-1023
        # output: a tuple of 8 values for each motor. range: 0-1023
        # example: output = (1023,512,512,512,512,512,512,512)

        pub = rospy.Publisher("motor_signal", MotorSignal, queue_size=10)
        motor_signal = MotorSignal()
        motor_signal.speed = speed
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
            speed = 200
            output = (1023,512,512,512,512,512,512,512)
              # set publisher and publish computed motor signals
            self.pubish_signal(output, speed)
            rospy.loginfo(output)

def main():
    rospy.init_node("control_node")
    n = ControlNode()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
