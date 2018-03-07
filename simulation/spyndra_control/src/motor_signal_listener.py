#!/usr/bin/env python
import rospy
import math
import sys
import time

from std_msgs.msg import Float64
from spyndra.msg import MotorSignal 

def callback(data):
    motor_signal = data
    signal = motor_signal.signal
    for i in range(8):
        if signal[i] != -1:
            ctrlrs[i].publish((signal[i] - 512) * 0.00511326497)

def motor_signal_listener():
    while not rospy.is_shutdown():
        rospy.Subscriber("motor_signal", MotorSignal, callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    # Init Spyndra joint position publishers for joint controllers.
    ctrlrs = [None] * 8
    ctrlrs[0] = rospy.Publisher('/spyndra/base_to_femur_1_position_controller/command', Float64, queue_size=10)
    ctrlrs[1] = rospy.Publisher('/spyndra/base_to_femur_2_position_controller/command', Float64, queue_size=10)
    ctrlrs[2] = rospy.Publisher('/spyndra/base_to_femur_3_position_controller/command', Float64, queue_size=10)
    ctrlrs[3] = rospy.Publisher('/spyndra/base_to_femur_4_position_controller/command', Float64, queue_size=10)
    ctrlrs[4] = rospy.Publisher('/spyndra/femur_to_tibia_1_position_controller/command', Float64, queue_size=10)
    ctrlrs[5] = rospy.Publisher('/spyndra/femur_to_tibia_2_position_controller/command', Float64, queue_size=10)
    ctrlrs[6] = rospy.Publisher('/spyndra/femur_to_tibia_3_position_controller/command', Float64, queue_size=10)
    ctrlrs[7] = rospy.Publisher('/spyndra/femur_to_tibia_4_position_controller/command', Float64, queue_size=10)

    rospy.init_node('motor_signal_listener', anonymous=True)

    try: motor_signal_listener()
    except rospy.ROSInterruptException: pass