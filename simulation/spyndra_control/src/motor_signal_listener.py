#!/usr/bin/env python
import rospy
import math
import sys
import time

from std_msgs.msg import Float64
from spyndra.msg import MotorSignal


#Spyndra joint position publishers for joint controllers.
b2f_1, b2f_2, b2f_3, b2f_4, f2t_1, f2t_2, f2t_3, f2t_4 = None, None, None, None, None, None, None, None

def init_joint_position_publishers():
    #Initiate node for controlling joint positions.
    #rospy.init_node('joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
    b2f_1 = rospy.Publisher('/spyndra/base_to_femur_1_position_controller/command', Float64, queue_size=10)
    b2f_2 = rospy.Publisher('/spyndra/base_to_femur_2_position_controller/command', Float64, queue_size=10)
    b2f_3 = rospy.Publisher('/spyndra/base_to_femur_3_position_controller/command', Float64, queue_size=10)
    b2f_4 = rospy.Publisher('/spyndra/base_to_femur_4_position_controller/command', Float64, queue_size=10)
    f2t_1 = rospy.Publisher('/spyndra/femur_to_tibia_1_position_controller/command', Float64, queue_size=10)
    f2t_2 = rospy.Publisher('/spyndra/femur_to_tibia_2_position_controller/command', Float64, queue_size=10)
    f2t_3 = rospy.Publisher('/spyndra/femur_to_tibia_3_position_controller/command', Float64, queue_size=10)
    f2t_4 = rospy.Publisher('/spyndra/femur_to_tibia_4_position_controller/command', Float64, queue_size=10)

def callback(data):
    motor_signal = data.data
    signal = motor_signal.signal

    b2f_1.publish(signal[0])
    b2f_2.publish(signal[1])
    b2f_3.publish(signal[2])
    b2f_4.publish(signal[3])
    f2t_1.publish(signal[4])
    f2t_2.publish(signal[5])
    f2t_3.publish(signal[6])
    f2t_4.publish(signal[7])

def motor_signal_listener():
    rospy.init_node('motor_signal_listener', anonymous=True)

    rospy.Subscriber("motor_signal", MotorSignal, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    init_joint_position_publishers()

    try: motor_signal_listener()
    except rospy.ROSInterruptException: pass