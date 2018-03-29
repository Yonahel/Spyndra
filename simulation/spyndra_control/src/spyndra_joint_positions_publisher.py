#!/usr/bin/env python
import rospy
import math
import sys
import time

from std_msgs.msg import Float64
from spyndra.msg import MotorSignal 

#For testing gaits

#Spyndra joint position publishers for joint controllers.
b2f_1, b2f_2, b2f_3, b2f_4, f2t_1, f2t_2, f2t_3, f2t_4 = None, None, None, None, None, None, None, None

def spyndra_joint_positions_publisher():
    #Initiate node for controlling joint positions.
    rospy.init_node('joint_positions_node', anonymous=True)

	#Define publishers for each joint position controller commands.
    b2f_1 = rospy.Publisher('/spyndra/base_to_femur_1_position_controller/command', Float64, queue_size=10)
    b2f_2 = rospy.Publisher('/spyndra/base_to_femur_2_position_controller/command', Float64, queue_size=10)
    b2f_3 = rospy.Publisher('/spyndra/base_to_femur_3_position_controller/command', Float64, queue_size=10)
    b2f_4 = rospy.Publisher('/spyndra/base_to_femur_4_position_controller/command', Float64, queue_size=10)
    f2t_1 = rospy.Publisher('/spyndra/femur_to_tibia_1_position_controller/command', Float64, queue_size=10)
    f2t_2 = rospy.Publisher('/spyndra/femur_to_tibia_2_position_controller/command', Float64, queue_size=10)
    f2t_3 = rospy.Publisher('/spyndra/femur_to_tibia_3_position_controller/command', Float64, queue_size=10)
    f2t_4 = rospy.Publisher('/spyndra/femur_to_tibia_4_position_controller/command', Float64, queue_size=10)

    action_publisher = rospy.Publisher('motor_signal', MotorSignal, queue_size=5)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        line = sys.stdin.readline()
        pos = [float(p) for p in line.split()]

        motor_signal = MotorSignal()
        motor_signal.motor_type = 1
        motor_signal.signal = pos[:8]
        action_publisher.publish(motor_signal)

        # b2f_1.publish(pos[0])
        # b2f_2.publish(pos[1])
        # b2f_3.publish(pos[2])
        # b2f_4.publish(pos[3])
        # f2t_1.publish(pos[4])
        # f2t_2.publish(pos[5])
        # f2t_3.publish(pos[6])
        # f2t_4.publish(pos[7])
        
        rate.sleep()

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    try: spyndra_joint_positions_publisher()
    except rospy.ROSInterruptException: pass