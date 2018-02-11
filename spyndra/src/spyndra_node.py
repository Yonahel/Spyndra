#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from spyndra.msg import MotorSignal
import ax12
# from spyndra import adafruitModule
import ast

# spyndraMotor = motorModule.SpyndraMotor()

def callback(msg):
    motor_type = msg.motor_type
    chassis_1, chassis_2, chassis_3, chassis_4 = msg.chassis_1, msg.chassis_2, msg.chassis_3, msg.chassis_4
    tibia_1,   tibia_2,   tibia_3,   tibia_4   = msg.tibia_1, msg.tibia_2, msg.tibia_3, msg.tibia_4
    
    rospy.loginfo(chassis_1)
    # connect to 8 motors in a serial
    motors = ax12.Ax12()

    #outoput signal to id and goal position, need translation
    motors.moveSpeed(1, int(chassis_1), 100)
    

def main():
    rospy.init_node('spyndra', anonymous=True)
    rospy.Subscriber('/motor_signal', MotorSignal, callback)
    rospy.spin()

if __name__ == '__main__':
    main()