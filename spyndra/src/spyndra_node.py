#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from spyndra.msg import MotorSignal
from spyndra import ax12
# from spyndra import adafruitModule
import ast

CHASSIS_1_ID = 1
CHASSIS_2_ID = 2
CHASSIS_3_ID = 3
CHASSIS_4_ID = 4

TIBIA_1_ID = 5
TIBIA_2_ID = 6
TIBIA_3_ID = 7
TIBIA_4_ID = 8

AX_BROADCAST_ID = 254

def callback(msg):
    speed = msg.speed
    motor_id = msg.motor_id
    output = msg.signal
    s = 'move motor %d to pos %d at speed %d' %(motor_id, output, speed)

    rospy.loginfo(s)
    '''connect to 8 motors in a serial'''

    # motors = ax12.Ax12()

    '''outoput signal to id and goal position, assume chassis 1-4 are assigned '''

    # motors.moveSpeed(motor_id, output, speed)
    
    '''get actual position of the motor, and then publish it back to control_node'''
    '''right now i simply set them as dummy'''
    actual_position = 512
    actual_speed = 200
    actual_id = 0

    pub = rospy.Publisher('/actual_signal', MotorSignal, queue_size=10)
    motor_state = MotorSignal()
    motor_state.motor_id = actual_id
    motor_state.speed = acutal_speed
    motor_state.signal = actual_position

    pub.publish(motor_state)

    

def main():
    rospy.init_node('spyndra', anonymous=True)
    rospy.Subscriber('/motor_signal', MotorSignal, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
