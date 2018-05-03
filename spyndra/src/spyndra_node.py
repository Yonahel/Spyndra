#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from spyndra.msg import MotorSignal
from spyndra import ax12
import ast

CHASSIS_1_ID = 1
CHASSIS_2_ID = 3
CHASSIS_3_ID = 5
CHASSIS_4_ID = 7

TIBIA_1_ID = 2
TIBIA_2_ID = 4
TIBIA_3_ID = 6
TIBIA_4_ID = 8

AX_BROADCAST_ID = 254

CHASSIS_1_MIN = 195
CHASSIS_1_MAX = 812
CHASSIS_2_MIN = 198
CHASSIS_2_MAX = 815
CHASSIS_3_MIN = 206
CHASSIS_3_MAX = 814
CHASSIS_4_MIN = 204
CHASSIS_4_MAX = 817

TIBIA_1_MIN = 262
TIBIA_1_MAX = 875
TIBIA_2_MIN = 201
TIBIA_2_MAX = 811
TIBIA_3_MIN = 203
TIBIA_3_MAX = 819
TIBIA_4_MIN = 203
TIBIA_4_MAX = 816

PORT = '/dev/ttyACM0'

def callback(msg):
    speed = msg.speed
    motor_id = msg.motor_id
    output = msg.signal
    s = 'move motor %d to pos %d at speed %d' %(motor_id, output, speed)

    rospy.loginfo(s)
    '''connect to 8 motors in a serial'''

    motors = ax12.Ax12(port=PORT)

    '''checking the output signal to prevent overloading'''
    if output >= 195 and output <= 875:
        '''outoput signal to id and goal position, assume chassis 1-4 are assigned '''
        error_code = motors.moveSpeed(motor_id, output, speed)
    else:
        error_code = -1


    

def main():
    rospy.init_node('spyndra', anonymous=True)
    rospy.Subscriber('/motor_signal', MotorSignal, callback)
    pub = rospy.Publisher('/actual_signal', MotorSignal, queue_size=10)
    motors = ax12.Ax12(port=PORT)
    while not rospy.is_shutdown():
        #keep publish current position
        for i in range(8):
            '''get actual position of the motor, and then publish it back to control_node'''
            '''right now i simply set them as dummy'''
            motor_id = i+1
            motor_state = MotorSignal()
            motor_state.motor_id = i+1
            motor_state.speed = ax12.readPresentSpeed(motor_id)
            motor_state.signal = ax12.readPosition(motor_id)
            motor_state.load = ax12.readLoad(motor_id)
            pub.publish(motor_state)
            rospy.loginfo(motor_state)

if __name__ == '__main__':
    main()
