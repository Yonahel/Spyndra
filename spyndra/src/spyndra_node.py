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

def callback(msg):
    speed = msg.speed
    motor_id = msg.motor_id
    output = msg.signal
    s = 'move motor %d to pos %d at speed %d' %(motor_id, output, speed)

    rospy.loginfo(s)
    '''connect to 8 motors in a serial'''

    motors = ax12.Ax12()

    '''checking the output signal to prevent overloading'''
    if output >= 195 and output =< 875:
        '''outoput signal to id and goal position, assume chassis 1-4 are assigned '''
        error_code = motors.moveSpeed(motor_id, output, speed)
        
        '''get actual position of the motor, and then publish it back to control_node'''
        '''right now i simply set them as dummy'''
        actual_position = ax12.readPosition(motor_id)
        actual_speed = ax12.readPresentSpeed(motor_id)
        actual_load = ax12.readLoad(motor_id)
        actual_id = motor_id
        action_error = error_code

        pub = rospy.Publisher('/actual_signal', MotorSignal, queue_size=10)
        motor_state = MotorSignal()
        motor_state.motor_id = actual_id
        motor_state.speed = acutal_speed
        motor_state.signal = actual_position
        motor_state.load = actual_load
        motor_state.action_error = action_error

    else:
        pub = rospy.Publisher('/actual_signal', MotorSignal, queue_size=10)
        motor_state.action_error = -1 # exceeding the minmax, no movements
    pub.publish(motor_state)

    

def main():
    rospy.init_node('spyndra', anonymous=True)
    rospy.Subscriber('/motor_signal', MotorSignal, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
