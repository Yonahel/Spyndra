#!/usr/bin/env python
import rospy
import math
import sys
import time

from sensor_msgs.msg import JointState
from spyndra.msg import MotorSignal

def callback(data):
    motor_signal = MotorSignal()
    motor_signal.motor_type = 1
    motor_signal.signal = [(p * 195.569759) + 512 for p in data.position]
    motor_state_publisher.publish(motor_signal)

def joint_state_listener():
    while not rospy.is_shutdown():
        rospy.Subscriber("/spyndra/joint_states", JointState, callback)
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    rospy.init_node('motor_state_publisher', anonymous=True)

    motor_state_publisher = rospy.Publisher('motor_state', MotorSignal, queue_size=5)

    try: joint_state_listener()
    except rospy.ROSInterruptException: pass