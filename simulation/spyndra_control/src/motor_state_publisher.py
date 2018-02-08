#!/usr/bin/env python
import rospy
import math
import sys
import time

# from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from spyndra.msg import MotorSignal

motor_state_publisher = None

def init_motor_state_publisher():
    #Initiate node for controlling joint positions.
    rospy.init_node('motor_state_publisher', anonymous=True)

	#Define publishers for each joint position controller commands.
    motor_state_publisher = rospy.Publisher('motor_state', MotorSignal, queue_size=5)

def callback(data):
    motor_signal = MotorSignal
    motor_signal.motor_type = 1
    motor_signal.signal = data.position
    motor_state_publisher.publish(motor_signal)

def joint_state_listener():
    # rospy.init_node('joint_state_listener', anonymous=True)

    rospy.Subscriber("/spyndra/joint_states", JointState, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
if __name__ == '__main__':
    init_motor_state_publisher()

    try: joint_state_listener()
    except rospy.ROSInterruptException: pass