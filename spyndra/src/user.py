#!/usr/bin/env python
import rospy
import getch
from std_msgs.msg import String
import sys
import time

def user():
    # user control node, for user to input commands
    rospy.init_node('user', anonymous=True)
    pub = rospy.Publisher('user_cmd', String, queue_size=10)
    # here we use keyboard inputs 
    # keyboard '0': cmd_0
    # keyboard "1": cmd_1
    # keyboard "2": cmd_2
    # keyboard "3": cmd_3
    # keyboard "4": cmd_4

    # when user types key via commandline arg
    # notice: a bad desciptor problem with unknown reason
    if len(sys.argv) == 2:
        key = sys.argv[1]
        if key == '0': 
            pub.publish('cmd_0')
        elif key == '1': #space
            pub.publish("cmd_1")
        elif key == '2':
            pub.publish("cmd_2")
        elif key == '3':
            pub.publish("cmd_3")
        elif key == '4':
            pub.publish("cmd_4")
    else:
        # when user types key via prompt (stdin)
        key = raw_input('Enter command (0-3):')
        if key == '0': 
            pub.publish('cmd_0')
        elif key == '1': #space
            pub.publish("cmd_1")
        elif key == '2':
            pub.publish("cmd_2")
        elif key == '3':
            pub.publish("cmd_3")
        elif key == '4':
            pub.publish("cmd_4")
    logmsg = 'pressed '+ str(key)
    rospy.loginfo(logmsg)

if __name__ == '__main__':
    try:
        user()
    except rospy.ROSInterruptException:
        pass
