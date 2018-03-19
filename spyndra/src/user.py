#!/usr/bin/env python
import rospy
import getch
from std_msgs.msg import String

def user():
    # user control node, for user to input commands
    rospy.init_node('user', anonymous=True)
    pub = rospy.Publisher('user_cmd', String, queue_size=0)
    while True:
        # here we use keyboard inputs 
        # keyboard '0': cmd_0
        # keyboard "1": cmd_1
        # keyboard "2": cmd_2
        # keyboard "3": cmd_3
        # keyboard "4": cmd_4
        key = ord(getch.getch())
        if key == 48: 
            pub.publish('cmd_0')
        elif key == 49: #space
            pub.publish("cmd_1")
        elif key == 50:
            pub.publish("cmd_2")
        elif key == 51:
            pub.publish("cmd_3")
        elif key == 52:
            pub.publish("cmd_4")
	logmsg = 'pressed '+ str(key)
        rospy.loginfo(logmsg)

if __name__ == '__main__':
    try:
        user()
    except rospy.ROSInterruptException:
        pass
