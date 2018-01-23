#!/usr/bin/env python
import rospy
import getch
from std_msgs.msg import String

def user():
    # user control node, for user to input command
    rospy.init_node('user', anonymous=True)
    pub = rospy.Publisher('user_cmd', String, queue_size=10)
    while True:
        # here we use keyboard inputs 
        # keyboard "1": cmd_1
        # keyboard "2": cmd_2
        # keyboard "3": cmd_3
        # keyboard "4": cmd_4
        key = ord(getch.getch())
        if key == 49: #space
            pub.publish("cmd_1")
            rospy.loginfo(key)
        elif key == 50:
            pub.publish("cmd_2")
            rospy.loginfo(key)
        elif key == 51:
            pub.publish("cmd_3")
            rospy.loginfo(key)
        elif key == 52:
            pub.publish("cmd_4")
            rospy.loginfo(key)

if __name__ == '__main__':
    try:
        user()
    except rospy.ROSInterruptException:
        pass
