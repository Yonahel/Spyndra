#!/usr/bin/env python
import rospy
from time import sleep
import sys
from spyndra import marvelmind
from spyndra.msg import BeaconPos


def main():
    # setting up the node
    rospy.init_node("gps_node", anonymous=True)
    pub_gps = rospy.Publisher('/gps/data', BeaconPos, queue_size=1)

    beacon = marvelmind.MarvelmindHedge(tty = "/dev/ttyACM1", adr=10, debug=False) # create MarvelmindHedge thread
    beacon.start() # start thread
    while not rospy.is_shutdown():
        sleep(.1)
        # beacon.position() last position as list: [usnAdr, usnX, usnY, usnZ, usnTimestamp]
        pos_msg = BeaconPos()
        pos = beacon.position()
        pos_msg.timestamp_ms = pos[4]
        pos_msg.address = pos[0]
        pos_msg.x_m = pos[1]
        pos_msg.y_m = pos[2]
        pos_msg.z_m = pos[3]
        pub_gps.publish(pos_msg)
        rospy.loginfo(str(beacon.position()))

    print 'stop beacon'
    beacon.stop()  # stop and close serial port
            
            # sys.exit()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass