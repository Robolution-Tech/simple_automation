#!/usr/bin/env python

import sys, select, tty, termios
import rospy
from std_msgs.msg import String


if __name__=='__main__':
    key_pub=rospy.Publisher('keys',String,queue_size=1)
    rospy.init_node("keyboard_driver")
    rate=rospy.Rate(50)
    old_att=termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    print("publishing teystrokes. press ctrl-c to exit...")
    while not rospy.is_shutdown():
        if select.select([sys.stdin],[],[],0)[0]==[sys.stdin]:
            print(sys.stdin.read(1))
            key_pub.publish(sys.stdin.read(1))
            rate.sleep()
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old_attr)
