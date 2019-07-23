#!/usr/bin/env python
# from ros wiki for initial testing

import rospy
from std_msgs.msg import Int16
joybuttons=[]
joyaxis=[]


def callback(data):
    global joybuttons, joyaxis
    resistance = data.data
    print(joybuttons)
    return


def listen_joystick():
    rospy.Subscriber('joy', Int16, callback)


def get_joystick():
    return joybuttons, joyaxis


if __name__ == '__main__':
    rospy.init_node('kite_main', anonymous=False)
    listen_joystick()
    rospy.spin()
