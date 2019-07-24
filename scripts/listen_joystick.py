#!/usr/bin/env python
# from ros wiki for initial testing

import rospy
from sensor_msgs.msg import Joy
joybuttons=[]
joyaxis=[]


def listen_joystick():
    rospy.Subscriber('joy', Joy, callback)


def callback(data):
    global joybuttons, joyaxis
    joybuttons = data.buttons
    joyaxis = data.axis
    print('but', joybuttons)
    print('axe', joyaxis)
    return


def get_joystick():
    return joybuttons, joyaxis


if __name__ == '__main__':
    rospy.init_node('kite_main', anonymous=False)
    listen_joystick()
    rospy.spin()
