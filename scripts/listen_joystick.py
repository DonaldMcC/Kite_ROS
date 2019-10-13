#!/usr/bin/env python
# from ros wiki for initial testing

import rospy
from sensor_msgs.msg import Joy
joybuttons = []
joyaxes = []


def listen_joystick():
    rospy.Subscriber('xwiimote_node/joy', Joy, callback)


def callback(data):
    global joybuttons, joyaxes
    joybuttons = data.buttons
    joyaxes = data.axes
    return


def get_joystick():
    return joybuttons, joyaxes


if __name__ == '__main__':
    rospy.init_node('joy_listen', anonymous=False)
    listen_joystick()
    rospy.spin()
