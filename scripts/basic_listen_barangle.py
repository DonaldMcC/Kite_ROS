#!/usr/bin/env python
# this gets the barangle from the arduino board

import time
import rospy
from std_msgs.msg import Int16
from kite_funcs import getangle
from mainclasses import calcbarangle, inferangle
from talker import motor_msg
barangle = 0
resistance = 200
mockresistance = 200
mockangle = 0

def callback(data):
    global resistance
    resistance = data.data
    return


def callmock(data):
    global mockresistance
    mockresistance = data.data
    return


def listen_kiteangle(message):
    if message == 'kiteangle':
        rospy.Subscriber(message, Int16, callback, queue_size=1)
    else:
        rospy.Subscriber(message, Int16, callmock, queue_size=1)


def get_actmockangle(kite, base, control, config):
    global mockangle, mockresistance
    mockangle = getangle(resistance, base.maxleft, base.maxright,
                         base.resistleft, base.resistright, base.resistcentre)
    return mockangle


# this should always return barangle except when barangle being set from the kite for simulation
# or on manbar when bar should be freely controlled
def get_barangle(kite, base, control, config):
    global barangle, resistance
    if config.setup == 'KiteBarActual':
        return kite.kiteangle / base.kitebarratio
    else:  # automated flight reading from some sort of sensor via ROS
        barangle = getangle(resistance, base.maxleft, base.maxright,
                            base.resistleft, base.resistright, base.resistcentre)
        return barangle


def get_angles(kite, base, control, config):
    base.resistance = resistance
    base.barangle = get_barangle(kite, base, control, config)
    print('setr to ' + str(resistance))
    if config.setup == 'KiteBarTarget':
        base.targetbarangle = kite.kiteangle / base.kitebarratio
    else:
        base.targetbarangle = calcbarangle(kite, base, control)
    if config.setup == 'BarKiteActual':  # derive kite from bar
        kite.kiteangle = base.barangle * base.kitebarratio
    elif config.setup == 'KiteBarInfer':
        base.inferbarangle = inferangle(kite, base, control)
    return


if __name__ == '__main__':
    rospy.init_node('kite_main', anonymous=False)
    listen_kiteangle('kiteangle')
    rospy.spin()
