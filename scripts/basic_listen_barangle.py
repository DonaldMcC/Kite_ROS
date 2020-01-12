#!/usr/bin/env python
# this gets the barangle from the arduino board and now also got an initialisation which
# should retract the actuators

import time
import rospy
from std_msgs.msg import Int16
from kite_funcs import getangle
from talker import motor_msg
barangle = 0
mockangle = 0

def callback(data):
    global barangle
    resistance = data.data
    barangle = getangle(resistance)
    return

def callmock(data):
    global mockangle
    resistance = data.data
    mockangle = getangle(resistance)
    return


def listen_kiteangle(message):
    if message == 'kiteangle':
        rospy.Subscriber(message, Int16, callback)
    else:
        rospy.Subscriber(message, Int16, callmock)


def get_actbarangle():
    global barangle
    return barangle

def get_actmockangle():
    global mockangle
    return mockangle


# this should always return barangle for Manbar or Standard operation Manfly should set
def get_barangle(kite, base, control):
    global barangle
    #  print('barangle', barangle)
    if control.config == 'Manfly' or control.config == 'Manbar':
        if base.updatemode == 'Manbar':
            return base.barangle
        else:  # when kiteangle is driving the barangle
            # TO DO add kitebar ratio to this
            return kite.kiteangle / base.kitebarratio
    else:  # automated flight reading from some sort of sensor via ROS
        return get_actbarangle()


def check_kite(kite, base, control):
    # this will now generally be called when motion_detection starts - it will do the following things
    # fully retract both actuators and once done confirm barangle is approximately zero
    MAX_RETRACT_TIME = 10
    TOLERANCE = 10
    motor_msg(0, 0, 0, 1)  # send backward signal
    time.sleep(MAX_RETRACT_TIME)  # assumed to be time for motors to fully retract
    motor_msg(0, 0, 0, 5)  # stop
    actangle = get_barangle(kite, base, control)
    if abs(actangle) < TOLERANCE:
        return "OK"
    else:
        return "Out of Tolerance "


if __name__ == '__main__':
    rospy.init_node('kite_main', anonymous=False)
    listen_kiteangle('kiteangle')
    rospy.spin()
