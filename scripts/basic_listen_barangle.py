#!/usr/bin/env python
# this gets the barangle from the arduino board and now also got an initialisation which
# should retract the actuators

import time, math
import rospy
from std_msgs.msg import Int16
from kite_funcs import getangle
from mainclasses import calcbarangle, inferangle
from talker import motor_msg
barangle = 0
resistance = 0
mockangle = 0


def callback(data):
    global barangle, resistance
    resistance = data.data
    barangle = getangle(resistance)
    return


def callmock(data):
    global mockangle, resistance
    resistance = data.data
    mockangle = getangle(resistance)
    return


def listen_kiteangle(message):
    if message == 'kiteangle':
        rospy.Subscriber(message, Int16, callback)
    else:
        rospy.Subscriber(message, Int16, callmock)


def get_actmockangle():
    global mockangle
    return mockangle


# this should always return barangle except when barangle being set from the kite for simulation
def get_barangle(kite, base, control, config):
    global barangle
    if config.setup == 'KiteBarActual':
        return kite.kiteangle / base.kitebarratio
    else:  # automated flight reading from some sort of sensor via ROS
        return barangle


def get_angles(kite, base, control, config):
    base.barangle = get_barangle(kite, base, control, config)
    base.resistance = resistance
    if config.setup == 'KiteBarTarget':
        base.targetbarangle = kite.kiteangle / base.kitebarratio
    else:
        base.targetbarangle = calcbarangle(kite, base, control)
    if config.setup == 'BarKiteActual':  # derive kite from bar
        kite.kiteangle = base.barangle * base.kitebarratio
    elif config.setup == 'KiteBarInfer':
        base.inferbarangle = inferangle(kite, base, control)
    return


def check_kite(kite, base, control, config):
    # this will now generally be called when motion_detection starts - it will do the following things
    # fully retract both actuators and once done confirm barangle is approximately zero
    tolerance = 10
    reset_bar(base)
    actangle = get_barangle(kite, base, control, config)
    if abs(actangle) < tolerance:
        return "OK"
    else:
        return "Out of Tolerance "


def reset_bar(base):
    global resistance
    max_retract_time = 5
    motor_msg(0, 0, 0, 1, 1)  # send backward signal
    time.sleep(max_retract_time)  # assumed to be time for motors to fully retract
    motor_msg(0, 0, 0, 5, 1)  # stop
    base.reset=False
    return resistance


if __name__ == '__main__':
    rospy.init_node('kite_main', anonymous=False)
    listen_kiteangle('kiteangle')
    rospy.spin()
