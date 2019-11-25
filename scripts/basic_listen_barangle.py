#!/usr/bin/env python
# from ros wiki for initial testing

import time
import rospy
from std_msgs.msg import Int16
from kite_funcs import getangle
from talker import motor_msg
barangle = 0


def callback(data):
    global barangle
    resistance = data.data
    barangle = getangle(resistance)
    return


def listen_kiteangle():
    print('initing')
    rospy.Subscriber('kiteangle', Int16, callback)

def get_actbarangle():
    global barangle
    return barangle

# this should always return barangle for Manbar or Standard operation Manfly should set
def get_barangle(kite, base, control):
    global barangle
    print ('barangle', barangle)
    if control.config == 'Manfly' :
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
    MAX_RETRACT_TIME=10
    TOLERANCE = 10
    motor_msg(0,0,0,2) # send backward signal
    time.sleep(MAX_RETRACT_TIME) # assumed to be time for motors to fully retract
    motor_msg(0,0,0,0) # stop
    actangle = get_barangle(kite, base, control)
    if abs(actangle) < TOLERANCE:
        return "OK"
    else:
        return "Out of Tolerance "




if __name__ == '__main__':
    rospy.init_node('kite_main', anonymous=False)
    listen_kiteangle()
    rospy.spin()
