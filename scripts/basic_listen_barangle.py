#!/usr/bin/env python
# from ros wiki for initial testing

import rospy
from std_msgs.msg import Int16
from kite_funcs import getangle
barangle = 0


def callback(data):
    global barangle
    resistance = data.data
    barangle = getangle(resistance)
    print(barangle)
    return


def listen_kiteangle():
    rospy.Subscriber('kiteangle', Int16, callback)


def get_barangle(kite, base, control):
    global barangle
    if control.config == 'Manfly':
        if base.updatemode < 2:
            return base.barangle
        else:  # when kiteangle is driving the barangle
            # TO DO add kitebar ratio to this
            return kite.kiteangle
    else:  # automated flight reading from some sort of sensor via ROS
        return barangle


if __name__ == '__main__':
    rospy.init_node('kiteangle_listen', anonymous=True)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    listen_kiteangle()
