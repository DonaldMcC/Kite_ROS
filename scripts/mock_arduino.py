#!/usr/bin/env python
# from ros wiki for initial testing

import rospy
import rospy
from std_msgs.msg import String, Int16
pub=0


def listen_motormsg():
    rospy.Subscriber('motor_msg', Int16, callback)

def callback(data):
    global joybuttons, joyaxes
    joybuttons = data.buttons
    joyaxes = data.axes
    return


def init_ros():
    rospy.init_node('mock_arduino', anonymous=True)


def init_kiteangle():
    global pub
    pub = rospy.Publisher('kiteangle', Int16, queue_size=10)


def kiteangle(barangle):
    global pub
    pub.publish(barangle)
    return


if __name__ == '__main__':
    # talker()
    try:
        # kite_pos(100, 200, 45, 1, 0, 0, 0)
        rospy.init_node('mock_arduino', anonymous=False)
        init_kiteangle()
        kiteangle(0)
    except rospy.ROSInterruptException:
        pass
