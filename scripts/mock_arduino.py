#!/usr/bin/env python
# from ros wiki for initial testing

import rospy
from std_msgs.msg import String, Int16
motorvalue = 0


def listen_motormsg():
    rospy.Subscriber('motormsg', Int16, callback)


def callback(data):
    global motorvalue
    motorvalue = data.data
    return


def get_motorv():
    global motorvalue
    return motorvalue

def kiteangle(barangle):
    resistleft = 340
    resistright = 740
    global motorvalue
    pub = rospy.Publisher('kiteangle', Int16, queue_size=3)
    rospy.init_node('mock_arduino', anonymous=False)
    bar_speed = 50
    rate = rospy.Rate(5)  # 5hz
    listen_motormsg()
    while not rospy.is_shutdown():
        get_motorv()
        print('motorv', motorvalue)
        rospy.loginfo(barangle)
        pub.publish(barangle)
        rate.sleep()
        if motorvalue:
            if motorvalue < 200:
                barangle = barangle - (bar_speed/100.0)
            else:
                barangle = barangle + (bar_speed/100.0)
        if barangle < resistleft:
            barangle = resistleft
        if barangle > resistright:
            barangle = resistright
    return


if __name__ == '__main__':
    try:
        kiteangle(0)
    except rospy.ROSInterruptException:
        pass