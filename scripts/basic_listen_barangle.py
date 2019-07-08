#!/usr/bin/env python
# from ros wiki for initial testing

import rospy
from std_msgs.msg import Int16
barangle = 0


def callback(data):
    global barangle
    barangle = data.data
    print(barangle)
    return


def listen_kiteangle():
    # In ROS, nodes are uniquely named. If two nodes with the same-
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('kiteangle_listen', anonymous=True)
    # pub = rospy.Publisher('kitebase_node', Kitebase, queue_size=10)
    # rospy.Subscriber('kitebase_node', Kitebase, callback)
    rospy.Subscriber('kiteangle', Int16, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


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
    listen_kiteangle()
