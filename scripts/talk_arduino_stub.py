#!/usr/bin/env python
# from ros wiki for initial testing
import rospy
from kite_ros.msg import Kite_arduino


def kite_arduino(posx, posy, kiteangle, dirx, diry, routepoints, priorpos):
    pub = rospy.Publisher('custom_chatter', Kite_arduino, queue_size=10)
    rospy.init_node('custom_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    msg = Kite_arduino()
    #msg.posx=100
    msg.rleft = 100
    msg.rcent = 150
    msg.rright = 200
    msg.heading = 150
    msg.varx = 30
    msg.vary = 60
    msg.varz = 90
    while not rospy.is_shutdown():
    #    hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
    return


if __name__ == '__main__':
    # talker()
    try:
        kite_arduino(100, 200, 45, 1, 0, 0, 0)
    except rospy.ROSInterruptException:
        pass