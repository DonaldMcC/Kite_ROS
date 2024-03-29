#!/usr/bin/env python
# from ros wiki for initial testing
# this should just mockup some values into the kite_arduino message to confirm that the kitebar process works
# this is not currently working but not required arduino is now just measuring kiteangle - however left
# in case I need to come back to custom arduino message for this
import rospy
from kite_ros.msg import Kite_arduino


def kite_arduino(rleft, rcent, rright, heading, varx, vary, varz):
    pub = rospy.Publisher('kite_arduino', Kite_arduino, queue_size=3)
    rospy.init_node('kite_arduino', anonymous=True)
    rate = rospy.Rate(5)  # 5hz
    msg = Kite_arduino()
    msg.rleft = rleft
    msg.rcent = rcent
    msg.rright = rright
    msg.heading = heading
    msg.varx = varx
    msg.vary = vary
    msg.varz = varz
    inc = 1
    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
        msg.rcent += inc
        if msg.rcent >= 1005:
            msg.rcent = 674
    return


if __name__ == '__main__':
    try:
        kite_arduino(100, 674, 45, 1, 0, 0, 0)
    except rospy.ROSInterruptException:
        pass