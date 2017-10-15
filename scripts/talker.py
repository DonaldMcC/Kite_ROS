#!/usr/bin/env python
# from ros wiki for initial testing
import rospy
from std_msgs.msg import String
from kite_ros.msg import Kitepos


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
    return


def kite_pos(posx, posy, kiteangle, dirx, diry, routepoints, priorpos):
    pub = rospy.Publisher('custom_chatter', Kitepos, queue_size=10)
    rospy.init_node('custom_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    msg = Kitepos()
    msg.name = "Kite Position"
    msg.posx = posx
    msg.posy = posy
    msg.kiteangle = kiteangle
    msg.dirx = dirx
    msg.diry = diry

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
    return


if __name__ == '__main__':
    # talker()
    try:
        kite_pos(100,200,45,1,0)
    except rospy.ROSInterruptException:
        pass
