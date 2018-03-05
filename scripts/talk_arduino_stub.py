#!/usr/bin/env python
# from ros wiki for initial testing
import rospy
from std_msgs.msg import String
from kite_ros.msg import Kitepos
from Kite_arduino.msg import Kite_arduino


def kite_arduino(posx, posy, kiteangle, dirx, diry, routepoints, priorpos):
    pub = rospy.Publisher('custom_chatter', Kite_arduino, queue_size=10)
    rospy.init_node('custom_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    msg = Kite_arduino()
    msg.name = "Kite Position"
    msg.posx = posx
    msg.posy = posy
    msg.kiteangle = kiteangle
    msg.dirx = dirx
    msg.diry = diry
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