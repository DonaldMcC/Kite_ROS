#!/usr/bin/env python
# from ros wiki for initial testing
import rospy
from std_msgs.msg import String, Int16
from kite_ros.msg import Kitepos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
pub=0


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
    pub = rospy.Publisher('kite_position', Kitepos, queue_size=10)
    msg = Kitepos()
    msg.name = "Kite Position"
    msg.posx = posx
    msg.posy = posy
    msg.kiteangle = kiteangle
    msg.dirx = dirx
    msg.diry = diry
    rospy.loginfo(msg)
    pub.publish(msg)
    return


def init_ros():
    rospy.init_node('kite_main', anonymous=True)


def init_motor_msg():
    global pub
    pub = rospy.Publisher('motormsg', Int16, queue_size=10)


def motor_msg(barangle, targetbarangle, tolerance=10, action=None):
    global pub
    if action:
        pub.publish(action) # 1 for forward and 2 for backward
        return
    diff = barangle - targetbarangle
    if abs(diff) < tolerance:
        pub.publish(0)
    elif diff > 0:
        pub.publish(3)   # Left
    else:
        pub.publish(4)   # Right
    return


class KiteImage:

    def __init__(self):
        self.image_pub = rospy.Publisher('kite_image', Image, queue_size=10)
        self.bridge = CvBridge()

    def pubimage(self, cv_image):
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    # talker()
    try:
        # kite_pos(100, 200, 45, 1, 0, 0, 0)
        rospy.init_node('kite_main', anonymous=False)
        init_motor_msg()
        motor_msg(200, 300)
    except rospy.ROSInterruptException:
        pass
