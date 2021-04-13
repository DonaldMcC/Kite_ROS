#!/usr/bin/env python3
# from ros wiki for initial testing
import rospy
from std_msgs.msg import String, Int16
from kite_ros.msg import Kitepos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
pub = 0


def kite_pos(posx, posy, kiteangle, dirx, diry, routepoints, priorpos):
    pub = rospy.Publisher('kite_position', Kitepos, queue_size=10)
    msg = Kitepos()
    msg.name = "Kite Position"
    msg.posx = posx
    msg.posy = posy
    msg.kiteangle = kiteangle
    msg.dirx = dirx
    msg.diry = diry
    # rospy.loginfo(msg)
    pub.publish(msg)
    return


def init_ros():
    rospy.init_node('kite_main', anonymous=True)


def init_motor_msg():
    global pub
    pub = rospy.Publisher('motormsg', Int16, queue_size=10)


def motor_msg(action):
    #print('action:', str(action))
    pub.publish(action)
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
    try:
        rospy.init_node('kite_main', anonymous=False)
        init_motor_msg()
        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            motor_msg(0)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
