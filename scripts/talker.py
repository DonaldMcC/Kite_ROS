#!/usr/bin/env python
# from ros wiki for initial testing
import rospy
from std_msgs.msg import String, Int16
from kite_ros.msg import Kitepos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
pub=0


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


def motor_msg(barangle, targetbarangle, tolerance=10, action=None, doaction=False, speed=100):
    # Now added ability to send motor message 6 for leftonly and 7 for rightonly
    # and will now add speed into the message as %age of max value up to 99 but 0 is max speed
    MAXLEFT = -20  # These are to try and avoid breaking the bar
    MAXRIGHT = 20  # similarly to protect bar as attached close to pivot
    msg = 0
    if doaction:
        msg = action  # 100 for forward and 200 for backward - will now execute 0 as doaction drives but
    else:
        diff = barangle - targetbarangle
        if abs(diff) < tolerance:
            msg = 0 # stop
        elif diff > 0 and barangle > MAXLEFT:
            msg = 300   # Left
        elif diff < 0 and barangle < MAXRIGHT:
            msg = 400  # Right
    msg = int(msg + speed) if 0 < speed < 100 else int(msg)
    pub.publish(msg)
    return msg


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
            motor_msg(0, 0, 0, 0, 1)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
