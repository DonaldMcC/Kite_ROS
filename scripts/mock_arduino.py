#!/usr/bin/env python
# this should receive motor msg which is currenlty just left or right based on 299 or 199 or stop at 0
# however was trying to send back an actual angle and it shouldn't
# it should send back the actual resistance

import time
import rospy
from std_msgs.msg import String, Int16
from kite_funcs import getangle
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
    bar_speed = 500

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
        rate.sleep()
    return

def test_kiteangle():
    """So our setup is currently 2 actuators that should move in opposite directions and a central guide rail
    for a wooden bar which will hold the kite by means of putting velcro on the handles - so basically just a
    lever with short distance to the actuator - using ones designed for automatic doors and longer distance
    to the kite handle.

    Actuator currently in use is https://www.ebay.co.uk/itm/303125148840

    dist_act will be from fulcrum to actuator
    dist_handle will be from fulcrum to kite handle
    speed_act is speed of actuator in mm per second 30mm/sec is current setup
    force_act is max force of actuator (without leverage) 200N in my case
    speed_handle is speed of handle in meters per second which I would like to be at least 30cm per sec as and lets
    aim for 50cm of dist_handle and 5cm of dist_act as starting point as seems to make the maths easy so got 2 * 200N
    of force which I think is enough as we are only changing the bar angle the main kite pull force should still be on
    the frame

    So above is simple arithmetic but bar_angle is slightly more tricky as dist_act will be constrained to an arc and
    also got problem of not breaking lever which makes wider spacing tempting but I think too slow - probably we can
    aim to stop at certain bar angles but this relies on sensor forces presumably change a bit outside centre of arc
    but I think work on centre span needs first

    """

    rospy.init_node('mock_arduino', anonymous=False)
    bar_speed = 500
    rate = rospy.Rate(5)  # 5hz
    while not rospy.is_shutdown():
        print(time.time())
        rate.sleep()



if __name__ == '__main__':
    try:
        # kiteangle(0) will revert to this once working again
        test_kiteangle()
    except rospy.ROSInterruptException:
        pass
