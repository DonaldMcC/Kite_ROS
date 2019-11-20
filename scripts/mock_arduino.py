#!/usr/bin/env python
# this should receive motor msg which is currenlty just left or right based on 299 or 199 or stop at 0
# however was trying to send back an actual angle and it shouldn't
# it should send back the actual resistance

import time, math
import rospy
from std_msgs.msg import String, Int16
from kite_funcs import getangle, getresist

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
    """This now attempts to simulate how we believe the bar should responde to messages sent to
    the actuator given known distance from 'fulcrum' to mounting points and speed of the actuator.
    Motorvalue is received for left and right and resistance is sent back as kitenagle message."""

    resistleft = 340
    resistright = 740
    global motorvalue
    pub = rospy.Publisher('kiteangle', Int16, queue_size=3)
    rospy.init_node('mock_arduino', anonymous=False)
    bar_speed = 500

    DIST_ACT = 35.0   # mm
    DIST_HANDLE = 350.0  # mm
    SPEED_ACT = 30.0  # mm/sec
    FORCE_ACT = 200 # N but not sure if will actually use this
    CIRC_ACT = 2 * math.pi * DIST_ACT

    loop_time = time.time()
    rate = rospy.Rate(5)  # 5hz
    listen_motormsg()
    while not rospy.is_shutdown():
        get_motorv()

        elapsed_time = time.time() - loop_time
        loop_time = time.time()
        if motorvalue:
            if motorvalue < 200:
                act_dist = 0 - (SPEED_ACT * elapsed_time)
            else:
                act_dist = SPEED_ACT * elapsed_time
            anglechange = (360 * act_dist) / CIRC_ACT
            # left_act_pos = get_coord(left_act_pos[0], left_act_pos[1], anglechange)
            barangle += anglechange

        resistance = getresist(barangle)
        if resistance < resistleft:
            resistance = resistleft
        if resistance > resistright:
            resistance = resistright

        print('motorv', motorvalue, barangle, resistance)
        rospy.loginfo(resistance)
        pub.publish(resistance)
        rate.sleep()
    return


def test_kiteangle(barangle):
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
    aim for 35cm of dist_handle and 3.5cm of dist_act as starting point as seems to make the maths easy so got 2 * 200N
    of force which I think is enough as we are only changing the bar angle the main kite pull force should still be on
    the frame

    So above is simple arithmetic but bar_angle is slightly more tricky as dist_act will be constrained to an arc and
    also got problem of not breaking lever which makes wider spacing tempting but I think too slow - probably we can
    aim to stop at certain bar angles but this relies on sensor forces presumably change a bit outside centre of arc
    but I think work on centre span needs first

    """
    global motorvalue
    pub = rospy.Publisher('kiteangle', Int16, queue_size=3)
    rospy.init_node('mock_arduino', anonymous=False)
    rate = rospy.Rate(5)  # 5hz
    motorvalue = 100 #   so always go left to start with

    DIST_ACT = 35.0   # mm
    DIST_HANDLE = 350.0  # mm
    SPEED_ACT = 30.0  # mm/sec
    FORCE_ACT = 200 # N but not sure if will actually use this

    CIRC_ACT = 2 * math.pi * DIST_ACT
    # left_act_pos = get_coord(0-DIST_ACT, 0, barangle) not convinced this serves purpose
    loop_time = time.time()

    while not rospy.is_shutdown():
        print('motorv', motorvalue)
        elapsed_time = time.time() - loop_time
        loop_time = time.time()
        if motorvalue:
            if motorvalue < 200:
                act_dist = 0 - (SPEED_ACT * elapsed_time)
            else:
                act_dist = SPEED_ACT * elapsed_time
            anglechange = (360 * act_dist) / CIRC_ACT
            # left_act_pos = get_coord(left_act_pos[0], left_act_pos[1], anglechange)
            barangle += anglechange
        # should then oscillate
        if barangle < -40:
            motorvalue = 250
        elif barangle > 40:
            motorvalue = 100
        resistance = getresist(barangle)
        rospy.loginfo(barangle)
        pub.publish(resistance)
        print(time.time(), barangle, resistance)
        rate.sleep()


if __name__ == '__main__':
    try:
        kiteangle(0)
        # test_kiteangle(0) this was just for testing new approach
    except rospy.ROSInterruptException:
        pass
