#!/usr/bin/env python
# this should generally receive motor msg which is currently just left or right based on 3 or 4 respectively
# or stop at 0.  1 and 2 are forward and backwards for initialisation - the purpose of this was initially to allow
# test of the setup without any actual arduino hardware and that should be provided by the kiteangle function
# testkiteangle is similar and possibly now redundant but both were receiving motormsg published by bascic_motion_det.
# What we now also want to do is verif that our simulation and actual operation are aligned this seems to need a
# different approach in two ways
# 1 We generate the motor_msg more simply - and possibly without the analysis
# 2 the mock_arduino process returns a different message from the actual one and we can ideally compare them and
# 3 understand if performance is as expected

"""Our setup is currently 2 actuators that should move in opposite directions and a central guide rail
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


import time
import math
import rospy
import argparse
from std_msgs.msg import String, Int16
from kite_funcs import getangle, getresist
motorvalue = 3
barangle = 0
MAXLEFT = -20  # These are to simulate limits of angles
MAXRIGHT = 20  # similarly to protect bar as attached close to pivot

DIST_ACT = 35.0  # mm
DIST_HANDLE = 350.0  # mm
SPEED_ACT = 30.0  # mm/sec
FORCE_ACT = 200  # N but not sure if will actually use this
CIRC_ACT = 2 * math.pi * DIST_ACT


def listen_motormsg():
    rospy.Subscriber('motormsg', Int16, callback)


def callback(data):
    global motorvalue
    motorvalue = data.data
    return


def get_motorv():
    global motorvalue
    return motorvalue


def mock_kiteangle(message):
    global motorvalue
    global barangle
    pub = rospy.Publisher(message, Int16, queue_size=3)
    rospy.init_node('mock_arduino', anonymous=False)
    rate = rospy.Rate(10)  # 5hz

    # left_act_pos = get_coord(0-DIST_ACT, 0, barangle) not convinced this serves purpose
    loop_time = time.time()
    listen_motormsg()

    while not rospy.is_shutdown():
        get_motorv()
        print('motorv', motorvalue)
        elapsed_time = time.time() - loop_time
        loop_time = time.time()
        barangle = mockangle(barangle, elapsed_time)#
        resistance = get_resistance(barangle)
        rospy.loginfo(barangle)
        pub.publish(resistance)
        print(elapsed_time, barangle, resistance)
        rate.sleep()


def mockangle(angle, elapsed_time):
    """This now attempts to simulate how we believe the bar should respond to messages sent to
    the actuator given known distance from 'fulcrum' to mounting points and speed of the actuator.
    Motorvalue is received for left and right and resistance is sent back as kiteangle message."""
    global motorvalue

    get_motorv()
    print(angle)
    if motorvalue:
        if motorvalue == 1 or motorvalue == 2:
            angle = 0
        else:
            if motorvalue == 3:  # Left
                act_dist = 0 - (SPEED_ACT * elapsed_time)
                print(act_dist)
            elif motorvalue == 4:  # Right
                act_dist = SPEED_ACT * elapsed_time
            anglechange = (360 * act_dist) / CIRC_ACT
            print(anglechange)
            angle += anglechange
    if angle <= MAXLEFT:
        angle = MAXLEFT
    elif angle >= MAXRIGHT:
        angle = MAXRIGHT

    return angle

def get_resistance(barangle):
    resistleft = 340
    resistright = 740
    resistance = getresist(barangle)
    if resistance < resistleft:
        resistance = resistleft
    elif resistance > resistright:
        resistance = resistright
    return resistance

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('-m', '--message', type=str, default='kiteangle',
                            help='message to generate either kiteangle or mockangle')
        args = parser.parse_args()
        new_angle = mock_kiteangle(args.message)
        #mock_kiteangle(0, 'kiteangle')
        #kiteangle(0)
        # test_kiteangle(0) this was just for testing new approach
    except rospy.ROSInterruptException:
        pass
